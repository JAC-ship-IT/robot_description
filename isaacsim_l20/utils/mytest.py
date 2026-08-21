#!/usr/bin/env python3
"""Train PPO on the Linker Hand Isaac Lab grasp environment with Stable-Baselines3.

Must launch Omniverse via AppLauncher before importing isaaclab.

Example:
    python scripts/training/train_grasp_issac.py --headless --n-envs 64
    python scripts/training/train_grasp_issac.py --headless --n-envs 64 \\
        --base-model-path runs/grasp_64env_42/final_model.zip \\
        --base-vec-normalize-path runs/grasp_64env_42/vec_normalize.pkl
"""

from __future__ import annotations

import argparse
import contextlib
import gc
import os
import signal
import sys
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train Linker Hand grasping in Isaac Lab with PPO.")
parser.add_argument("--n-envs", type=int, default=64, help="Parallel Isaac Lab environments.")
parser.add_argument("--total-timesteps", type=int, default=50_000_000)
parser.add_argument("--learning-rate", type=float, default=3e-4)
parser.add_argument("--batch-size", type=int, default=4096)
parser.add_argument("--n-steps-per-env", type=int, default=128)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--base-model-path", type=str, default=None, help="Optional PPO checkpoint to fine-tune.")
parser.add_argument(
    "--base-vec-normalize-path",
    type=str,
    default=None,
    help="Optional VecNormalize stats to load before training.",
)
parser.add_argument("--no-wandb", action="store_true", help="Disable Weights & Biases logging.")
parser.add_argument("--run-name", type=str, default=None, help="Override run directory / wandb name.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


def _cleanup_pbar(*_args) -> None:
    tqdm_objects = [obj for obj in gc.get_objects() if "tqdm" in type(obj).__name__]
    for tqdm_object in tqdm_objects:
        if "tqdm_rich" in type(tqdm_object).__name__:
            tqdm_object.close()
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, _cleanup_pbar)

# --------------------------------------------------------------------------- #
# Imports that require a running Omniverse app
# --------------------------------------------------------------------------- #
import torch
import wandb
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.vec_env import VecNormalize
from wandb.integration.sb3 import WandbCallback

from isaaclab_rl.sb3 import Sb3VecEnvWrapper

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from linker_hand.config import TrainConfig
from linker_hand.curriculum.callbacks import LossCallback, MetricsCallback
from linker_hand.isaac_env.grasp_env_cfg import Robot_arm_Cfg
from linker_hand.isaac_env.grasp_env_lab import Robot_arm_Env


def _unwrap_isaac_env(vec_env):
    env = vec_env
    while hasattr(env, "venv"):
        env = env.venv
    return env.env


class IsaacExtrasLogCallback(BaseCallback):
    """Log DirectRLEnv ``extras['log']`` scalars to wandb."""

    def __init__(self, log_every: int = 1000, verbose: int = 0):
        super().__init__(verbose)
        self.log_every = log_every

    def _on_step(self) -> bool:
        if self.num_timesteps % self.log_every != 0:
            return True
        isaac_env = _unwrap_isaac_env(self.model.get_env())
        log_dict = isaac_env.extras.get("log", {})
        if not log_dict:
            return True
        payload = {}
        for key, value in log_dict.items():
            if torch.is_tensor(value):
                payload[key] = float(value.mean().item())
            elif isinstance(value, (float, int)):
                payload[key] = value
        if payload:
            wandb.log(payload, step=self.num_timesteps)
        return True


def build_train_config() -> TrainConfig:
    return TrainConfig(
        n_envs=args_cli.n_envs,
        total_timesteps=args_cli.total_timesteps,
        learning_rate=args_cli.learning_rate,
        batch_size=args_cli.batch_size,
        n_steps_per_env=args_cli.n_steps_per_env,
        seed=args_cli.seed,
        base_model_path=args_cli.base_model_path,
        base_vec_normalize_path=args_cli.base_vec_normalize_path,
    )


def make_isaac_vec_env(config: TrainConfig):
    env_cfg = Robot_arm_Cfg()
    env_cfg.scene.num_envs = config.n_envs
    env_cfg.seed = config.seed
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    env = Robot_arm_Env(cfg=env_cfg)
    return Sb3VecEnvWrapper(env)


def train(config: TrainConfig) -> None:
    run_name = args_cli.run_name or f"grasp_isaac_{config.n_envs}env_{config.seed}"
    run_dir = Path("runs") / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    rollout_size = config.n_envs * config.n_steps_per_env
    if config.batch_size > rollout_size:
        config.batch_size = rollout_size
        print(f"Clamped batch_size to {config.batch_size} (n_envs * n_steps_per_env)")

    if not args_cli.no_wandb:
        wandb.init(
            project="linker-hand",
            name=run_name,
            config=asdict(config),
        )

    vec_env = make_isaac_vec_env(config)

    if config.norm_obs or config.norm_reward:
        vec_env = VecNormalize(
            vec_env,
            norm_obs=config.norm_obs,
            norm_reward=config.norm_reward,
            clip_obs=10.0,
            gamma=config.gamma,
        )

    if config.base_vec_normalize_path and Path(config.base_vec_normalize_path).exists():
        if isinstance(vec_env, VecNormalize):
            vec_env = VecNormalize.load(config.base_vec_normalize_path, vec_env)
            vec_env.training = True
            vec_env.norm_reward = config.norm_reward
        else:
            print("Warning: base_vec_normalize_path provided but VecNormalize is disabled.")

    activation_fn = {"elu": torch.nn.ELU, "relu": torch.nn.ReLU, "tanh": torch.nn.Tanh}[config.activation]
    policy_kwargs = {
        "net_arch": dict(pi=config.pi.copy(), vf=config.vf.copy()),
        "activation_fn": activation_fn,
    }

    if config.base_model_path and Path(config.base_model_path).exists():
        model = PPO.load(
            config.base_model_path,
            env=vec_env,
            device=config.device,
            print_system_info=True,
        )
        print(f"Loaded base model from {config.base_model_path}")
    else:
        model = PPO(
            "MlpPolicy",
            vec_env,
            learning_rate=config.learning_rate,
            n_steps=config.n_steps_per_env,
            batch_size=config.batch_size,
            n_epochs=config.n_epochs,
            gamma=config.gamma,
            gae_lambda=config.gae_lambda,
            clip_range=config.clip_range,
            ent_coef=config.ent_coef,
            vf_coef=config.vf_coef,
            max_grad_norm=config.max_grad_norm,
            policy_kwargs=policy_kwargs,
            verbose=config.verbose,
            seed=config.seed,
            device=config.device,
            tensorboard_log=str(run_dir / "tb"),
        )

    callbacks = [
        CheckpointCallback(
            save_freq=max(500_000 // config.n_envs, 1),
            save_path=str(run_dir / "checkpoints"),
            name_prefix="ppo",
        ),
    ]
    if not args_cli.no_wandb:
        callbacks = [
            MetricsCallback(),
            LossCallback(),
            IsaacExtrasLogCallback(log_every=1000),
            *callbacks,
            WandbCallback(
                model_save_path=str(run_dir),
                model_save_freq=max(100_000 // config.n_envs, 1),
                verbose=1,
                log="all",
            ),
        ]

    with contextlib.suppress(KeyboardInterrupt):
        model.learn(
            total_timesteps=config.total_timesteps,
            callback=callbacks,
            progress_bar=True,
        )

    model.save(str(run_dir / "final_model"))
    if isinstance(vec_env, VecNormalize):
        vec_env.save(str(run_dir / "vec_normalize.pkl"))

    print(f"Saved to {run_dir}")
    if not args_cli.no_wandb:
        wandb.finish()

    vec_env.close()


def main() -> None:
    config = build_train_config()
    train(config)
    simulation_app.close()


if __name__ == "__main__":
    main()
