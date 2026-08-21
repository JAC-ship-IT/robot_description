"""Wandb observer for rl_games Runner.

Initializes wandb before rl_games' summary writer (so sync_tensorboard works),
and directly logs env metrics — reward terms, successes, etc. — like SB3's
IsaacExtrasLogCallback in `mytest.py`.
"""

from __future__ import annotations

import os
import time
from datetime import datetime

from isaacsim_l20.utils.reformat import omegaconf_to_dict
from isaacsim_l20.utils.rlgames_utils import EnvStatsAlgoObserver


class WandbAlgoObserver(EnvStatsAlgoObserver):
    """Init wandb + log env/PPO metrics to the W&B dashboard.

    Subclasses `EnvStatsAlgoObserver` so reward terms (`episode_cumulative/*`),
    episode outcomes (`episode_final/*`), and step scalars (e.g. `successes`,
    `current_success_tolerance`) are written to both TensorBoard and wandb.
    rl_games PPO losses/lr still flow through TensorBoard via `sync_tensorboard`.
    """

    def __init__(self, cfg):
        super().__init__()
        self.cfg = cfg

    def before_init(self, base_name, config, experiment_name):
        import wandb

        # rl_games' `experiment_name` is derived from `agent_cfg.params.config.name`
        # (yaml-side, pinned and prefixed with `0_` to satisfy its policy_idx parsing).
        # The CLI `--wandb_name` value lives on `self.cfg.wandb_name` — prefer that
        # if set so per-run sub-file `EXPERIMENT_NAME`s actually show up in wandb.
        chosen = getattr(self.cfg, "wandb_name", "") or experiment_name
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        wandb_unique_id = f"{chosen}_{timestamp}"
        display_name = f"{chosen}_{timestamp}"
        print(f"[Wandb] unique id: {wandb_unique_id}")

        cfg = self.cfg
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        default_logcode_dir = os.path.join(repo_root, "isaacsim_l20")
        logcode_dir = cfg.wandb_logcode_dir if cfg.wandb_logcode_dir else default_logcode_dir

        # Isaac Sim + Omniverse already use multiprocessing; forked wandb init
        # commonly deadlocks or leaves a half-open local stream.  Retrying
        # wandb.init() with the same run id then hits "run ID ... is in use"
        # (see wandb/run-*/logs/debug-core.log) and wandb.run stays None.
        print("[Wandb] initializing...")
        init_exc = None
        for attempt in range(3):
            run_id = wandb_unique_id if attempt == 0 else f"{wandb_unique_id}_r{attempt}"
            try:
                wandb.init(
                    project=cfg.wandb_project, 
                    entity=cfg.wandb_entity or None,
                    group=cfg.wandb_group or None,
                    tags=cfg.wandb_tags,
                    notes=cfg.wandb_notes if hasattr(cfg, "wandb_notes") else "",
                    sync_tensorboard=True,
                    id=run_id,
                    name=display_name if attempt == 0 else f"{display_name}_r{attempt}",
                    resume="never",
                    settings=wandb.Settings(start_method="thread"),
                )
                if logcode_dir and os.path.isdir(logcode_dir):
                    try:
                        wandb.run.log_code(root=logcode_dir)
                        print(f"[Wandb] run dir: {wandb.run.dir} (log_code root: {logcode_dir})")
                    except Exception as log_code_exc:
                        print(f"[Wandb] log_code skipped: {log_code_exc}")
                else:
                    print(f"[Wandb] run dir: {wandb.run.dir} (log_code skipped)")
                entity = wandb.run.entity
                project = wandb.run.project
                print(f"[Wandb] dashboard: https://wandb.ai/{entity}/{project}/runs/{wandb.run.id}")
                init_exc = None
                break
            except Exception as exc:
                init_exc = exc
                print(f"[Wandb] init attempt {attempt + 1}/3 failed: {exc}")
                time.sleep(min(2 ** attempt, 30))

        if init_exc is not None:
            print(f"[Wandb] init failed after retries: {init_exc}")

        if wandb.run is not None:
            # rl_games uses env frame count as the TensorBoard / training step.
            wandb.define_metric("frame")
            wandb.define_metric("*", step_metric="frame")

        if wandb.run is None:
            print("[Wandb] run is None — skipping diff + config upload.")
            return

        # Capture a git diff so the run is reproducible from HEAD.
        diff_path = os.path.join(wandb.run.dir, "diff.patch")
        with open(diff_path, "w") as f:
            os.system(f"cd {repo_root} && git diff > {f.name}")
        diff_artifact = wandb.Artifact("diff", type="file", description="Git diff")
        diff_artifact.add_file(diff_path)
        wandb.run.log_artifact(diff_artifact)

        cfg_dict = self.cfg if isinstance(self.cfg, dict) else omegaconf_to_dict(self.cfg)
        wandb.config.update(cfg_dict, allow_val_change=True)

    def after_print_stats(self, frame, epoch_num, total_time):
        metrics = self.gather_env_metrics()
        metrics["train/epoch"] = float(epoch_num)
        metrics["train/total_time_s"] = float(total_time)

        super().after_print_stats(frame, epoch_num, total_time)

        import wandb

        if wandb.run is None or not metrics:
            return
        wandb.log(metrics, step=int(frame))
