#!/usr/bin/env python3
"""MuJoCo workspace visualizer for Marvin + SimToolReal.

Loads ``scene.xml``, applies the training default arm pose, overlays:
  - cyan box: ``target_volume`` (goal pose sampling region — NOT object reset)
  - orange box: object reset sampling region (table_z + offset ± noise)
  - red marker: nominal object spawn center
  - green markers: random goal samples inside ``target_volume``
  - magenta markers: palm positions from uniform random arm joint samples (reach)
  - yellow marker: palm reference site at the default arm pose (``test``)

Usage (from repo root or this directory):

    python assets/urdf/marvin_left_sharpa/visualize_workspace_mujoco.py

    python assets/urdf/marvin_left_sharpa/visualize_workspace_mujoco.py \\
      --preset mujoco_scene --num-goals 40 --num-reach-samples 5000

    python assets/urdf/marvin_left_sharpa/visualize_workspace_mujoco.py \\
      --workspace-min -0.2 -0.2 0.45 --workspace-max 0.2 0.2 0.75 \\
      --table-z 0.38 --object-z-offset 0.25

Press Ctrl+C to exit the viewer.
"""

from __future__ import annotations

import argparse
import textwrap
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_SCENE = HERE / "scene.xml"

# Matches isaacsimenvs_marvin_sharpa/tasks/simtoolreal/utils/scene_utils.py
ARM_DEFAULT_JOINT_POS: dict[str, float] = {
    "dobot_joint_1": -0.879,
    "dobot_joint_2": 0.377,
    "dobot_joint_3": -1.63,
    "dobot_joint_4": -1.88,
    "dobot_joint_5": -1.0,
    "dobot_joint_6": -0.314,
}

# Matches isaacsimenvs_marvin_sharpa/cfg/task/SimToolReal.yaml (Marvin preset).
# target_volume_*: goal pose sampling bounds (env-local frame), not object reset.
MARVIN_ISAAC_TARGET_VOLUME_MIN = (-0.2, -0.2, 0.6)
MARVIN_ISAAC_TARGET_VOLUME_MAX = (0.2, 0.2, 0.85)
MARVIN_ISAAC_TABLE_Z = 0.38
MARVIN_ISAAC_OBJECT_Z_OFFSET = 0.25
MARVIN_ISAAC_OBJECT_RESET_NOISE_XY = 0.1
MARVIN_ISAAC_OBJECT_RESET_NOISE_Z = 0.02

ARM_JOINT_NAMES = tuple(f"dobot_joint_{i}" for i in range(1, 7))
PALM_SITE_NAME = "test"
TABLE_HALF_Z_M = 0.05  # table_narrow.urdf full height 0.1 m


def _parse_vec3(values: list[float], name: str) -> np.ndarray:
    if len(values) != 3:
        raise argparse.ArgumentTypeError(f"{name} expects three floats")
    return np.asarray(values, dtype=np.float64)


def _scaled_bounds(
    lo: np.ndarray, hi: np.ndarray, scale: float
) -> tuple[np.ndarray, np.ndarray]:
    center = 0.5 * (lo + hi)
    half = 0.5 * (hi - lo) * scale
    return center - half, center + half


def _set_joint_pos(model: mujoco.MjModel, data: mujoco.MjData, joint_pos: dict[str, float]) -> None:
    for name, value in joint_pos.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"Joint not found in model: {name}")
        qpos_adr = model.jnt_qposadr[joint_id]
        data.qpos[qpos_adr] = float(value)


def _site_pos(model: mujoco.MjModel, data: mujoco.MjData, site_name: str) -> np.ndarray:
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site_id < 0:
        raise ValueError(f"Site not found in model: {site_name}")
    return np.array(data.site_xpos[site_id], dtype=np.float64)


def _joint_limits(model: mujoco.MjModel, joint_names: tuple[str, ...]) -> tuple[np.ndarray, np.ndarray]:
    lo = np.empty(len(joint_names), dtype=np.float64)
    hi = np.empty(len(joint_names), dtype=np.float64)
    for idx, name in enumerate(joint_names):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"Joint not found in model: {name}")
        lo[idx] = float(model.jnt_range[joint_id, 0])
        hi[idx] = float(model.jnt_range[joint_id, 1])
    return lo, hi


def _set_arm_joint_vector(
    model: mujoco.MjModel, data: mujoco.MjData, joint_values: np.ndarray
) -> None:
    for idx, name in enumerate(ARM_JOINT_NAMES):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos_adr = model.jnt_qposadr[joint_id]
        data.qpos[qpos_adr] = float(joint_values[idx])


def _sample_reachable_palm_positions(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    num_samples: int,
    rng: np.random.Generator,
    base_joint_pos: dict[str, float],
) -> np.ndarray:
    """Monte Carlo palm positions from uniform random arm joint angles."""
    if num_samples <= 0:
        return np.empty((0, 3), dtype=np.float64)

    lo, hi = _joint_limits(model, ARM_JOINT_NAMES)
    if base_joint_pos:
        _set_joint_pos(model, data, base_joint_pos)
    else:
        data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)

    palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, PALM_SITE_NAME)
    if palm_id < 0:
        raise ValueError(f"Site not found in model: {PALM_SITE_NAME}")

    samples = np.empty((num_samples, 3), dtype=np.float64)
    for i in range(num_samples):
        _set_arm_joint_vector(model, data, lo + (hi - lo) * rng.uniform(size=lo.shape))
        mujoco.mj_forward(model, data)
        samples[i] = data.site_xpos[palm_id]
    return samples


def _subsample_points(points: np.ndarray, max_points: int, rng: np.random.Generator) -> np.ndarray:
    if points.shape[0] <= max_points:
        return points
    idx = rng.choice(points.shape[0], size=max_points, replace=False)
    return points[idx]


def _add_visual_box(
    worldbody,
    *,
    name: str,
    center: np.ndarray,
    half_size: np.ndarray,
    rgba: tuple[float, float, float, float],
) -> None:
    body = worldbody.add_body(name=name, pos=center.tolist())
    geom = body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=half_size.tolist(),
        rgba=list(rgba),
    )
    geom.contype = 0
    geom.conaffinity = 0
    geom.group = 1


def _add_visual_sphere(
    worldbody,
    *,
    name: str,
    center: np.ndarray,
    radius: float,
    rgba: tuple[float, float, float, float],
) -> None:
    body = worldbody.add_body(name=name, pos=center.tolist())
    geom = body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[radius, 0.0, 0.0],
        rgba=list(rgba),
    )
    geom.contype = 0
    geom.conaffinity = 0
    geom.group = 1


def _build_model_with_overlays(
    scene_path: Path,
    *,
    target_volume_lo: np.ndarray,
    target_volume_hi: np.ndarray,
    object_reset_lo: np.ndarray,
    object_reset_hi: np.ndarray,
    object_spawn: np.ndarray,
    goal_samples: np.ndarray,
    reach_samples: np.ndarray,
) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(str(scene_path))
    world = spec.worldbody

    tv_center = 0.5 * (target_volume_lo + target_volume_hi)
    tv_half = 0.5 * (target_volume_hi - target_volume_lo)
    _add_visual_box(
        world,
        name="target_volume",
        center=tv_center,
        half_size=tv_half,
        rgba=(0.0, 0.85, 0.85, 0.18),
    )

    or_center = 0.5 * (object_reset_lo + object_reset_hi)
    or_half = 0.5 * (object_reset_hi - object_reset_lo)
    _add_visual_box(
        world,
        name="object_reset_volume",
        center=or_center,
        half_size=or_half,
        rgba=(1.0, 0.55, 0.0, 0.20),
    )

    _add_visual_sphere(
        world,
        name="object_spawn_marker",
        center=object_spawn,
        radius=0.015,
        rgba=(1.0, 0.15, 0.15, 1.0),
    )

    for idx, goal in enumerate(goal_samples):
        _add_visual_sphere(
            world,
            name=f"goal_sample_{idx:03d}",
            center=goal,
            radius=0.012,
            rgba=(0.20, 0.72, 0.31, 1.0),
        )

    for idx, palm in enumerate(reach_samples):
        _add_visual_sphere(
            world,
            name=f"reach_sample_{idx:04d}",
            center=palm,
            radius=0.008,
            rgba=(0.85, 0.25, 0.85, 0.55),
        )

    return spec.compile()


def _print_report(
    *,
    scene_path: Path,
    target_volume_lo: np.ndarray,
    target_volume_hi: np.ndarray,
    object_reset_lo: np.ndarray,
    object_reset_hi: np.ndarray,
    object_spawn: np.ndarray,
    table_z: float,
    palm_pos: np.ndarray,
    goal_samples: np.ndarray,
    reach_samples: np.ndarray,
    reach_display_count: int,
    notes: list[str],
) -> None:
    print("\n=== MuJoCo workspace report ===")
    print(f"scene: {scene_path}")
    print(
        "target_volume (goal sampling, NOT object reset): "
        f"x=[{target_volume_lo[0]:.3f}, {target_volume_hi[0]:.3f}] "
        f"y=[{target_volume_lo[1]:.3f}, {target_volume_hi[1]:.3f}] "
        f"z=[{target_volume_lo[2]:.3f}, {target_volume_hi[2]:.3f}]"
    )
    print(
        "object_reset_volume: "
        f"x=[{object_reset_lo[0]:.3f}, {object_reset_hi[0]:.3f}] "
        f"y=[{object_reset_lo[1]:.3f}, {object_reset_hi[1]:.3f}] "
        f"z=[{object_reset_lo[2]:.3f}, {object_reset_hi[2]:.3f}]"
    )
    print(f"table_z (config): {table_z:.3f}  (surface ~ {table_z + TABLE_HALF_Z_M:.3f})")
    print(f"object spawn center: {object_spawn.round(4).tolist()}")
    print(f"palm site ({PALM_SITE_NAME}) @ default pose: {palm_pos.round(4).tolist()}")
    print(
        f"palm -> object distance: {float(np.linalg.norm(palm_pos - object_spawn)):.3f} m"
    )
    if goal_samples.size:
        inside = np.all(
            (goal_samples >= target_volume_lo) & (goal_samples <= target_volume_hi),
            axis=1,
        )
        print(
            f"goal samples: {goal_samples.shape[0]}  "
            f"inside target_volume: {float(np.mean(inside)):.0%}"
        )
    if reach_samples.size:
        reach_lo = reach_samples.min(axis=0)
        reach_hi = reach_samples.max(axis=0)
        goals_inside_reach = np.all(
            (goal_samples >= reach_lo) & (goal_samples <= reach_hi), axis=1
        )
        print(
            "arm reach (uniform random arm joints, fixed hand pose): "
            f"x=[{reach_lo[0]:.3f}, {reach_hi[0]:.3f}] "
            f"y=[{reach_lo[1]:.3f}, {reach_hi[1]:.3f}] "
            f"z=[{reach_lo[2]:.3f}, {reach_hi[2]:.3f}] "
            f"(from {reach_samples.shape[0]} samples, showing {reach_display_count})"
        )
        if goal_samples.size:
            print(
                f"goal samples inside reach AABB: {float(np.mean(goals_inside_reach)):.0%}"
            )
    for note in notes:
        print(f"NOTE: {note}")
    print(
        "Legend: cyan=target_volume (goal), orange=object reset, "
        "red=object center, green=goal samples, magenta=arm reach, yellow=palm default"
    )
    print("===============================\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize SimToolReal workspace in MuJoCo.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent(
            """\
            Presets:
              marvin_isaac  - bounds from isaacsimenvs_marvin_sharpa SimToolReal.yaml
              mujoco_scene  - same bounds; object spawn uses table body z from scene.xml
            """
        ),
    )
    parser.add_argument(
        "--scene",
        type=Path,
        default=DEFAULT_SCENE,
        help="Path to scene.xml (default: alongside this script).",
    )
    parser.add_argument(
        "--preset",
        choices=("marvin_isaac", "mujoco_scene"),
        default="marvin_isaac",
        help="Default workspace/table interpretation.",
    )
    parser.add_argument(
        "--target-volume-min",
        "--workspace-min",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help="Goal sampling bounds (target_volume), not object reset.",
    )
    parser.add_argument(
        "--target-volume-max",
        "--workspace-max",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help="Goal sampling bounds (target_volume), not object reset.",
    )
    parser.add_argument("--target-volume-scale", "--workspace-scale", type=float, default=1.0)
    parser.add_argument("--table-z", type=float, default=None)
    parser.add_argument("--object-z-offset", type=float, default=None)
    parser.add_argument("--object-reset-noise-xy", type=float, default=None)
    parser.add_argument("--object-reset-noise-z", type=float, default=None)
    parser.add_argument("--object-xy", type=float, nargs=2, default=(0.0, 0.0), metavar=("X", "Y"))
    parser.add_argument("--num-goals", type=int, default=25, help="Random goal markers to draw.")
    parser.add_argument(
        "--num-reach-samples",
        type=int,
        default=3000,
        help="Monte Carlo arm joint samples for reachable palm cloud.",
    )
    parser.add_argument(
        "--reach-display-max",
        type=int,
        default=2500,
        help="Max magenta reach markers drawn in the viewer.",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--zero-pose",
        action="store_true",
        help="Use all-zero joint angles instead of ARM_DEFAULT_JOINT_POS.",
    )
    args = parser.parse_args()

    scene_path = args.scene.resolve()
    if not scene_path.is_file():
        raise FileNotFoundError(f"scene.xml not found: {scene_path}")

    tv_min = (
        _parse_vec3(args.target_volume_min, "target-volume-min")
        if args.target_volume_min is not None
        else np.array(MARVIN_ISAAC_TARGET_VOLUME_MIN, dtype=np.float64)
    )
    tv_max = (
        _parse_vec3(args.target_volume_max, "target-volume-max")
        if args.target_volume_max is not None
        else np.array(MARVIN_ISAAC_TARGET_VOLUME_MAX, dtype=np.float64)
    )
    tv_lo, tv_hi = _scaled_bounds(tv_min, tv_max, args.target_volume_scale)

    table_z = (
        float(args.table_z)
        if args.table_z is not None
        else MARVIN_ISAAC_TABLE_Z
    )
    object_z_offset = (
        float(args.object_z_offset)
        if args.object_z_offset is not None
        else MARVIN_ISAAC_OBJECT_Z_OFFSET
    )
    object_noise_xy = (
        float(args.object_reset_noise_xy)
        if args.object_reset_noise_xy is not None
        else MARVIN_ISAAC_OBJECT_RESET_NOISE_XY
    )
    object_noise_z = (
        float(args.object_reset_noise_z)
        if args.object_reset_noise_z is not None
        else MARVIN_ISAAC_OBJECT_RESET_NOISE_Z
    )

    if args.preset == "mujoco_scene":
        base_model = mujoco.MjModel.from_xml_path(str(scene_path))
        table_body_id = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_BODY, "table")
        if table_body_id >= 0:
            table_z = float(base_model.body_pos[table_body_id, 2])
        if args.object_z_offset is None:
            object_z_offset = MARVIN_ISAAC_OBJECT_Z_OFFSET

    object_spawn = np.array(
        [args.object_xy[0], args.object_xy[1], table_z + object_z_offset],
        dtype=np.float64,
    )
    object_reset_lo = object_spawn - np.array(
        [object_noise_xy, object_noise_xy, object_noise_z], dtype=np.float64
    )
    object_reset_hi = object_spawn + np.array(
        [object_noise_xy, object_noise_xy, object_noise_z], dtype=np.float64
    )

    rng = np.random.default_rng(args.seed)
    goal_samples = (
        tv_lo + (tv_hi - tv_lo) * rng.uniform(size=(max(args.num_goals, 0), 3))
        if args.num_goals > 0
        else np.empty((0, 3))
    )

    base_model = mujoco.MjModel.from_xml_path(str(scene_path))
    reach_data = mujoco.MjData(base_model)
    hand_pose = {} if args.zero_pose else ARM_DEFAULT_JOINT_POS
    reach_samples = _sample_reachable_palm_positions(
        base_model,
        reach_data,
        num_samples=max(args.num_reach_samples, 0),
        rng=rng,
        base_joint_pos=hand_pose,
    )
    reach_display = _subsample_points(
        reach_samples,
        max(args.reach_display_max, 0),
        rng,
    )

    model = _build_model_with_overlays(
        scene_path,
        target_volume_lo=tv_lo,
        target_volume_hi=tv_hi,
        object_reset_lo=object_reset_lo,
        object_reset_hi=object_reset_hi,
        object_spawn=object_spawn,
        goal_samples=goal_samples,
        reach_samples=reach_display,
    )
    data = mujoco.MjData(model)

    if args.zero_pose:
        data.qpos[:] = 0.0
    else:
        _set_joint_pos(model, data, ARM_DEFAULT_JOINT_POS)
    mujoco.mj_forward(model, data)

    palm_pos = _site_pos(model, data, PALM_SITE_NAME)
    notes: list[str] = []
    if palm_pos[2] < object_spawn[2]:
        notes.append("palm site is below the nominal object spawn height.")
    if float(np.linalg.norm(palm_pos - object_spawn)) > 0.35:
        notes.append("palm is >35 cm from object spawn; check base/table/workspace alignment.")
    if args.preset == "marvin_isaac":
        notes.append(
            "markers use Isaac Sim env-local coordinates; "
            "MuJoCo scene.xml may use a different base/table layout — compare visually."
        )
    notes.append(
        "magenta reach cloud samples arm joints uniformly; it ignores collisions "
        "and does not vary finger joints."
    )

    _print_report(
        scene_path=scene_path,
        target_volume_lo=tv_lo,
        target_volume_hi=tv_hi,
        object_reset_lo=object_reset_lo,
        object_reset_hi=object_reset_hi,
        object_spawn=object_spawn,
        table_z=table_z,
        palm_pos=palm_pos,
        goal_samples=goal_samples,
        reach_samples=reach_samples,
        reach_display_count=reach_display.shape[0],
        notes=notes,
    )

    print("Launching MuJoCo viewer (Ctrl+C to quit)...")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_forward(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
