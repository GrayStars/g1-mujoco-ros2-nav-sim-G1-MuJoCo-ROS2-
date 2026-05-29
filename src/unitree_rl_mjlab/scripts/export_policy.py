"""Export a trained RSL-RL checkpoint to ONNX."""

import os
from dataclasses import asdict, dataclass
from pathlib import Path

EXPORT_CACHE_ROOT = Path("/tmp/mjlab_export_policy_cache")
EXPORT_CACHE_ROOT.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("WARP_CACHE_PATH", str(EXPORT_CACHE_ROOT / "warp"))
os.environ.setdefault("MPLCONFIGDIR", str(EXPORT_CACHE_ROOT / "matplotlib"))

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.rl.exporter_utils import attach_metadata_to_onnx, get_base_metadata
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
from mjlab.tasks.tracking.mdp import MotionCommandCfg
from mjlab.utils.torch import configure_torch_backends


PROJECT_ROOT = Path(__file__).resolve().parents[1]

# Edit these values before running this script.
TASK_ID = "Unitree-G1-Flat"
CHECKPOINT_FILE = "logs/rsl_rl/g1_velocity/2026-05-26_19-30-57/model_7900.pt"

# Set to None to write/overwrite policy.onnx in the checkpoint folder.
OUTPUT_FILE: str | None = None

# Required only for tracking tasks.
MOTION_FILE: str | None = None

DEVICE: str | None = None
NUM_ENVS = 1
ATTACH_METADATA = True


def resolve_path(path: str) -> Path:
  resolved_path = Path(path).expanduser()
  if resolved_path.is_absolute():
    return resolved_path
  return PROJECT_ROOT / resolved_path


@dataclass(frozen=True)
class ExportConfig:
  checkpoint_file: str
  output_file: str | None = None
  motion_file: str | None = None
  device: str | None = None
  num_envs: int = 1
  attach_metadata: bool = True


def run_export(task_id: str, cfg: ExportConfig) -> None:
  configure_torch_backends()

  checkpoint_path = resolve_path(cfg.checkpoint_file)
  if not checkpoint_path.exists():
    raise FileNotFoundError(f"Checkpoint file not found: {checkpoint_path}")

  output_path = (
    resolve_path(cfg.output_file)
    if cfg.output_file is not None
    else checkpoint_path.parent / "policy.onnx"
  )

  device = cfg.device or ("cuda:0" if torch.cuda.is_available() else "cpu")

  env_cfg = load_env_cfg(task_id, play=True)
  agent_cfg = load_rl_cfg(task_id)
  env_cfg.scene.num_envs = cfg.num_envs

  if "motion" in env_cfg.commands and isinstance(
    env_cfg.commands["motion"], MotionCommandCfg
  ):
    if cfg.motion_file is None:
      raise ValueError("Tracking tasks require --motion-file for ONNX export.")
    motion_path = resolve_path(cfg.motion_file)
    if not motion_path.exists():
      raise FileNotFoundError(f"Motion file not found: {motion_path}")
    motion_cmd = env_cfg.commands["motion"]
    assert isinstance(motion_cmd, MotionCommandCfg)
    motion_cmd.motion_file = str(motion_path)

  env = ManagerBasedRlEnv(cfg=env_cfg, device=device, render_mode=None)
  try:
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    runner_cls = load_runner_cls(task_id) or MjlabOnPolicyRunner
    runner = runner_cls(env, asdict(agent_cfg), device=device)
    runner.load(
      str(checkpoint_path),
      load_cfg={"actor": True},
      strict=True,
      map_location=device,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    runner.export_policy_to_onnx(str(output_path.parent), output_path.name)

    if cfg.attach_metadata:
      metadata = get_base_metadata(env.unwrapped, checkpoint_path.parent.name)
      command_manager = getattr(env.unwrapped, "command_manager", None)
      motion_term = None
      if command_manager is not None:
        try:
          motion_term = command_manager.get_term("motion")
        except KeyError:
          motion_term = None
      if motion_term is not None:
        metadata.update(
          {
            "anchor_body_name": getattr(motion_term.cfg, "anchor_body_name", None),
            "body_names": list(getattr(motion_term.cfg, "body_names", ())),
          }
        )
      attach_metadata_to_onnx(output_path, metadata)

    print(f"[INFO]: Exported ONNX policy to: {output_path}")
  finally:
    env.close()


def main():
  import mjlab.tasks  # noqa: F401
  import src.tasks  # noqa: F401

  cfg = ExportConfig(
    checkpoint_file=CHECKPOINT_FILE,
    output_file=OUTPUT_FILE,
    motion_file=MOTION_FILE,
    device=DEVICE,
    num_envs=NUM_ENVS,
    attach_metadata=ATTACH_METADATA,
  )
  run_export(TASK_ID, cfg)


if __name__ == "__main__":
  main()
