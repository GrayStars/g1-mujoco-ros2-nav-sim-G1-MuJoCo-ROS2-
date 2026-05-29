"""Compare a PyTorch checkpoint policy with its exported ONNX policy."""

import os
from dataclasses import asdict, dataclass
from pathlib import Path

COMPARE_CACHE_ROOT = Path("/tmp/mjlab_compare_policy_cache")
COMPARE_CACHE_ROOT.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("WARP_CACHE_PATH", str(COMPARE_CACHE_ROOT / "warp"))
os.environ.setdefault("MPLCONFIGDIR", str(COMPARE_CACHE_ROOT / "matplotlib"))

import numpy as np
import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
from mjlab.tasks.tracking.mdp import MotionCommandCfg
from mjlab.utils.torch import configure_torch_backends


PROJECT_ROOT = Path(__file__).resolve().parents[1]

# Edit these values before running this script.
TASK_ID = "Unitree-G1-Flat"
CHECKPOINT_FILE = "logs/rsl_rl/g1_velocity/2026-05-26_19-30-57/model_7900.pt"

# Set to None to use policy.onnx in the checkpoint folder.
ONNX_FILE: str | None = None

# Required only for tracking tasks.
MOTION_FILE: str | None = None

DEVICE: str | None = None
NUM_ENVS = 1
NUM_STEPS = 200
PRINT_EVERY = 10
PASS_MAX_ABS_DIFF = 1e-4

# Which policy should drive the environment while both are compared.
CONTROL_SOURCE = "pt"  # "pt" or "onnx"


def resolve_path(path: str) -> Path:
  resolved_path = Path(path).expanduser()
  if resolved_path.is_absolute():
    return resolved_path
  return PROJECT_ROOT / resolved_path


@dataclass(frozen=True)
class CompareConfig:
  checkpoint_file: str
  onnx_file: str | None = None
  motion_file: str | None = None
  device: str | None = None
  num_envs: int = 1
  num_steps: int = 200
  print_every: int = 10
  pass_max_abs_diff: float = 1e-4
  control_source: str = "pt"


def _load_onnxruntime():
  try:
    import onnxruntime as ort
  except ImportError as exc:
    raise ImportError(
      "Python package 'onnxruntime' is required for compare.py. "
      "Install it in the mjlab environment, for example: "
      "pip install onnxruntime"
    ) from exc
  return ort


def _prepare_paths(cfg: CompareConfig) -> tuple[Path, Path]:
  checkpoint_path = resolve_path(cfg.checkpoint_file)
  if not checkpoint_path.exists():
    raise FileNotFoundError(f"Checkpoint file not found: {checkpoint_path}")

  onnx_path = (
    resolve_path(cfg.onnx_file)
    if cfg.onnx_file is not None
    else checkpoint_path.parent / "policy.onnx"
  )
  if not onnx_path.exists():
    raise FileNotFoundError(f"ONNX file not found: {onnx_path}")

  return checkpoint_path, onnx_path


def _make_env(task_id: str, cfg: CompareConfig, device: str) -> RslRlVecEnvWrapper:
  env_cfg = load_env_cfg(task_id, play=True)
  agent_cfg = load_rl_cfg(task_id)
  env_cfg.scene.num_envs = cfg.num_envs

  if "motion" in env_cfg.commands and isinstance(
    env_cfg.commands["motion"], MotionCommandCfg
  ):
    if cfg.motion_file is None:
      raise ValueError("Tracking tasks require MOTION_FILE for policy comparison.")
    motion_path = resolve_path(cfg.motion_file)
    if not motion_path.exists():
      raise FileNotFoundError(f"Motion file not found: {motion_path}")
    motion_cmd = env_cfg.commands["motion"]
    assert isinstance(motion_cmd, MotionCommandCfg)
    motion_cmd.motion_file = str(motion_path)

  env = ManagerBasedRlEnv(cfg=env_cfg, device=device, render_mode=None)
  return RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)


def _load_pt_policy(
  task_id: str,
  env: RslRlVecEnvWrapper,
  checkpoint_path: Path,
  device: str,
):
  agent_cfg = load_rl_cfg(task_id)
  runner_cls = load_runner_cls(task_id) or MjlabOnPolicyRunner
  runner = runner_cls(env, asdict(agent_cfg), device=device)
  runner.load(
    str(checkpoint_path),
    load_cfg={"actor": True},
    strict=True,
    map_location=device,
  )
  return runner.get_inference_policy(device=device)


def _flatten_actor_obs(obs, pt_policy) -> torch.Tensor:
  return torch.cat([obs[group] for group in pt_policy.obs_groups], dim=-1)


def run_compare(task_id: str, cfg: CompareConfig) -> None:
  if cfg.control_source not in {"pt", "onnx"}:
    raise ValueError("CONTROL_SOURCE must be 'pt' or 'onnx'.")

  ort = _load_onnxruntime()
  configure_torch_backends()

  checkpoint_path, onnx_path = _prepare_paths(cfg)
  device = cfg.device or ("cuda:0" if torch.cuda.is_available() else "cpu")

  env = _make_env(task_id, cfg, device)
  try:
    pt_policy = _load_pt_policy(task_id, env, checkpoint_path, device)
    onnx_session = ort.InferenceSession(
      str(onnx_path), providers=["CPUExecutionProvider"]
    )
    onnx_input_name = onnx_session.get_inputs()[0].name
    onnx_output_name = onnx_session.get_outputs()[0].name

    obs, _ = env.reset()
    max_diffs: list[float] = []
    mean_diffs: list[float] = []

    print(f"[INFO]: Comparing checkpoint: {checkpoint_path}")
    print(f"[INFO]: Against ONNX:       {onnx_path}")
    print(
      f"[INFO]: ONNX input='{onnx_input_name}', output='{onnx_output_name}', "
      f"steps={cfg.num_steps}"
    )

    for step in range(cfg.num_steps):
      with torch.no_grad():
        action_pt = pt_policy(obs)
        flat_obs = _flatten_actor_obs(obs, pt_policy).detach().cpu().numpy().astype(np.float32)
        action_onnx_np = onnx_session.run(
          [onnx_output_name], {onnx_input_name: flat_obs}
        )[0]
        action_onnx = torch.from_numpy(action_onnx_np).to(
          device=action_pt.device, dtype=action_pt.dtype
        )

      diff = torch.abs(action_pt - action_onnx)
      max_diff = float(diff.max().item())
      mean_diff = float(diff.mean().item())
      max_diffs.append(max_diff)
      mean_diffs.append(mean_diff)

      if step % cfg.print_every == 0 or step == cfg.num_steps - 1:
        print(
          f"[step {step:04d}] max_abs_diff={max_diff:.8e}, "
          f"mean_abs_diff={mean_diff:.8e}"
        )

      action = action_pt if cfg.control_source == "pt" else action_onnx
      obs, _, dones, _ = env.step(action)
      if hasattr(pt_policy, "reset"):
        pt_policy.reset(dones)

    overall_max = max(max_diffs)
    overall_mean = sum(mean_diffs) / len(mean_diffs)
    status = "PASS" if overall_max <= cfg.pass_max_abs_diff else "FAIL"
    print(
      f"[RESULT]: {status} overall_max_abs_diff={overall_max:.8e}, "
      f"overall_mean_abs_diff={overall_mean:.8e}, "
      f"threshold={cfg.pass_max_abs_diff:.8e}"
    )
  finally:
    env.close()


def main():
  import mjlab.tasks  # noqa: F401
  import src.tasks  # noqa: F401

  cfg = CompareConfig(
    checkpoint_file=CHECKPOINT_FILE,
    onnx_file=ONNX_FILE,
    motion_file=MOTION_FILE,
    device=DEVICE,
    num_envs=NUM_ENVS,
    num_steps=NUM_STEPS,
    print_every=PRINT_EVERY,
    pass_max_abs_diff=PASS_MAX_ABS_DIFF,
    control_source=CONTROL_SOURCE,
  )
  run_compare(TASK_ID, cfg)


if __name__ == "__main__":
  main()
