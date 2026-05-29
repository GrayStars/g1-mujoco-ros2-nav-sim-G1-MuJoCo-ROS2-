"""Script to play RL agent with RSL-RL."""

import os
import select
import sys
import termios
import threading
import tty
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

import torch
import tyro

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import list_tasks, load_env_cfg, load_rl_cfg, load_runner_cls
from mjlab.tasks.tracking.mdp import MotionCommandCfg
from mjlab.utils.os import get_wandb_checkpoint_path
from mjlab.utils.torch import configure_torch_backends
from mjlab.utils.wrappers import VideoRecorder
from mjlab.viewer import NativeMujocoViewer, ViserPlayViewer


class KeyboardVelocityController:
  """Terminal keyboard controller for velocity commands."""

  def __init__(
    self,
    linear_step: float = 0.1,
    yaw_step: float = 0.1,
  ):
    self.linear_step = linear_step
    self.yaw_step = yaw_step
    self.command = [0.0, 0.0, 0.0]
    self._lock = threading.Lock()
    self._stop_event = threading.Event()
    self._thread: threading.Thread | None = None
    self._old_terminal_settings = None

  def start(self) -> None:
    if not sys.stdin.isatty():
      print("[WARN]: Keyboard control requires an interactive terminal.")
      return

    self._old_terminal_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    self._thread = threading.Thread(target=self._read_keyboard, daemon=True)
    self._thread.start()
    print(
      "\n[INFO]: Keyboard control enabled\n"
      f"  w/s: increase/decrease x velocity by {self.linear_step} m/s\n"
      f"  a/d: increase/decrease y velocity by {self.linear_step} m/s\n"
      f"  q/e: increase/decrease yaw velocity by {self.yaw_step} rad/s\n"
      "  space: stop\n"
      "  Ctrl-C: quit\n"
    )

  def stop(self) -> None:
    self._stop_event.set()
    if self._thread is not None:
      self._thread.join(timeout=0.5)
    if self._old_terminal_settings is not None:
      termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_terminal_settings)

  def write_to_command_term(self, command_term) -> None:
    with self._lock:
      command = tuple(self.command)
    command_tensor = command_term.vel_command_b.new_tensor(command)
    command_term.vel_command_b[:, :] = command_tensor
    if hasattr(command_term, "is_heading_env"):
      command_term.is_heading_env[:] = False
    if hasattr(command_term, "is_standing_env"):
      command_term.is_standing_env[:] = False

  def _read_keyboard(self) -> None:
    while not self._stop_event.is_set():
      readable, _, _ = select.select([sys.stdin], [], [], 0.05)
      if not readable:
        continue
      key = sys.stdin.read(1).lower()
      with self._lock:
        if key == "w":
          self.command[0] += self.linear_step
        elif key == "s":
          self.command[0] -= self.linear_step
        elif key == "a":
          self.command[1] += self.linear_step
        elif key == "d":
          self.command[1] -= self.linear_step
        elif key == "q":
          self.command[2] += self.yaw_step
        elif key == "e":
          self.command[2] -= self.yaw_step
        elif key == " ":
          self.command = [0.0, 0.0, 0.0]
        else:
          continue
        print(
          f"\r[cmd] x={self.command[0]: .2f} m/s, "
          f"y={self.command[1]: .2f} m/s, "
          f"yaw={self.command[2]: .2f} rad/s",
          end="",
          flush=True,
        )


@dataclass(frozen=True)
class PlayConfig:
  agent: Literal["zero", "random", "trained"] = "trained"
  checkpoint_file: str | None = None
  motion_file: str | None = None
  num_envs: int | None = None
  device: str | None = None
  video: bool = False
  video_length: int = 200
  video_height: int | None = None
  video_width: int | None = None
  camera: int | str | None = None
  viewer: Literal["auto", "native", "viser"] = "auto"
  no_terminations: bool = False
  """Disable all termination conditions (useful for viewing motions with dummy agents)."""

  # Internal flag used by demo script.
  _demo_mode: tyro.conf.Suppress[bool] = False


def _prompt_play_mode() -> Literal["test", "control"]:
  if not sys.stdin.isatty():
    print("[WARN]: Non-interactive terminal detected; using test mode.")
    return "test"

  while True:
    mode = input("请选择 play 模式：[t] 测试模式 / [c] 键盘控制模式：").strip().lower()
    if mode in {"", "t", "test", "测试", "测试模式"}:
      return "test"
    if mode in {"c", "control", "控制", "控制模式"}:
      return "control"
    print("请输入 t 或 c。")


def _configure_keyboard_velocity_command(env_cfg) -> None:
  if "twist" not in env_cfg.commands:
    raise ValueError(
      "Keyboard control mode only supports velocity tasks with a 'twist' command."
    )

  twist_cmd = env_cfg.commands["twist"]
  twist_cmd.rel_standing_envs = 0.0
  if getattr(twist_cmd, "heading_command", False):
    twist_cmd.heading_command = False
    twist_cmd.ranges.heading = None


def _install_keyboard_velocity_control(env) -> KeyboardVelocityController:
  command_term = env.command_manager.get_term("twist")
  if command_term is None or not hasattr(command_term, "vel_command_b"):
    raise ValueError(
      "Keyboard control mode could not find a velocity 'twist' command term."
    )

  controller = KeyboardVelocityController()
  original_compute = command_term.compute

  def compute_with_keyboard(dt: float) -> None:
    original_compute(dt)
    controller.write_to_command_term(command_term)

  command_term.compute = compute_with_keyboard
  controller.write_to_command_term(command_term)
  controller.start()
  return controller


def run_play(task_id: str, cfg: PlayConfig):
  configure_torch_backends()

  device = cfg.device or ("cuda:0" if torch.cuda.is_available() else "cpu")

  env_cfg = load_env_cfg(task_id, play=True)
  agent_cfg = load_rl_cfg(task_id)
  play_mode = _prompt_play_mode()

  DUMMY_MODE = cfg.agent in {"zero", "random"}
  TRAINED_MODE = not DUMMY_MODE

  # Disable terminations if requested (useful for viewing motions).
  if cfg.no_terminations:
    env_cfg.terminations = {}
    print("[INFO]: Terminations disabled")

  if play_mode == "control":
    _configure_keyboard_velocity_command(env_cfg)

  # Check if this is a tracking task by checking for motion command.
  is_tracking_task = "motion" in env_cfg.commands and isinstance(
    env_cfg.commands["motion"], MotionCommandCfg
  )

  if is_tracking_task and cfg._demo_mode:
    # Demo mode: use uniform sampling to see more diversity with num_envs > 1.
    motion_cmd = env_cfg.commands["motion"]
    assert isinstance(motion_cmd, MotionCommandCfg)
    motion_cmd.sampling_mode = "uniform"

  if is_tracking_task:
    motion_cmd = env_cfg.commands["motion"]
    assert isinstance(motion_cmd, MotionCommandCfg)

    # Check for local motion file first (works for both dummy and trained modes).
    if cfg.motion_file is not None and Path(cfg.motion_file).exists():
      print(f"[INFO]: Using local motion file: {cfg.motion_file}")
      motion_cmd.motion_file = cfg.motion_file
    elif DUMMY_MODE:
      if not cfg.registry_name:
        raise ValueError(
          "Tracking tasks require either:\n"
          "  --motion-file /path/to/motion.npz (local file)\n"
          "  --registry-name your-org/motions/motion-name (download from WandB)"
        )
  log_dir: Path | None = None
  resume_path: Path | None = None
  if TRAINED_MODE:
    log_root_path = (Path("logs") / "rsl_rl" / agent_cfg.experiment_name).resolve()
    if cfg.checkpoint_file is not None:
      resume_path = Path(cfg.checkpoint_file)
      if not resume_path.exists():
        raise FileNotFoundError(f"Checkpoint file not found: {resume_path}")
      print(f"[INFO]: Loading checkpoint: {resume_path.name}")
    else:
      if cfg.wandb_run_path is None:
        raise ValueError(
          "`wandb_run_path` is required when `checkpoint_file` is not provided."
        )
      resume_path, was_cached = get_wandb_checkpoint_path(
        log_root_path, Path(cfg.wandb_run_path)
      )
      # Extract run_id and checkpoint name from path for display.
      run_id = resume_path.parent.name
      checkpoint_name = resume_path.name
      cached_str = "cached" if was_cached else "downloaded"
      print(
        f"[INFO]: Loading checkpoint: {checkpoint_name} (run: {run_id}, {cached_str})"
      )
    log_dir = resume_path.parent

  if cfg.num_envs is not None:
    env_cfg.scene.num_envs = cfg.num_envs
  if cfg.video_height is not None:
    env_cfg.viewer.height = cfg.video_height
  if cfg.video_width is not None:
    env_cfg.viewer.width = cfg.video_width

  render_mode = "rgb_array" if (TRAINED_MODE and cfg.video) else None
  if cfg.video and DUMMY_MODE:
    print(
      "[WARN] Video recording with dummy agents is disabled (no checkpoint/log_dir)."
    )
  env = ManagerBasedRlEnv(cfg=env_cfg, device=device, render_mode=render_mode)
  keyboard_controller: KeyboardVelocityController | None = None
  if play_mode == "control":
    keyboard_controller = _install_keyboard_velocity_control(env)

  if TRAINED_MODE and cfg.video:
    print("[INFO] Recording videos during play")
    assert log_dir is not None  # log_dir is set in TRAINED_MODE block
    env = VideoRecorder(
      env,
      video_folder=log_dir / "videos" / "play",
      step_trigger=lambda step: step == 0,
      video_length=cfg.video_length,
      disable_logger=True,
    )

  env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
  if DUMMY_MODE:
    action_shape: tuple[int, ...] = env.unwrapped.action_space.shape
    if cfg.agent == "zero":

      class PolicyZero:
        def __call__(self, obs) -> torch.Tensor:
          del obs
          return torch.zeros(action_shape, device=env.unwrapped.device)

      policy = PolicyZero()
    else:

      class PolicyRandom:
        def __call__(self, obs) -> torch.Tensor:
          del obs
          return 2 * torch.rand(action_shape, device=env.unwrapped.device) - 1

      policy = PolicyRandom()
  else:
    runner_cls = load_runner_cls(task_id) or MjlabOnPolicyRunner
    runner = runner_cls(env, asdict(agent_cfg), device=device)
    runner.load(
      str(resume_path), load_cfg={"actor": True}, strict=True, map_location=device
    )
    policy = runner.get_inference_policy(device=device)

  # Handle "auto" viewer selection.
  if cfg.viewer == "auto":
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    resolved_viewer = "native" if has_display else "viser"
    del has_display
  else:
    resolved_viewer = cfg.viewer

  try:
    if resolved_viewer == "native":
      NativeMujocoViewer(env, policy).run()
    elif resolved_viewer == "viser":
      ViserPlayViewer(env, policy).run()
    else:
      raise RuntimeError(f"Unsupported viewer backend: {resolved_viewer}")
  finally:
    if keyboard_controller is not None:
      keyboard_controller.stop()
    env.close()


def main():
  # Parse first argument to choose the task.
  # Import tasks to populate the registry.
  import mjlab.tasks  # noqa: F401
  import src.tasks

  all_tasks = list_tasks()
  chosen_task, remaining_args = tyro.cli(
    tyro.extras.literal_type_from_choices(all_tasks),
    add_help=False,
    return_unknown_args=True,
    config=mjlab.TYRO_FLAGS,
  )

  # Parse the rest of the arguments + allow overriding env_cfg and agent_cfg.
  agent_cfg = load_rl_cfg(chosen_task)

  args = tyro.cli(
    PlayConfig,
    args=remaining_args,
    default=PlayConfig(),
    prog=sys.argv[0] + f" {chosen_task}",
    config=mjlab.TYRO_FLAGS,
  )
  del remaining_args, agent_cfg

  run_play(chosen_task, args)


if __name__ == "__main__":
  main()
