"""
A toy trainner to test pipeline behavior without the need of AI core.
"""

import numpy as np
import time
from scipy.spatial.transform import Rotation

from rclpy.node import Node
from acrobatic_oa.environment import GzEnvironment

from alpha_ai_core.interface import (
    EpisodeConfiguration,
    SpawnCondition, Action, StepResult,
    TaskType,
    Observation, SimulationInformation
)

import time
MAX_LINEAR_SPEED = 50.0   # m/s, total speed
MAX_ANGULAR_SPEED = 6.0  # rad/s, total rate
MAX_TILT_ANGLE = 1.57    # rad, angle between body z-axis and world z-axis

def random_vector(rng: np.random.Generator, max_norm: float) -> np.ndarray:
    """Random direction, magnitude uniform in [0, max_norm]. Total norm never exceeds max_norm."""
    direction = rng.normal(size=3)
    direction /= np.linalg.norm(direction)
    return direction * rng.uniform(0.0, max_norm)


def random_spawn_attitude(rng: np.random.Generator, max_tilt: float) -> list[float]:
    """
    Returns [roll, pitch, yaw] compatible with Rotation.from_euler('xyz', ...).
    Tilt (body z vs world z) is in [0, max_tilt], tilt direction and yaw are uniform over 360 deg.
    """
    tilt = rng.uniform(0.0, max_tilt)
    tilt_azimuth = rng.uniform(0.0, 2.0 * np.pi)
    yaw = rng.uniform(0.0, 2.0 * np.pi)

    tilt_axis = np.array([np.cos(tilt_azimuth), np.sin(tilt_azimuth), 0.0])
    rotation = Rotation.from_euler('z', yaw) * Rotation.from_rotvec(tilt * tilt_axis)
    return rotation.as_euler('xyz').tolist()


def random_spawn_condition(rng: np.random.Generator, position) -> SpawnCondition:
    return SpawnCondition(
        position=list(position),
        linear_velocity=random_vector(rng, MAX_LINEAR_SPEED).tolist(),
        angular_velocity=random_vector(rng, MAX_ANGULAR_SPEED).tolist(),
        roll_pitch_yaw=random_spawn_attitude(rng, MAX_TILT_ANGLE),
    )

def format_step_result(step_result: StepResult) -> str:
    """Formats a StepResult dataclass into a perfectly aligned text table string."""
    obs: Observation = step_result.observation
    info: SimulationInformation = step_result.information

    # Check spatial boxes
    coarse_status = "Empty (All Zeros)" if np.all(obs.spatial_coarse == 0) else f"Occupied ({np.count_nonzero(obs.spatial_coarse)} active elements)"
    detail_status = "Empty (All Zeros)" if np.all(obs.spatial_detail == 0) else f"Occupied ({np.count_nonzero(obs.spatial_detail)} active elements)"

    # Format arrays compactly with fixed decimal places
    state_str = "[" + ", ".join(f"{x:6.2f}" for x in obs.state) + "]"
    intent_str = "[" + ", ".join(f"{x:5.2f}" for x in obs.intent) + "]"
    pos_str = "[" + ", ".join(f"{x:6.2f}" for x in info.position) + "]"

    # Column widths designed to fit long array strings comfortably
    col1 = 17
    col2 = 76
    total_width = col1 + col2 + 3  

    def make_row(label, val):
        return f"│ {label:<{col1}} │ {val:<{col2}} │"

    top_border = "┌" + "─" * (total_width + 2) + "┐"
    header_sep = "├" + "─" * (total_width + 2) + "┤"
    mid_border = "├" + "─" * (col1 + 2) + "┬" + "─" * (col2 + 2) + "┤"
    data_sep   = "├" + "─" * (col1 + 2) + "┼" + "─" * (col2 + 2) + "┤"
    bot_border = "└" + "─" * (total_width + 2) + "┘"
    
    title_line = f"│ {'STEP RESULT LOG':^{total_width}} │"

    return "\n".join([
        top_border,
        title_line,
        header_sep,
        make_row("Sim Time", f"{info.simulation_time_us} us"),
        make_row("Position (x,y,z)", pos_str),
        make_row("Body Contact", str(info.body_contact)),
        make_row("Propeller Contact", str(info.propeller_contact)),
        data_sep,
        make_row("State", state_str),
        make_row("Intent", intent_str),
        make_row("Spatial Coarse", coarse_status),
        make_row("Spatial Detail", detail_status),
        bot_border
    ])

class TrainerAnalyzer:
    """Helper class to collect and log performance metrics via ROS 2 logger."""
    def __init__(self, node):
        self.node = node
        self.reset_times: list[float] = []
        self.loop_times: list[float] = []
        self._reset_start = None
        self._step_start = None

    def start_reset(self):
        self._reset_start = time.perf_counter()

    def end_reset(self):
        if self._reset_start is not None:
            self.reset_times.append(time.perf_counter() - self._reset_start)
            self._reset_start = None

    def start_step(self):
        self._step_start = time.perf_counter()

    def end_step(self):
        if self._step_start is not None:
            self.loop_times.append(time.perf_counter() - self._step_start)
            self._step_start = None

    def print_summary(self):
        avg_reset = np.mean(self.reset_times) if self.reset_times else 0.0
        avg_loop = np.mean(self.loop_times) if self.loop_times else 0.0
        
        summary_msg = (
            "\n========== ToyTrainer Performance Summary ==========\n"
            f"Resets executed : {len(self.reset_times)}\n"
            f"  - Total time  : {sum(self.reset_times):.4f} s\n"
            f"  - Average time: {avg_reset:.4f} s/reset\n"
            f"Total of Steps  : {len(self.loop_times)}\n"
            f"  - Total time  : {sum(self.loop_times):.4f} s\n"
            f"  - Average time: {avg_loop:.4f} s/step\n"
            "===================================================="
        )
        try:
            self.node.get_logger().info(summary_msg)
        except Exception:
            # Fallback if the ROS node is already destroyed during shutdown
            print(summary_msg)

class ToyTrainer:
    def __init__(self, the_node: Node):
        self.the_node = the_node
        self.gz_environment = GzEnvironment(the_node)
        self.step_result: StepResult = None
        self.analyzer = TrainerAnalyzer(the_node)

    def __del__(self):
        """Called automatically when the ToyTrainer instance is destroyed."""
        self.analyzer.print_summary()


    def training_loop(self):
        """
        Repeatedly run reset, then 30 steps while collecting StepResult.
        """
        time.sleep(1)
        rng = np.random.default_rng()
        while True:
            spawn = random_spawn_condition(rng, position=[0.0, 0.0, 3.0])
            ep_conf = EpisodeConfiguration(
                task=TaskType.HOVER,
                seed=9,
                spawn=spawn,
                condition_index=0,
                repeat_index=0,
                episode_index=0
            )
            
            # 1. Reset with timing
            self.analyzer.start_reset()
            self.step_result = self.gz_environment.reset(episode_config=ep_conf)
            self.analyzer.end_reset()
            
            self.the_node.get_logger().info(format_step_result(self.step_result))

            # 2. Step loop with timing (30 steps)
            counter = 0.0
            flip = -1
            for _ in range(0, 50):
                self.analyzer.start_step()
                if flip > 0 and counter >= 1.0:
                    flip = -1
                elif flip < 0 and counter <= -1.0:
                    flip = 1

                counter += 0.1 * flip
                dummy_actions = np.array([counter, counter, counter, counter, counter, counter, -counter, -counter])

                self.step_result = self.gz_environment.step(Action(values=dummy_actions))
                self.the_node.get_logger().info(format_step_result(self.step_result))
                self.analyzer.end_step()


        


            
    