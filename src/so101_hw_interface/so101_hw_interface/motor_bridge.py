#!/usr/bin/env python3
"""ROS 2 node that bridges Feetech servos <-> topic_based_ros2_control.

It:
* subscribes to `/so101_follower/joint_commands` (sensor_msgs/JointState)
  and writes Goal_Position to the servos.
* publishes `/so101_follower/joint_states` every 20 ms using Present_Position.

Make sure the USB-to-UART adapter of the servo bus is accessible
(e.g. `/dev/ttyUSB0`) and you have permission to open it.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import math
import yaml, pathlib
from ament_index_python.packages import get_package_share_directory

# -----------------------------------------------------------------------------
# Provide lightweight stubs for optional third-party dependencies that the
# Feetech SDK imports but are not strictly required to operate the motors.
# -----------------------------------------------------------------------------
try:
    import deepdiff  # type: ignore
except ImportError:  # create stub
    import types
    deepdiff_stub = types.ModuleType("deepdiff")
    class _DD(dict):
        pass
    deepdiff_stub.DeepDiff = _DD  # type: ignore
    sys.modules["deepdiff"] = deepdiff_stub

try:
    import tqdm  # type: ignore
except ImportError:  # create stub
    import types
    tqdm_stub = types.ModuleType("tqdm")
    def _tqdm(iterable=None, **kwargs):
        return iterable if iterable is not None else []
    tqdm_stub.tqdm = _tqdm  # type: ignore
    sys.modules["tqdm"] = tqdm_stub

# -----------------------------------------------------------------------------

from so101_hw_interface.motors.feetech.feetech import FeetechMotorsBus
from so101_hw_interface.motors import Motor, MotorNormMode

PORT_DEFAULT = "/dev/ttyACM0"

JOINTS = {
    "1": {"id": 1, "model": "sts3215"},
    "2": {"id": 2, "model": "sts3215"},
    "3": {"id": 3, "model": "sts3215"},
    "4": {"id": 4, "model": "sts3215"},
    "5": {"id": 5, "model": "sts3215"},
    "6": {"id": 6, "model": "sts3215"},
}

INITIAL_RAW_POSITIONS = {
     "1": 2146,
     "2": 851,
     "3": 2974,
     "4": 2619,
     "5": 1919,
     "6": 2042,
}

CALIB_FILE = (
    pathlib.Path(get_package_share_directory("so101_hw_interface"))
    / "config/so101_calibration.yaml"
)

class MotorBridge(Node):
    def __init__(self):
        super().__init__("so101_motor_bridge")
        # Declare parameters so they can be overridden from launch/CLI
        self.declare_parameter("port", PORT_DEFAULT)
        self.declare_parameter("calib_file", str(CALIB_FILE))

        port = self.get_parameter("port").get_parameter_value().string_value
        if not port:
            port = PORT_DEFAULT

        # Build motor objects
        motors = {
            name: Motor(cfg["id"], cfg["model"], MotorNormMode.DEGREES)
            for name, cfg in JOINTS.items()
        }
        self.bus = FeetechMotorsBus(port, motors)

        self.get_logger().info(f"Connecting to Feetech bus on {port} …")
        try:
            self.bus.connect()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Could not open motor bus: {exc}")
            raise
        self.bus.configure_motors()
        self.bus.enable_torque()
        self.get_logger().info("Motor bus connected and configured.")

        # Publishers / Subscribers
        self.joint_states_pub = self.create_publisher(JointState, "so101_follower/joint_states", 10)
        self.joint_commands_sub = self.create_subscription(
            JointState,
            "so101_follower/joint_commands",
            self._command_cb,
            10,
        )

        # State variables
        self.current_commands: dict[str, float] = {}
        self._home_offsets: dict[str, int] | None = None
        self._limits: dict[str, tuple[int, int]] | None = None
        self._initial_move_done = False
        self._is_read_turn = True # Flag to alternate between read and write

        self._steps_per_rad = 4096.0 / (2 * math.pi)

        # Load calibration
        calib_path = pathlib.Path(self.get_parameter("calib_file").get_parameter_value().string_value).expanduser()
        if calib_path.is_file():
            with open(calib_path, "r", encoding="utf-8") as fp:
                calib = yaml.safe_load(fp)
            self._limits = {j: (calib[j]["range_min"], calib[j]["range_max"]) for j in JOINTS if j in calib}
            self.get_logger().info(f"Loaded joint limits from {calib_path}")
        else:
            self.get_logger().warn(f"Calibration file {calib_path} not found.")

        # Timer for periodic read/write (50 Hz)
        self.timer = self.create_timer(0.02, self._timer_cb)
        self.get_logger().info("Motor bridge node started with alternating read/write timer.")

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------
    def _command_cb(self, msg: JointState):
        """Store desired joint positions from topic (rad)."""
        for name, pos in zip(msg.name, msg.position):
            if name in JOINTS:
                self.current_commands[name] = pos

    def _timer_cb(self):
        # On each timer tick, we either do a read or a write, but never both.
        if self._is_read_turn:
            self._do_read()
        else:
            self._do_write()

        # Flip the flag for the next turn
        self._is_read_turn = not self._is_read_turn

    def _do_read(self):
        try:
            raw_positions = self.bus.sync_read("Present_Position", normalize=False)
        except Exception as exc:
            self.get_logger().warn(f"sync_read failed: {exc}")
            return

        if self._home_offsets is None:
            self._home_offsets = raw_positions
            self.get_logger().info(f"Captured home offsets on first read: {self._home_offsets}")
            # Initialize current_commands to the starting pose in radians
            self.current_commands = {
                n: (raw - self._home_offsets.get(n, 0)) * (2 * math.pi) / 4096.0
                for n, raw in raw_positions.items()
            }

        # Publish current joint states
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINTS.keys())
        js.position = [
            (raw - self._home_offsets.get(n, 0)) * (2 * math.pi) / 4096.0
            for n, raw in raw_positions.items()
        ]
        self.joint_states_pub.publish(js)

    def _do_write(self):
        if self._home_offsets is None:
            self.get_logger().warn("Skipping write: home offsets not yet captured.")
            return

        # On the first write cycle, command the motors to their predefined initial pose.
        if not self._initial_move_done:
            self.get_logger().info(f"Commanding initial pose: {INITIAL_RAW_POSITIONS}")
            try:
                self.bus.sync_write("Goal_Position", INITIAL_RAW_POSITIONS, normalize=False)
                self._initial_move_done = True
            except Exception as exc:
                self.get_logger().error(f"Failed to write initial pose: {exc}")
            return # Skip regular command on this turn

        # On subsequent write cycles, send the latest commands from the topic.
        if not self.current_commands:
            return # Nothing to write

        try:
            raw_goals = {}
            for n, rad in self.current_commands.items():
                home = self._home_offsets.get(n, 0)
                raw = int(home + rad * self._steps_per_rad)
                if self._limits and n in self._limits:
                    low, high = self._limits[n]
                    raw = max(low, min(high, raw))
                raw_goals[n] = raw
            self.bus.sync_write("Goal_Position", raw_goals, normalize=False)
        except Exception as exc:
            self.get_logger().warn(f"sync_write failed: {exc}")


# -------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------

def main():  # noqa: D401
    """Entry-point."""
    rclpy.init()
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Disconnecting from motor bus...")
        node.bus.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()