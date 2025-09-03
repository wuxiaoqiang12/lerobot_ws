#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Joy, JointState
from control_msgs.msg import JointJog
from std_msgs.msg import Float64MultiArray

from pymoveit2.moveit2_servo import MoveIt2Servo

class JoyToServo(Node):
    def __init__(self):
        super().__init__("joy_to_servo_node")

        # Button mapping
        self.button_map = {
            "enable": 0,      # A button
            "disable": 1,     # B button
            "joint_mode": 4,  # LB button
        }
        # Joystick and trigger (Axes) mapping
        self.axis_map = {
            # Cartesian space control
            "linear_x": 0,          # Left stick L/R -> X
            "linear_y": 1,          # Left stick U/D -> Y
            "linear_z": 4,          # Right stick U/D -> Z
            "angular_y": 6,         # D-Pad L/R -> Pitch
            "angular_z": 3,         # Right stick L/R -> Yaw
            # Gripper control
            "gripper_close": 2,     # LT (Left Trigger)
            "gripper_open": 5,      # RT (Right Trigger)
        }
        # Joystick mapping for joint space control
        self.joint_axis_map = [0, 1, 3, 4, 6] # Corresponds to 5 joints
        # Arm joint names
        self.arm_joint_names = ["1", "2", "3", "4", "5"]
        # Gripper joint information
        self.gripper_joint_name = "6"
        self.gripper_open_pos = 1.745     # Gripper fully open position (upper limit)
        self.gripper_closed_pos = -0.1745 # Gripper fully closed position (lower limit)
        self.gripper_jog_speed = 4.0      # Gripper movement speed (rad/s)

        # -- Initialize Servo --
        self.servo = MoveIt2Servo(
            self, frame_id="gripper", linear_speed=1.0, angular_speed=1.0, enable_at_init=False
        )
        self.get_logger().info("MoveIt2Servo Initialized.")
        self.get_logger().info("Press 'A' to enable, 'B' to disable.")
        self.get_logger().info("Long press 'LB' ( > 0.5s ) to toggle control mode.")

        # -- Create subscribers and publishers --
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.joint_jog_pub = self.create_publisher(JointJog, "/servo_node/joint_jog", 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        # -- Logical state variables --
        self.prev_buttons = None
        self.is_joint_mode = False
        self.long_press_duration = Duration(seconds=0.5)
        self.lb_press_time = None
        self.axis_names = {
            0: "Left Stick L/R", 1: "Left Stick U/D", 2: "Left Trigger (LT)",
            3: "Right Stick L/R", 4: "Right Stick U/D", 5: "Right Trigger (RT)",
            6: "D-Pad L/R", 7: "D-Pad U/D"
        }
        # Store the current gripper position
        self.current_gripper_pos = self.gripper_open_pos
        self.gripper_joint_idx = -1

    def joint_state_callback(self, msg: JointState):
        """Callback function to update the gripper's current position from the /joint_states topic."""
        if self.gripper_joint_idx == -1:
            try:
                self.gripper_joint_idx = msg.name.index(self.gripper_joint_name)
            except ValueError:
                # If the gripper joint name is not found in joint_states, print an error only once
                self.get_logger().error(f"Gripper joint '{self.gripper_joint_name}' not found in /joint_states. Gripper control will be disabled.")
                self.gripper_joint_idx = -2 # Mark as checked and not found
                return
        
        if self.gripper_joint_idx >= 0:
            self.current_gripper_pos = msg.position[self.gripper_joint_idx]

    def joy_callback(self, msg: Joy):
        """Main callback function to handle all joystick inputs."""
        if self.prev_buttons is None:
            self.prev_buttons = list(msg.buttons)

        # Enable/Disable Servo
        if msg.buttons[self.button_map["enable"]] == 1 and self.prev_buttons[self.button_map["enable"]] == 0:
            self.get_logger().info("'A' button pressed, enabling Servo...")
            self.servo.enable()
        if msg.buttons[self.button_map["disable"]] == 1 and self.prev_buttons[self.button_map["disable"]] == 0:
            self.get_logger().info("'B' button pressed, disabling Servo...")
            self.servo.disable()

        # Long press logic for switching modes
        lb_state = msg.buttons[self.button_map["joint_mode"]]
        prev_lb_state = self.prev_buttons[self.button_map["joint_mode"]]

        if lb_state == 1 and prev_lb_state == 0:
            self.lb_press_time = self.get_clock().now()
        if lb_state == 0 and prev_lb_state == 1:
            if self.lb_press_time and (self.get_clock().now() - self.lb_press_time) >= self.long_press_duration:
                self.is_joint_mode = not self.is_joint_mode
                self.print_mode_status()
            self.lb_press_time = None
        
        self.prev_buttons = list(msg.buttons)

        # Send commands based on the current mode
        if self.servo.is_enabled:
            # Arm control
            if self.is_joint_mode:
                self.send_joint_command(msg)
            else:
                self.send_cartesian_command(msg)
            # Gripper control
            self.handle_gripper_command(msg)

    def print_mode_status(self):
        """Print detailed information about the current control mode."""
        gripper_mapping_info = f"      - Gripper Open/Close: {self.axis_names.get(self.axis_map['gripper_open'])} / {self.axis_names.get(self.axis_map['gripper_close'])}\n"
        if self.is_joint_mode:
            log_message = "--- Switched to JOINT control mode ---\n"
            log_message += "    Operation: Joysticks now control individual joint velocities.\n"
            log_message += "    Mapping:\n"
            log_message += gripper_mapping_info
            for i, joint_name in enumerate(self.arm_joint_names):
                axis_index = self.joint_axis_map[i]
                axis_desc = self.axis_names.get(axis_index, f"Axis {axis_index}")
                log_message += f"      - {joint_name}: {axis_desc}\n"
            self.get_logger().info(log_message)
        else:
            log_message = "--- Switched to CARTESIAN control mode ---\n"
            log_message += "    Operation: Joysticks now control the end-effector's velocity.\n"
            log_message += "    Mapping:\n"
            log_message += gripper_mapping_info
            log_message += f"      - X-axis movement:    {self.axis_names.get(self.axis_map['linear_x'])}\n"
            log_message += f"      - Y-axis movement:    {self.axis_names.get(self.axis_map['linear_y'])}\n"
            log_message += f"      - Z-axis movement:    {self.axis_names.get(self.axis_map['linear_z'])}\n"
            log_message += f"      - Pitch (rotation-y): {self.axis_names.get(self.axis_map['angular_y'])}\n"
            log_message += f"      - Yaw (rotation-z):   {self.axis_names.get(self.axis_map['angular_z'])}"
            self.get_logger().info(log_message)
    
    def handle_gripper_command(self, msg: Joy):
        """Handle trigger signals and send incremental gripper commands."""
        if self.gripper_joint_idx < 0: return # If the gripper joint is not found, do nothing

        rt_val = msg.axes[self.axis_map['gripper_open']]
        lt_val = msg.axes[self.axis_map['gripper_close']]
        
        # Assume joy_callback frequency is approx. 50Hz, so dt ≈ 0.02s
        dt = 0.02 
        target_pos = self.current_gripper_pos
        
        if lt_val < 1.0: # If the left trigger (close) is pressed
            normalized_press = (lt_val - 1.0) / -2.0  # [0, 1]
            # step = speed * pressure * time
            delta = self.gripper_jog_speed * normalized_press * dt
            # new_target = current_position - step (because it's closing)
            target_pos = self.current_gripper_pos - delta
        elif rt_val < 1.0: # If the right trigger (open) is pressed
            normalized_press = (rt_val - 1.0) / -2.0 # [0, 1]
            delta = self.gripper_jog_speed * normalized_press * dt
            # new_target = current_position + step (because it's opening)
            target_pos = self.current_gripper_pos + delta
        
        # Only send a command when a trigger is pressed
        if lt_val < 1.0 or rt_val < 1.0:
            # Clamp the target position within the safe range
            clamped_pos = max(self.gripper_closed_pos, min(self.gripper_open_pos, target_pos))
            self.send_gripper_command(clamped_pos)

    def send_gripper_command(self, position):
        """Build and publish the Float64MultiArray message for the gripper."""
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [position]
        self.gripper_pub.publish(cmd_msg)

    def send_cartesian_command(self, msg: Joy):
        """Send Cartesian velocity commands via MoveIt Servo."""
        linear = (
            msg.axes[self.axis_map["linear_y"]],
            msg.axes[self.axis_map["linear_x"]],
            -msg.axes[self.axis_map["linear_z"]]
        )
        angular = (0.0, msg.axes[self.axis_map["angular_y"]], msg.axes[self.axis_map["angular_z"]])
        self.servo(linear=linear, angular=angular)

    def send_joint_command(self, msg: Joy):
        """Send joint velocity commands (JointJog) to MoveIt Servo."""
        joint_jog_msg = JointJog()
        joint_jog_msg.header.stamp = self.get_clock().now().to_msg()
        joint_jog_msg.joint_names = self.arm_joint_names
        
        velocities = [0.0] * len(self.arm_joint_names)
        for i, axis_index in enumerate(self.joint_axis_map):
            velocities[i] = -msg.axes[axis_index]
        joint_jog_msg.velocities = velocities
        
        self.joint_jog_pub.publish(joint_jog_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToServo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()