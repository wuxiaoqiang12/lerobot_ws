#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from pymoveit2.moveit2_servo import MoveIt2Servo

class JoyToServo(Node):
    def __init__(self):
        super().__init__("joy_to_servo")

        # Initialize MoveIt2Servo, but start it in a disabled state
        self.servo = MoveIt2Servo(
            self,
            frame_id="world",
            linear_speed=10.0,
            angular_speed=10.0,
            enable_at_init=False,  # Start disabled
        )
        self.get_logger().info("MoveIt2Servo Initialized. Press 'A' to enable, 'B' to disable.")

        # Store the previous state of the buttons to detect presses
        self.prev_buttons = None

        # Standard mapping for Xbox controllers
        self.button_map = {"A": 0, "B": 1}

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # Axis mapping remains the same
        self.axis_map = {
            "linear_x": 1,   # Left stick up/down
            "linear_y": 0,   # Left stick left/right
            "linear_z": 4,   # Right stick up/down
            "angular_z": 3,  # Right stick left/right
        }

    def joy_callback(self, msg: Joy):
        """
        Detects button presses to enable/disable servo and sends commands if enabled.
        """
        # Initialize previous buttons on the first message
        if self.prev_buttons is None:
            self.prev_buttons = list(msg.buttons)

        # Check for 'A' button press (rising edge from 0 to 1)
        if msg.buttons[self.button_map["A"]] == 1 and self.prev_buttons[self.button_map["A"]] == 0:
            self.get_logger().info("'A' button pressed, enabling Servo...")
            self.servo.enable()

        # Check for 'B' button press (rising edge from 0 to 1)
        if msg.buttons[self.button_map["B"]] == 1 and self.prev_buttons[self.button_map["B"]] == 0:
            self.get_logger().info("'B' button pressed, disabling Servo...")
            self.servo.disable()

        # Store the current button states for the next callback
        self.prev_buttons = list(msg.buttons)

        # Only process axis commands if Servo is enabled
        if self.servo.is_enabled:
            # Build linear velocity tuple from joystick input
            linear = (
                msg.axes[self.axis_map["linear_y"]],
                msg.axes[self.axis_map["linear_x"]],
                msg.axes[self.axis_map["linear_z"]]
            )

            # Build angular velocity tuple from joystick input
            angular = (
                0.0,
                0.0,
                msg.axes[self.axis_map["angular_z"]]
            )

            # Call the servo object as a function to send the command
            self.servo(linear=linear, angular=angular)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToServo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()