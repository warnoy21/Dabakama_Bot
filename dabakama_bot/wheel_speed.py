#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from example_interfaces.msg import Float32


class WheelSpeedValue(Node): 
    def __init__(self):
        super().__init__("wheel_speed_publisher") 
        
        self.publisher_ = self.create_publisher(Float32, 'wheel_speed_value', 10)
        self.subscription = self.create_subscription(
            Joy,
            'JoyStick_topic',
            self.joy_callback,
            10)
        
        # ? Store speed as an instance variable
        self.pwm_speed = 0.25  # Initial motor speed

    def joy_callback(self, msg):
        increase = msg.buttons[16]  # Get the SteamDeck up button
        decrease = msg.buttons[17]  # Get the SteamDeck down button

        # ? Modify self.pwm_speed instead of resetting
        if increase == 1:
            self.pwm_speed += 0.05
        elif decrease == 1:
            self.pwm_speed -= 0.05

        # ? Keep within bounds
        self.pwm_speed = max(0.0, min(self.pwm_speed, 1.0))

        # ? Publish updated speed
        float32_msg = Float32()
        float32_msg.data = self.pwm_speed
        self.publisher_.publish(float32_msg)

        # ? Fix logging format
        self.get_logger().info(f'Publishing motor speed: {self.pwm_speed:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedValue()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
