######################################################################################
"""
Description : ROS2 publisher and subscriber using the SteamDeck to control Dabakama Robot.
              Converts the value of the joystick steamdeck analog to Twist message type.
              Subscribed to "JoyStick_topic" and publishes "motor_value_topic"

Version: 1.0        Initial Creation : January 21, 2025     Created by: Aaron Gumba

"""
#######################################################################################



#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoystickTurtleConverter(Node):
    def __init__(self):
        super().__init__('joystick_converter_node')
        self.publisher_ = self.create_publisher(Twist, 'motor_value_topic', 10)
        self.subscription = self.create_subscription(
            Joy,
            'JoyStick_topic',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning

    def joy_callback(self, msg):
        twist_msg = Twist()
        
        # Map joystick axes to turtle movement
        twist_msg.linear.x = msg.axes[1]  # Usually, the Y-axis controls linear speed (forward/backward)
        twist_msg.angular.z = msg.axes[0]  # Usually, the X-axis controls angular speed (rotation)

        # Publish the Twist message
        self.publisher_.publish(twist_msg)

        self.get_logger().info('Publishing velocity: Linear: %.2f, Angular: %.2f' %
                               (twist_msg.linear.x, twist_msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    joystick_turtle_controller = JoystickTurtleConverter()
    rclpy.spin(joystick_turtle_controller)
    joystick_turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
