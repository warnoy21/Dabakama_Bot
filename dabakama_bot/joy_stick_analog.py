######################################################################################
"""
Description : ROS2 publisher Joystick using the SteamDeck to control Dabakama Robot.            
             Take the input of the controller steamdeck and publish it in "JoyStick_topic"

Version: 1.0        Initial Creation : January 20, 2025     Created by: Aaron Gumba

"""
#######################################################################################



#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher_node')

        # Initialize the pygame joystick module
        pygame.init()
        pygame.joystick.init()

        # Check for connected joysticks
        self.joystick_count = pygame.joystick.get_count()
        if self.joystick_count == 0:
            self.get_logger().error("No joystick detected!")
            raise RuntimeError("No joystick connected")

        # Initialize the first joystick
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Joystick connected: {self.joystick.get_name()}")

        # Create a publisher for the Joy message
        self.publisher = self.create_publisher(Joy, 'JoyStick_topic', 10)

        # Timer for publishing at 5 Hz
        self.timer = self.create_timer(0.2, self.publish_joystick_data)

    def publish_joystick_data(self):
        # Update joystick events (required by pygame)
        pygame.event.pump()

        # Get the axes and button states
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]

        # Create and populate a Joy message
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes
        msg.buttons = buttons

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published joystick data: Axes: {msg.axes}, Buttons: {msg.buttons}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = JoystickPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(e)
    finally:
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
