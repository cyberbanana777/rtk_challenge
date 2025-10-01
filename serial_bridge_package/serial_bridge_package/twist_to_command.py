#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
Конвертер сообщений Twist в кастомный протокол для ESP32 с форматом
$linear;angular#. Извлекает только линейную скорость по X и угловую
по Z, игнорируя остальные компоненты. Использует стандартный топик
/cmd_vel для совместимости с ROS-навигационными стеками.

ANNOTATION
Twist message converter to custom ESP32 protocol in $linear;angular# format.
Extracts only linear X and angular Z velocities while ignoring other
components. Uses standard /cmd_vel topic for compatibility with ROS navigation
stacks.
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TwistToCommand(Node):
    """Node for converting Twist messages to custom ESP32 command protocol."""

    def __init__(self):
        super().__init__('twist_to_command')

        self.declare_parameter('Kp_left', 2.075)
        self.declare_parameter('Kd_left', 0.005)
        self.declare_parameter('Ki_left', 0.0)

        self.declare_parameter('Kp_right', 2.075)
        self.declare_parameter('Kd_right', 0.005)
        self.declare_parameter('Ki_right', 0.0)

        self.Kp_left = str(self.get_parameter('Kp_left').value)
        self.Kd_left = str(self.get_parameter('Kd_left').value)
        self.Ki_left = str(self.get_parameter('Ki_left').value)

        self.Kp_right = str(self.get_parameter('Kp_right').value)
        self.Kd_right = str(self.get_parameter('Kd_right').value)
        self.Ki_right = str(self.get_parameter('Ki_right').value)
        
        # Subscription to Twist topic (movement control)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Standard topic for movement control
            self.twist_callback,
            10
        )
        
        # Publisher for ESP32 communication topic
        self.publisher = self.create_publisher(
            String,
            '/esp32_input',  # Topic used for sending data to ESP32
            10
        )
        
        self.get_logger().info(
            'Twist-to-command node started. Waiting for Twist commands...'
        )

    def twist_callback(self, msg):
        """Convert Twist message to ESP32 command string."""
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Format message according to protocol
        command = f"${linear_x:.3f};{angular_z:.3f};{self.Kp_left};{self.Kd_left};{self.Ki_left};{self.Kp_right};{self.Kd_right};{self.Ki_right};#"
        
        # Create and publish String message
        string_msg = String()
        string_msg.data = command
        
        self.publisher.publish(string_msg)
        self.get_logger().debug(f'Sent command: {command}')


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    node = TwistToCommand()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Twist-to-command node stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()