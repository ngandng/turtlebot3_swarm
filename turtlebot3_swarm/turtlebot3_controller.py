#! /usr/bin/env python

import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class NMPCFlocking(Node):

    def __init__(self):
        super().__init__('controller')
        self.rate = 10  # Hz
        self.name = self.declare_parameter('name', 'turtlebot3').get_parameter_value().string_value
        
        self.publisher_vel = self.create_publisher(Twist, '{}/cmd_vel'.format(self.name), 10)
        self.timer = self.create_timer(1.0/self.rate, self.timer_callback)

    def timer_callback(self):
        msg_vel = Twist()
        msg_vel.linear.x = 0.0
        msg_vel.angular.z = 0.1

        self.publisher_vel.publish(msg_vel)


def main(args=None):
    rclpy.init(args=args)
    print("[INFO] Start node")

    controller = NMPCFlocking()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()