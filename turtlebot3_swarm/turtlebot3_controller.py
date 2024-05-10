#! /usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import cv2
import numpy as np
from turtlebot3_swarm.global_reference import *

class PredictiveController(Node):

    def __init__(self):
        super().__init__('controller')
        self.rate = 10  # Hz
        self.name = self.declare_parameter('name', 'turtlebot3').get_parameter_value().string_value
        
        self.subscriber_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.subscriber_odometry = self.create_subscription(Odometry, '{}/odom'.format(self.name), self.odom_callback, 10)
        self.subscriber_scan = self.create_subscription(LaserScan, '{}/scan'.format(self.name), self.scan_callback, 10)

        self.publisher_vel = self.create_publisher(Twist, '{}/cmd_vel'.format(self.name), 10)
        self.timer = self.create_timer(1.0/self.rate, self.timer_callback)

        self.goal = PoseStamped()
        self.odom = Odometry()
        self.scan = LaserScan()

    def goal_callback(self, msg:PoseStamped):
        self.goal = msg

    def odom_callback(self, msg:Odometry):
        self.odom = msg

    def scan_callback(self, msg:LaserScan):
        self.scan = msg

    def timer_callback(self):
        self.get_logger().info("Running .....")
        
        # Source code to run 

        # Send control signal
        msg_vel = Twist()
        msg_vel.linear.x = 0.0
        msg_vel.angular.z = 0.0

        self.publisher_vel.publish(msg_vel)


def main(args=None):
    try:
        rclpy.init(args=args)
        print("[INFO] Start node")

        controller = PredictiveController()

        rclpy.spin(controller)

    except:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()