#!/usr/bin/python3

import rclpy
import math
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Robot(Node):
    def __init__(self, name):
        super().__init__(name)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.obstacles_left = False
        self.obstacles_right = False
        self.very_close_to_obstacle = False

    def scan_callback(self, scanMsg):
        self.obstacles_left = False
        self.obstacles_right = False
        self.very_close_to_obstacle = False
        angle = scanMsg.angle_min

        for aDistance in scanMsg.ranges:
            if 0.1 < aDistance < 5.0:
                aPoint = [math.cos(angle) * aDistance, math.sin(angle) * aDistance]
                self.detect_obstacles(aPoint)
            angle += scanMsg.angle_increment

    def detect_obstacles(self, aPoint):
        close_threshold_x= 0.2  
        close_threshold_y= 0.2
        if abs(aPoint[0]) < close_threshold_x and abs(aPoint[1]) < close_threshold_y:
            self.very_close_to_obstacle = True
        elif 0.2 < aPoint[0] < 0.3 and 0.2 < aPoint[1] < 0.3:
            self.obstacles_right = True
        elif -0.3 < aPoint[0] < -0.2 and 0.2 < aPoint[1] < 0.3:
            self.obstacles_left = True
        print(f"Obstacle Detected - Left: {self.obstacles_left}, Right: {self.obstacles_right}, Very Close: {self.very_close_to_obstacle}")
    def go_forward(self):
        velo = Twist()
        if self.very_close_to_obstacle:
            velo.linear.x = 0.0
            velo.angular.z = 0.0
            print("Very close to obstacle, stopping")
        elif not self.obstacles_left and not self.obstacles_right:
            velo.linear.x = 0.3
            print("Path clear, moving forward")
        else:
            velo.linear.x = 0.0
            print("Obstacle detected, not moving forward")
        self.velocity_publisher.publish(velo)

    def choose_direction(self):
        if self.obstacles_left and self.obstacles_right is False:
            return -0.8
        elif self.obstacles_right and self.obstacles_left is False:
            return 0.8
        elif self.obstacles_left and self.obstacles_right:
            return -0.8
        
        else:
            return None

    def turn_or_reverse(self):
        turn_direction = self.choose_direction()
        velo = Twist()
        velo.linear.x = 0.0
        if turn_direction is not None:
            velo.angular.z = turn_direction
        self.velocity_publisher.publish(velo)

    def emergency_maneuver(self):
        velo = Twist()
        velo.linear.x = 0.0
        velo.angular.z = 0.3  
        self.velocity_publisher.publish(velo)

def main():
    rclpy.init()
    robot = Robot("challenge1")
    while rclpy.ok():
        robot.go_forward()
        if robot.very_close_to_obstacle:
            robot.emergency_maneuver()
        elif robot.obstacles_left or robot.obstacles_right:
            robot.turn_or_reverse()

        robot.obstacles_left = False
        robot.obstacles_right = False
        robot.very_close_to_obstacle = False
        rclpy.spin_once(robot, timeout_sec=0.1)

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
