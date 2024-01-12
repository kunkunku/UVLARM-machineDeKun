#!/usr/bin/python3

import rclpy
import time
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32

class Robot(Node):
    def __init__(self, name):
        super().__init__(name)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.obstacles_left = False
        self.obstacles_right = False

    def scan_callback(self, scanMsg):
        angle = scanMsg.angle_min
        obstacles = []
        
        for aDistance in scanMsg.ranges:
            if 0.1 < aDistance < 5.0:
                aPoint = [math.cos(angle) * aDistance, math.sin(angle) * aDistance]
                self.detect_obstacles(aPoint)
            angle += scanMsg.angle_increment

    def detect_obstacles(self, aPoint):
        if 0.01 < aPoint[0] < 0.5 and 0.15 < aPoint[1] < 1:
            self.obstacles_right = True
        if -0.5 < aPoint[0] < -0.01 and 0.15 < aPoint[1] < 1:
            self.obstacles_left = True

    def go_forward(self):
        velo = Twist()
        if not self.obstacles_left and not self.obstacles_right:
            velo.linear.x = 0.2
            velo.angular.z = 0.0
        else:
            velo.linear.x = 0.0
            velo.angular.z = 0.0
        self.velocity_publisher.publish(velo)

    def turn(self):
        velo = Twist()
        if self.obstacles_left:
            print("Obstacle on left, turning right")
            velo.angular.z = -0.5
        elif self.obstacles_right:
            print("Obstacle on right, turning left")
            velo.angular.z = 0.5
        else:
            return
        velo.linear.x = 0.0
        time.sleep(0.2)
        self.velocity_publisher.publish(velo)
        

def main():
    rclpy.init()
    robot = Robot("scan_interpreter")

    while rclpy.ok():
        robot.go_forward()
        if robot.obstacles_left or robot.obstacles_right:
            robot.turn()
        # Reset obstacle detection
        robot.obstacles_left = False
        robot.obstacles_right = False
        rclpy.spin_once(robot, timeout_sec=0.1)

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
