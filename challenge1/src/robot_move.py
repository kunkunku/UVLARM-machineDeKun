#!/usr/bin/python3

import rclpy
import math
import random
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32

class Robot(Node):
    def __init__(self, name):
        super().__init__(name)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.state = "forward"
        # self.obstacles_left = False
        # self.obstacles_right = False
        # self.very_close_to_obstacle = False

    def scan_callback(self, scanMsg):
        obstacles = []
        self.obstacles_left = False
        self.obstacles_right = False
        self.very_close_to_obstacle = False

        angle = scanMsg.angle_min

        for aDistance in scanMsg.ranges:
            if 0.1 < aDistance < 5.0:
                # aPoint = [math.cos(angle) * aDistance, math.sin(angle) * aDistance]
                aPoint= Point32()
                aPoint.x= (float)(math.cos(angle) * aDistance)
                aPoint.y= (float)(math.sin( angle ) * aDistance)
                aPoint.z= (float)(0)
                obstacles.append(aPoint) 
                self.detect_obstacles(obstacles)
            angle += scanMsg.angle_increment

        if self.very_close_to_obstacle or self.obstacles_left or self.obstacles_right:
            self.state = "turn"
        else:
            self.state = "forward"

        self.movement ()
    
    def movement(self):
        if self.state == "forward":
            self.go_forward()
        else:
            self.turn_or_reverse()

    def detect_obstacles(self, obstacles):
        # self.obstacles_left = False
        # self.obstacles_right = False
        # self.very_close_to_obstacle = False
        close_threshold_x= 0.2  
        close_threshold_y= 0.2
        for i in range (len(obstacles)):
            if abs(obstacles[i].x) < close_threshold_x and abs(obstacles[i].y) < close_threshold_y:
                self.very_close_to_obstacle = True
            elif 0.2 < obstacles[i].y < 0.5 and 0.2 < obstacles[i].x < 0.5:
                self.obstacles_left = True
            elif -0.5 < obstacles[i].y  < 0.2 and 0.2 < obstacles[i].x < 0.5:
                self.obstacles_right = True
        # print(f"Obstacle Detected - Left: {self.obstacles_left}, Right: {self.obstacles_right}, Very Close: {self.very_close_to_obstacle}")
    
    def go_forward(self):
        velo = Twist()
        # if self.very_close_to_obstacle:
        #     velo.linear.x = 0.0
        #     velo.angular.z = 0.0
        #     print("Very close to obstacle, stopping")
        # if not self.obstacles_left and not self.obstacles_right:
        velo.linear.x = 0.3
        velo.linear.z = 0.0
        print("Path clear, moving forward")
        # else:
        #     velo.linear.x = 0.0
        #     print("Obstacle detected, not moving forward")
        self.velocity_publisher.publish(velo)

    # def choose_direction(self):
    #     if self.obstacles_left and self.obstacles_right is False:
    #         return -0.8
    #     elif self.obstacles_right and self.obstacles_left is False:
    #         return 0.8
    #     elif self.obstacles_left and self.obstacles_right:
    #         return -0.8
        
    #     else:
    #         return None

    def turn_or_reverse(self):
        # turn_direction = self.choose_direction()
        velo = Twist()
        velo.linear.x = 0.0
        if self.obstacles_left and not self.obstacles_right :
            velo.angular.z = -1.0
        elif self.obstacles_right and not self.obstacles_left :
            velo.angular.z = 1.0
        elif self.obstacles_left and self.obstacles_right:
            random_num = random.random()
            if random_num < 0.5:
                velo.angular.z = -1.0
            else
                velo.angular.z = 1.0
        
        # if turn_direction is not None:
        #     velo.angular.z = turn_direction
        self.velocity_publisher.publish(velo)
        print("turning to avoid obstacles")

    # def emergency_maneuver(self):
    #     velo = Twist()
    #     velo.linear.x = 0.0
    #     velo.angular.z = 0.3
    #     self.velocity_publisher.publish(velo)

def main():
    rclpy.init()
    robot = Robot("challenge1")
    while rclpy.ok():
        # robot.go_forward()
        # if robot.very_close_to_obstacle:
        #     robot.emergency_maneuver()
        # elif robot.obstacles_left or robot.obstacles_right:
        #     robot.turn_or_reverse()

        # robot.obstacles_left = False
        # robot.obstacles_right = False
        # robot.very_close_to_obstacle = False
        rclpy.spin_once(robot, timeout_sec=0.1)

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
