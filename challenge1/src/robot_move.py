#!/usr/bin/python3

import rclpy
import math
import random
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

class Robot(Node):
    def __init__(self, name):
        super().__init__(name)
        self.velocity_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        self.state = "forward"

        self.marker_publisher = self.create_publisher( Marker,'marker_test',10)
        self.create_subscription( Odometry, 'Odom', self.odometry_callback, 10)
        self.create_subscription( Float32, 'bottle_x_postion',self.bottle_x_postion_callback, 10)
        self.create_subscription( Float32, 'bottle_z_postion',self.bottle_z_postion_callback, 10)
        self.create_subscription( String, 'detection',self.detection_callback, 10)
        self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        self.cloud_publisher = self.create_publisher(pc2.PointCloud2,'laser_link',10)
        self.isOk = True
        self.detection = False

    def scan_callback(self, scanMsg):
        
        obstacles = []
        self.obstacles_left = False
        self.obstacles_right = False
        self.very_close_to_obstacle = False

        angle = scanMsg.angle_min

        if self.detection == True:

            # global_x, global_y = self.transform_to_global(self.x_add, self.y_add, x_label, y_label, self.orientation_euler)

            marker = Marker()
            marker.header.frame_id = 'camera_link'
            marker.type = Marker.SPHERE
            marker.pose.position.x = self.x_label
            marker.pose.position.y = self.y_label
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            self.marker_publisher.publish(marker)


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
        
        # cloudPoints = pc2.create_cloud_xyz32(Header(frame_id='laser_link'),obstacles)
        # self.publish_move(velo, cloudPoints)
        
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
        close_threshold_x= 0.1  
        close_threshold_y= 0.1
        for i in range (len(obstacles)):
            if abs(obstacles[i].x) < close_threshold_x and abs(obstacles[i].y) < close_threshold_y:
                self.very_close_to_obstacle = True
            elif 0.1 < obstacles[i].y < 0.3 and 0.1 < obstacles[i].x < 0.3:
                self.obstacles_left = True
            elif -0.3 < obstacles[i].y  < 0.1 and 0.1 < obstacles[i].x < 0.3:
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
        velo.linear.x = 0.3
        if self.obstacles_left and not self.obstacles_right :
            velo.angular.z = -2.5
        elif self.obstacles_right and not self.obstacles_left :
            velo.angular.z = 2.5
        elif self.obstacles_left and self.obstacles_right:
            velo.linear.x = 0.02
            random_num = random.random()
            if random_num < 0.5:
                velo.angular.z = -4.0
            else:
                velo.angular.z = 4.0
        
        # if turn_direction is not None:
        #     velo.angular.z = turn_direction
        self.velocity_publisher.publish(velo)
        print("turning to avoid obstacles")

    # def emergency_maneuver(self):
    #     velo = Twist()
    #     velo.linear.x = 0.0
    #     velo.angular.z = 0.3
    #     self.velocity_publisher.publish(velo)
    # def transform_to_global(self, robot_x, robot_y, rel_x, rel_y, orientation_euler):
  
    #     angle = orientation_euler[2]  
    #     cos_angle = math.cos(angle)
    #     sin_angle = math.sin(angle)

        
    #     global_x = cos_angle * rel_x - sin_angle * rel_y
    #     global_y = sin_angle * rel_x + cos_angle * rel_y

        
    #     global_x += robot_x
    #     global_y += robot_y

    #     return global_x, global_y

    def detection_callback(self,detectionMsg):
        if detectionMsg.data == "bottle unfounded":
            print("bottle unfounded")
            self.detection = False
        elif detectionMsg.data == "bottle founded":
            print("bottle founded")
            self.detection = True
    
    def bottle_x_postion_callback(self,msg):
        self.x_label = msg.data
    
    def bottle_z_postion_callback(self,msg):
        self.y_label = msg.data
    
    def odometry_callback(self, msg):
        self.x_add = msg.pose.pose.position.x
        self.y_add = msg.pose.pose.position.y
        self.orientation = msg.pose.pose.orientation
        self.orientation_euler = euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        # position = data.pose.pose.position
        # orientation = data.pose.pose.orientation
        # orientation_euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

def main():
    rclpy.init()
    robot = Robot("challenge")
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
