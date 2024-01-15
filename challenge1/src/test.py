# pour tester le camera

# import pyrealsense2 as rs
# import cv2
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import sys
# import time
# import signal

# class Realsense(Node):
#     def __init__(self, fps=60):
#         super().__init__('realsense')
#         self.image_publisher = self.create_publisher(Image, 'image', 10)
#         self.detection_publisher = self.create_publisher(String, 'detection', 10)
#         self.bridge = CvBridge()

#         # RealSense pipeline configuration
#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, fps)
#         config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, fps)
#         self.pipeline.start(config)

#         self.isOk = True

#     def read_imgs(self):
#         frames = self.pipeline.wait_for_frames()
#         color_frame = frames.get_color_frame()
#         depth_frame = frames.get_depth_frame()

#         if not color_frame or not depth_frame:
#             return

#         # Convert images to numpy arrays
#         depth_image = np.asanyarray(depth_frame.get_data())
#         self.color_image = np.asanyarray(color_frame.get_data())

#         # RGB color space processing
#         lower_rgb = np.array([RGB下限值], dtype="uint8")  # Replace with actual values
#         upper_rgb = np.array([RGB上限值], dtype="uint8")  # Replace with actual values
#         mask = cv2.inRange(self.color_image, lower_rgb, upper_rgb)
#         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

#         # Show images
#         images = np.hstack((self.color_image, depth_colormap))
#         cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
#         cv2.imshow('RealSense', images)
#         cv2.waitKey(1)

#     def publish_imgs(self):
#         msg_image = self.bridge.cv2_to_imgmsg(self.color_image, "bgr8")
#         msg_image.header.stamp = self.get_clock().now().to_msg()
#         msg_image.header.frame_id = "image"
#         self.image_publisher.publish(msg_image)

#         detection_msg = String()
#         detection_msg.data = "bottle founded" if self.test else "bottle unfounded"
#         self.detection_publisher.publish(detection_msg)

# def process_img(args=None):
#     rclpy.init(args=args)
#     rsNode = Realsense()
#     signal.signal(signal.SIGINT, rsNode.signalInteruption)

#     while rsNode.isOk:
#         rsNode.read_imgs()
#         rsNode.publish_imgs()
#         rclpy.spin_once(rsNode, timeout_sec=0.001)

#     print("Ending...")
#     rsNode.pipeline.stop()
#     rsNode.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     process_img()#         self.test = np.sum(mask) > 1000  # Set test flag based on detection

#         self.color_image = cv2.bitwise_and(self.color_image, self.color_image, mask=mask)

#         # Apply colormap on depth image
#         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

#         # Show images
#         images = np.hstack((self.color_image, depth_colormap))
#         cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
#         cv2.imshow('RealSense', images)
#         cv2.waitKey(1)

#     def publish_imgs(self):
#         msg_image = self.bridge.cv2_to_imgmsg(self.color_image, "bgr8")
#         msg_image.header.stamp = self.get_clock().now().to_msg()
#         msg_image.header.frame_id = "image"
#         self.image_publisher.publish(msg_image)

#         detection_msg = String()
#         detection_msg.data = "bottle founded" if self.test else "bottle unfounded"
#         self.detection_publisher.publish(detection_msg)

# def process_img(args=None):
#     rclpy.init(args=args)
#     rsNode = Realsense()
#     signal.signal(signal.SIGINT, rsNode.signalInteruption)

#     while rsNode.isOk:
#         rsNode.read_imgs()
#         rsNode.publish_imgs()
#         rclpy.spin_once(rsNode, timeout_sec=0.001)

#     print("Ending...")
#     rsNode.pipeline.stop()
#     rsNode.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     process_img()


# import pyrealsense2 as rs
# import cv2
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import sys
# import time
# import signal

# class RealSenseNode(Node):
#     def __init__(self, fps=60):
#         super().__init__('realsense_node')
#         self.declare_parameters(namespace='', parameters=[
#             ('fps', fps),
#             ('rgb_camera_name', 'RGB Camera')
#         ])

#         self.fps = self.get_parameter('fps').value
#         self.rgb_camera_name = self.get_parameter('rgb_camera_name').value

#         self.image_publisher = self.create_publisher(Image, 'image', 10)
#         self.detection_publisher = self.create_publisher(String, 'detection', 10)
#         self.bridge = CvBridge()
#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, self.fps)
#         config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, self.fps)
#         self.pipeline.start(config)

#     def run(self):
#         try:
#             while rclpy.ok():
#                 self.process_images()
#         finally:
#             self.pipeline.stop()

#     def process_images(self):
#         frames = self.pipeline.wait_for_frames()
#         color_frame = frames.get_color_frame()
#         depth_frame = frames.get_depth_frame()

#         if not color_frame or not depth_frame:
#             return

#         color_image = np.asanyarray(color_frame.get_data())
#         depth_image = np.asanyarray(depth_frame.get_data())

#         # Image processing
#         color_image = self.detect_object(color_image)

#         # Publishing
#         self.publish_images(color_image)

#     def detect_object(self, image):
#         hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#         lower_green = np.array([45, 150, 100])
#         upper_green = np.array([75, 255, 255])
#         mask = cv2.inRange(hsv_image, lower_green, upper_green)
#         detection_result = "bottle founded" if np.sum(mask) > 1000 else "bottle unfounded"
#         self.detection_publisher.publish(String(data=detection_result))
#         return cv2.bitwise_and(image, image, mask=mask)

#     def publish_images(self, image):
#         msg_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
#         msg_image.header.stamp = self.get_clock().now().to_msg()
#         msg_image.header.frame_id = "image"
#         self.image_publisher.publish(msg_image)

# def main(args=None):
#     rclpy.init(args=args)
#     real_sense_node = RealSenseNode()
#     signal.signal(signal.SIGINT, lambda sig, frame: rclpy.shutdown())
#     real_sense_node.run()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import pyrealsense2 as rs
# import signal, time, numpy as np
# import sys, cv2, rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image 
# from cv_bridge import CvBridge
# from std_msgs.msg import String
# import cv2
# import numpy as np
# import os
# from sklearn.svm import LinearSVC
# from scipy.cluster.vq import *
# from sklearn.preprocessing import StandardScaler
# from sklearn import preprocessing

# # Node processes:
# def process_img(args=None):
#     rclpy.init(args=args)
#     rsNode= Realsense()

#     signal.signal(signal.SIGINT, rsNode.signalInteruption)

#     while rsNode.isOk:
#         rsNode.read_imgs()
#         rsNode.publish_imgs()
#         rclpy.spin_once(rsNode, timeout_sec=0.001)

#     # Stop streaming
#     print("Ending...")
#     rsNode.pipeline.stop()
#     # Clean end
#     rsNode.destroy_node()
#     rclpy.shutdown()





# # Realsense Node:
# class Realsense(Node):
#     def __init__(self, fps= 60):
#         super().__init__('realsense')
#         self.image_publisher = self.create_publisher(Image,'image',10)
#         self.detection_publisher = self.create_publisher(String,'detection',10)
#         self.bridge=CvBridge()
#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
#         pipeline_profile = config.resolve(pipeline_wrapper)
#         device = pipeline_profile.get_device()
#         device_product_line = str(device.get_info(rs.camera_info.product_line))
#         found_rgb = True
#         for s in device.sensors:
#             print( "Name:" + s.get_info(rs.camera_info.name) )
#             if s.get_info(rs.camera_info.name) == 'RGB Camera':
#                 found_rgb = True
#         if not (found_rgb):
#             print("Depth camera equired !!!")
#             exit(0)
#         config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
#         config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
# # Start streaming
#         self.pipeline.start(config)
#         self.isOk = True
       
    
    
#     def read_imgs(self):
#         sys.stdout.write("-")
#         count= 1
#         refTime= time.process_time()
#         freq= 60
#         self.test = False

        
#         # Wait for a coherent tuple of frames: depth, color and accel
#         frames = self.pipeline.wait_for_frames()

#         color_frame = frames.first(rs.stream.color)
#         depth_frame = frames.first(rs.stream.depth)

#         if not (depth_frame and color_frame):
#             return

#         # Convert images to numpy arrays
#         depth_image = np.asanyarray(depth_frame.get_data())
#         self.color_image = np.asanyarray(color_frame.get_data())


#         hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
#         h,s,v = cv2.split(hsv)
         
#         print(h)
#         print(s)
#         print(v)
#         hmin = 45
#         hmax = 75
#         tmp = np.uint8((hmin < h)&(h < hmax) & (s> 150) & (v> 100))

#         if(np.sum(tmp)>1000):
#             self.test = True


#         tmp = tmp*255
#         self.color_image=cv2.merge((tmp,tmp,tmp))
#         kernel = np.ones((3,3),np.uint8)
#         self.color_image = cv2.dilate(self.color_image,kernel,iterations = 10)


#         # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
#         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

#         depth_colormap_dim = depth_colormap.shape
#         color_colormap_dim = self.color_image.shape

#         sys.stdout.write( f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(freq)} fps)" )

#         # Show images
#         images = np.hstack((self.color_image, depth_colormap))

#         # Show images
#         cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
#         cv2.imshow('RealSense', images)
#         cv2.waitKey(1)

#         # Frequency:
#         if count == 10 :
#             newTime= time.process_time()
#             freq= 10/((newTime-refTime))
#             refTime= newTime
#             count= 0
#         count+= 1

#     # Capture ctrl-c event
#     def signalInteruption(signum, frame):
#         print( "\nCtrl-c pressed" )
#         self.isOk= False

#     def publish_imgs(self):
#         msg_image = self.bridge.cv2_to_imgmsg(self.color_image,"bgr8")
#         msg_image.header.stamp = self.get_clock().now().to_msg()
#         msg_image.header.frame_id = "image"
#         self.image_publisher.publish(msg_image)
#         myStr = String()
#         if self.test == True:
#             myStr.data = "bottle founded"
#         else:
#             myStr.data = "bottle unfounded"
#         self.detection_publisher.publish(myStr)
        



# process_img()

