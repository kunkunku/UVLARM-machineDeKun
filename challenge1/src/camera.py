#!/usr/bin/env python3
# Realsense Node:
#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

#!/usr/bin/env python3

#!/usr/bin/env python3

import pyrealsense2 as rs
import cv2
import numpy as np
import signal
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

# Define the global flag for exiting
isOk = True

class Realsense(Node):
    def __init__(self):
        super().__init__('realsense_node')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'image', 10)
        self.detection_publisher = self.create_publisher(String, 'detection', 10)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
        self.pipeline.start(self.config)
        self.isOk = True

    def signal_interuption(self, signum, frame):
        global isOk
        print("\nCtrl-c pressed")
        isOk = False

    def detect_green_object(self, color_image):
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=10)
        
        green_object = cv2.bitwise_and(color_image, color_image, mask=mask)
        
        num_green_pixels = np.count_nonzero(mask)
        
        if num_green_pixels > 1000:  # Adjust the threshold as needed
            print("yes")
            return True, green_object
        else:
            print("no")
            return False, green_object


    def read_imgs(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())
        msg_image = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)

        # Detect green object and publish detection result
        is_green_object_detected = self.detect_green_object(color_image)
        detection_msg = String()
        detection_msg.data = "bottle founded" if is_green_object_detected else "bottle unfounded"
        self.detection_publisher.publish(detection_msg)

        images = np.hstack((color_image, self.get_depth_colormap(depth_frame)))

        # Show images
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

    def get_depth_colormap(self, depth_frame):
        # Convert the depth frame to a colormap
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(np.asanyarray(depth_frame.get_data()), alpha=0.03), cv2.COLORMAP_JET)
        return depth_colormap


    def run(self):
        while isOk:
            self.read_imgs()
            rclpy.spin_once(self, timeout_sec=0.001)

        # Stop streaming
        print("Ending...")
        self.pipeline.stop()

def main(args=None):
    global isOk
    rclpy.init(args=args)
    rsNode = Realsense()

    signal.signal(signal.SIGINT, rsNode.signal_interuption)

    try:
        rsNode.run()
    except KeyboardInterrupt:
        pass

    # Clean up
    rclpy.shutdown()

if __name__ == '__main__':
    main()
