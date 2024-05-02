#!/usr/bin/env python]
#taken straight from lab4, edits made. Used to be cone_detector

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from vs_msgs.msg import LineLocationPixel



class LineDetector(Node):
    """
    A class for applying line detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_line_px (ConeLocationPixel) : the coordinates of the line in the image frame (units are pixels).
    The reason its a ConeLocationPixel is because I did not want to change other code
    """
    def __init__(self):
        super().__init__("line_detector")
       
       

        # Subscribe to ZED camera RGB frames
        self.line_pub = self.create_publisher(LineLocationPixel, "/relative_line_px", 10)
        self.debug_pub = self.create_publisher(Image, "/line_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("Line Detector Initialized")

    def image_callback(self, image_msg):
        # This takes in the image and will extract the line directly to the left of the car
        #this line will be white (HSV filter) and have the most positive slope
        # publish this pixel (u, v) to the /relative_line_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        cv_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
         # Convert BGR image to HSV
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    
        # Define lower and upper bounds for the line color in HSV
        lower_bound = np.array([0, 0, 200])  # Lower bound for white in HSV [0,55,106] to [11,255,255]
        upper_bound = np.array([179, 30, 255])  # Upper bound for white in HSV
       
        # Threshold the HSV image to get a binary mask
        mask = cv2.inRange(hsv_img, lower_bound, upper_bound)

        # Apply the Hough Line Transform to detect lines in the masked image
        lines = cv2.HoughLines(mask, rho=1, theta=np.pi/180, threshold=100)
        #threshhold is the number of points needed to constitute a line, adjust as needed

        max_slope_line = None
        max_slope = -np.inf

        # Iterate through the detected lines
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                # Calculate slope (m) from theta (angle)
                m = -np.tan(theta)
                if m > max_slope:
                    max_slope = m
                    max_slope_line = line
        
        if max_slope_line is not None:
            rho, theta = max_slope_line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            
            # Calculate the endpoints without extending the line
            x1 = int(x0)
            y1 = int(y0)
            x2 = int(x0)
            y2 = int(y0)
            
            # Calculate the midpoint, adjust as needed based on lookahead distance
            mid_x = (x1 + x2) // 2
            mid_y = (y1 + y2) // 2

   

        pixel = LineLocationPixel()
        pixel.u = float(mid_x)
        pixel.v = float(mid_y)
        self.line_pub.publish(pixel)
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    line_detector = LineDetector()
    rclpy.spin(line_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
