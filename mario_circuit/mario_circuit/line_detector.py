#!/usr/bin/env python]
#taken straight from lab4, edits made. Used to be cone_detector

import rclpy
from rclpy.node import Node
import numpy as np
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from cs_msgs.msg import LineLocationPixel



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
        self.valid_lines = []



    
    def image_callback(self, image_msg):
        # This takes in the image and will extract the line directly to the left of the car
        #this line will be white (HSV filter) and have the most positive slope
        # publish this pixel (u, v) to the /relative_line_px topic; the homography transformer will
        # convert it to the car frame.

        mid_x = None
        mid_y = None


        #################################
        cv_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
         # Convert BGR image to HSV
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    
        # Define lower and upper bounds for the line color in HSV
        lower_bound = np.array([0, 0, 180])  # Lower bound for white in HSV [0,55,106] to [11,255,255]
        upper_bound = np.array([179, 50, 255])  # Upper bound for white in HSV
       
        # Ignore the top half of the image
        height = image_msg.height
        width = image_msg.width
        #self.get_logger().info(f'Height:{height}')
        trapezoid_mask = np.zeros_like(hsv_img[:, :, 0])  # Rename mask to trapezoid_mask
        trap_top_width = int(0.6 * width)  # Width of the top of the trapezoid
        trap_bottom_width = width  # Width of the bottom of the trapezoid
   

    # Define the vertices of the trapezoid
        vertices = np.array([[(width - trap_bottom_width) // 2, 3 * height // 4],  # Bottom left
                        [width - (width - trap_bottom_width) // 2, 3 * height // 4],  # Bottom right
                        [width - (width - trap_top_width) // 2, 1*height // 2],  # Top right
                        [(width - trap_top_width) // 2, 1*height // 2]], dtype=np.int32)  # Top left

    # Fill the trapezoid with white (255)
        cv2.fillPoly(trapezoid_mask, [vertices], 255)

    # Threshold the HSV image to get a binary mask
        mask = cv2.inRange(hsv_img, lower_bound, upper_bound)
        mask = cv2.bitwise_and(mask, trapezoid_mask) 
       
       
        #Refine the edges, only return the edge of a line
        kernel = np.ones((5,5),np.uint8)
        mask_dilated = cv2.dilate(mask, kernel, iterations=1)
        # edges = cv2.Canny(mask_dilated, 50, 150)
        lines = cv2.HoughLines(mask_dilated, rho=1, theta=np.pi/180, threshold=100)
        #self.get_logger().info(f'Lines: {lines}')
      
        left_rho, left_theta, right_rho, right_theta = self.categorize_lines(lines)

        if any(item is None or math.isnan(item) for item in [left_rho, left_theta]):
            left_rho = 180
            left_theta = 1.15
        if any(item is None or math.isnan(item) for item in[right_rho, right_theta]):
            right_rho = -10
            right_theta = 1.9

        
        l_x , l_y = self.process_slope_line([[left_rho, left_theta]], height) 
        r_x , r_y = self.process_slope_line([[right_rho, right_theta]], height)  

        #now average the left and right hand averages to get the pixel we need
        avg_x = (l_x+r_x)//2
        avg_y = (l_y+r_y)//2
        # self.get_logger().info(f'Lines {lines}')
        # ##For debugging
        # for slope_line in self.valid_lines:
        #    # Extract the start and end points of the line
        #     rho, theta = slope_line[0]
        #     #self.get_logger().info(f'Theta: {theta}')
        #     a = np.cos(theta)
        #     b = np.sin(theta)
        #     x0 = a * rho
        #     y0 = b * rho
        #     # Calculate the endpoints of the line
        #     x1 = int(x0 + 10000 * (-b))  # Extend the line by a large factor for visualization
        #     y1 = int(y0 + 10000 * (a))
        #     x2 = int(x0 - 10000 * (-b))
        #     y2 = int(y0 - 10000 * (a))
        #     # Calculate the intersection of lane with image midline
        #     mid_y = 50*height // 100 #adjust as needed for lookahead
        #     # Polar to Cartestian Convert
        #     y_int = rho/np.sin(theta)
        #     m = -np.cos(theta)/np.sin(theta)
        #     mid_x = int((mid_y-y_int)/m)

        #     cv_img = cv2.line(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 5)  # Draw the line in green


            # cv_img = cv2.circle(cv_img, (mid_x, mid_y), 10, (255, 0, 0), -1)
        for slope_line in [[[left_rho,left_theta]],[[right_rho,right_theta]]]:
            rho, theta = slope_line[0]
            #self.get_logger().info(f'Theta: {theta}')
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            # Calculate the endpoints of the line
            x1 = int(x0 + 10000 * (-b))  # Extend the line by a large factor for visualization
            y1 = int(y0 + 10000 * (a))
            x2 = int(x0 - 10000 * (-b))
            y2 = int(y0 - 10000 * (a))
            # Calculate the intersection of lane with image midline
            mid_y = 45*height // 100 #adjust as needed for lookahead
            # Polar to Cartestian Convert
            y_int = rho/np.sin(theta)
            m = -np.cos(theta)/np.sin(theta)
            mid_x = int((mid_y-y_int)/m)

            cv_img = cv2.line(cv_img, (x1, y1), (x2, y2), (0, 0, 255), 5)  # Draw the line in green
           
          
        

        cv_img = cv2.circle(cv_img, (avg_x, avg_y), 10, (255, 0, 0), -1)
        debug_image = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
        self.debug_pub.publish(debug_image)
        # if mid_x == None and mid_y == None:
        #     mid_x = width//2
        #     mid_y = 3*height//5
        pixel = LineLocationPixel()
        pixel.u = float(avg_x)
        pixel.v = float(avg_y)
        
        self.line_pub.publish(pixel)
        #################################

        # image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        # self.debug_pub.publish(debug_msg)

        ####Test for topic publisher
        # if max_slope_line is not None:
        #     cv_img2 = cv2.line(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 5)  # Draw the line in green
        #     cv_img2 = cv2.circle(cv_img2, (mid_x, mid_y), 10, (255, 0, 0), -1)
        #     debug_image = self.bridge.cv2_to_imgmsg(cv_img2, "bgr8")
        #     self.debug_pub.publish(debug_image)

    def categorize_lines(self, lines):
        left_lines = []
        right_lines = []
        left_rho, left_theta, right_rho, right_theta = None, None, None, None
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                if 0.4 <= theta <= 1.3:
                    left_lines.append(line)
                elif 1.8 <= theta <= 2.3:
                    right_lines.append(line)

            # Calculate averages
            left_rho = np.mean([line[0][0] for line in left_lines])
            left_theta = np.mean([line[0][1] for line in left_lines])
            right_rho = np.mean([line[0][0] for line in right_lines])
            right_theta = np.mean([line[0][1] for line in right_lines])

        self.valid_lines = left_lines+right_lines
        return left_rho, left_theta, right_rho, right_theta
            


        

    def process_slope_line(self, slope_line, height):
        
        rho, theta = slope_line[0]
        #self.get_logger().info(f'rho,theta: {rho,theta}')
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 10000 * (-b))
        y1 = int(y0 + 10000 * (a))
        x2 = int(x0 - 10000 * (-b))
        y2 = int(y0 - 10000 * (a))
        mid_y = 45* height // 100
        y_int = rho / np.sin(theta)
        m = -np.cos(theta) / np.sin(theta)
        mid_x = int((mid_y - y_int) / m)
    
        return mid_x,mid_y

def main(args=None):
    rclpy.init(args=args)
    line_detector = LineDetector()
    rclpy.spin(line_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

