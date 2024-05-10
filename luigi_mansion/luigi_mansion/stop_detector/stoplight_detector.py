import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

class StopLightDetector(Node):
    """
    A class for applying line detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_line_px (ConeLocationPixel) : the coordinates of the line in the image frame (units are pixels).
    The reason its a ConeLocationPixel is because I did not want to change other code
    """
    def __init__(self):
        super().__init__("stoplight_detector")
       
       

        # Subscribe to ZED camera RGB frames
        self.stoplight_pub = self.create_publisher(LineLocationPixel, "/stop_light", 10)
    
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("StopLight Detector Initialized")
        self.debug_image = self.create_publisher(Image, "/debug")



    
    def image_callback(self, image_msg):
        # This takes in the image and will extract the line directly to the left of the car
        #this line will be white (HSV filter) and have the most positive slope
        # publish this pixel (u, v) to the /relative_line_px topic; the homography transformer will
        # convert it to the car frame.

        cv_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
         # Convert BGR image to HSV
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    
        # Define lower and upper bounds for the line color in HSV
        lower_bound = np.array([170, 100, 100])  # Lower bound for white in HSV [0,55,106] to [11,255,255]
        upper_bound = np.array([10, 255, 255])  # Upper bound for white in HSV
       
        # Ignore the top half of the image
        height = image_msg.height
        width = image_msg.width
        #self.get_logger().info(f'Height:{height}')
        top_mask = np.zeros_like(hsv_img[:, :, 0])
        top_mask[height//2:, :] = 255
        

        # Threshold the HSV image to get a binary mask, combine with top half mask
        mask = cv2.inRange(hsv_img, lower_bound, upper_bound)
        mask = cv2.bitwise_and(mask, top_mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Initialize bounding box
        box = ((0, 0), (0, 0))
    
    # Iterate through contours and find the bounding box of the largest contour
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            box = ((x, y), (x + w, y + h))
       
        x=(box[0][0]+box[1][0])//2 #takes the middle 
        y=(box[1][0]+box[1][1])//2#grabs the point on the bottom of the bounding box
        pixel = ConeLocationPixel()
        pixel.u = float(x)
        pixel.v = float(y)
        self.stoplight_pub.publish(pixel)



def main(args=None):
    rclpy.init(args=args)
    stopsign_detector = StopLightDetector()
    rclpy.spin(stopsign_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()