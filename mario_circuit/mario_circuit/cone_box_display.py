#!/usr/bin/env python
#from Lab4, no edits needed

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from cs_msgs.msg import LineLocationPixel

from computer_vision.color_segmentation import draw_bounding_box


class ConeBoxDisplay(Node):
    def __init__(self):
        super().__init__("cone_box_display")

        # Subscribe to ZED camera RGB frames
        self.box_pub = self.create_publisher(Image, "/image_bounding_box", 10)
        self.display_bounding_box = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("Cone Box Display Initialized")

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        cv_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        img_with_box = draw_bounding_box(cv_img)
        pub_img = self.bridge.cv2_to_imgmsg(img_with_box, "bgr8")
        #################################

        self.box_pub.publish(pub_img)

def main(args=None):
    rclpy.init(args=args)
    cone_box_display = ConeBoxDisplay()
    rclpy.spin(cone_box_display)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
