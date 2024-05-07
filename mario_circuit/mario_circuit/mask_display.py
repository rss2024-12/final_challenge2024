#!/usr/bin/env python
#pulled straight from lab4, no changes should be needed 
#adjust to also publish the line being followed
import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from cs_msgs.msg import LineLocationPixel

from computer_vision.color_segmentation import draw_mask


class MaskDisplay(Node):
    def __init__(self):
        super().__init__("mask_display")

        # Subscribe to ZED camera RGB frames
        self.mask_pub = self.create_publisher(Image, "/image_mask", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("Mask Display Initialized")

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
        img_with_box = draw_mask(cv_img, self.get_logger().info)
        pub_img = self.bridge.cv2_to_imgmsg(img_with_box, "8UC1")
        #################################
        self.mask_pub.publish(pub_img)

def main(args=None):
    rclpy.init(args=args)
    mask_display = MaskDisplay()
    rclpy.spin(mask_display)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
