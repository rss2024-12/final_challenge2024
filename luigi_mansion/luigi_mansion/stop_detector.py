import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from sensor_msgs.msg import Image
from stop_detector_helpers.detector import StopSignDetector

from ackermann_msgs.msg import AckermannDriveStamped
from cs_msgs.msg import LineLocationPixel, LineLocation

class SignDetector(Node):
    def __init__(self):
        super().__init__("stop_detector")
        # self.declare_parameter("detector_drive_topic", "default")
        # self.declare_parameter("speed", 0)
        # self.declare_parameter("camera", "default")
        # self.DRIVE_TOPIC = self.get_parameter("detector_drive_topic").get_parameter_value().string_value
        # self.SPEED = self.get_parameter("speed").get_parameter_value().double_value
        self.DRIVE_TOPIC = "/vesc/high_level/input/nav_0"
        self.SPEED = 4.0
        self.CAMERA = "/zed/zed_node/rgb/image_rect_color"

        self.detector = StopSignDetector()
        self.publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 1)
        self.subscriber = self.create_subscription(Image, self.CAMERA, self.img_callback, 1)
        self.bridge = CvBridge()

        self.debug_pub = self.create_publisher(Image, "/stop_detector_box", 1) # DEBUGGING
        # homography controls
        self.min_dist = 0.5
        self.max_dist = 1.0
        self.homography_pub = self.create_publisher(LineLocationPixel, "/relative_line_px", 1)

        self.get_logger().info("Stop Detector Initialized")

    def img_callback(self, img_msg:Image):
        # Process image with CV Bridge
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        #TODO: 
        sign_found, box_coors = self.detector.predict(image)

        if sign_found:
            # DEBUG BEGIN
            debug_img = self.detector.draw_box(image, box_coors)
            debug_img_msg = self.bridge.cv2_to_imgmsg(np.array(debug_img), "bgr8")
            self.debug_pub.publish(debug_img_msg)
            # DEBUG END
            relative_msg = LineLocationPixel()
            relative_msg.u = box_coors[0]
            relative_msg.v = box_coors[1]

            self.homography_pub.publish(relative_msg)
        else:
            self.debug_pub.publish(img_msg)

    def homography_callback(self, msg:LineLocation):
        current_dist = msg.x_pos
        self.get_logger().info(f"DETECTED: {current_dist}m")
        if current_dist < self.max_dist: # Makes sure that we don't stop too soon
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0 # TODO: Make it stop gradually
            self.publisher.publish(drive_msg)
            

def main(args=None):
    rclpy.init(args=args)
    detector = SignDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
