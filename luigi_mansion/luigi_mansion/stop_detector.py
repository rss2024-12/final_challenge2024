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
        self.max_dist = 1.0

        # alt dist sol
        focus = 275
        self.convert = lambda x: 0.2032*focus/x
        self.img = None

        self.timer_period = 0.5
        self.stop_cycles = 0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Stop Detector Initialized")

    def img_callback(self, img_msg):
        self.img = img_msg
    
    def timer_callback(self):
        # Process image with CV Bridge
        if self.img:
            image = self.bridge.imgmsg_to_cv2(self.img, "bgr8")

            sign_found, box_coors = self.detector.predict(image)

            if sign_found:
                pixel_width = abs(box_coors[0]-box_coors[2])
                current_dist = self.convert(pixel_width)
                if current_dist < self.max_dist and self.stop_cycles < 5/self.timer_period:
                    self.get_logger().info("Stopped")
                    drive_msg = AckermannDriveStamped()
                    drive_msg.drive.speed = 0.0 # TODO: Make it stop gradually
                    self.publisher.publish(drive_msg)
                    self.stop_cycles += 1
                else:
                    self.get_logger().info("Running")
                    drive_msg = AckermannDriveStamped()
                    drive_msg.drive.speed = self.SPEED
                    self.publisher.publish(drive_msg)
            else:
                self.stop_cycles = 0

def main(args=None):
    rclpy.init(args=args)
    detector = SignDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
