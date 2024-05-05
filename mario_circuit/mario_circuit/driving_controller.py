#!/usr/bin/env python
#this is pulled straight from Lab 4, name changed to driving_controller
#changes will be needed

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import LineLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class DrivingController(Node):
    """
    A controller for driving while following a line.
    Listens for a relative line location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("driving_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar
        self.declare_parameter("drive_speed")
        self.speed = self.get_parameter("drive_speed").value#from params

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/driving_error", 10)

        self.create_subscription(LineLocation, "/relative_line", 
            self.relative_line_callback, 1)


        self.relative_x = 0
        self.relative_y = 0

        self.get_logger().info("Driving Controller Initialized")

    def relative_line_callback(self, msg):
        lane_width = 38 * 0.0254 #gives Johnson track lane width in m
        half_width = lane_width/2

        self.relative_x = msg.x_pos 
        self.relative_y = msg.y_pos - half_width #potential error, I think x is forward

        drive_cmd = AckermannDriveStamped()
        
        L = .32 # distance between front and back wheels im m
        
        L1 = np.sqrt((self.relative_x)**2 + (self.relative_y)**2) 
        theta = np.arctan(self.relative_x/self.relative_y)
        eta = np.pi/2 - theta
        delta = np.arctan((2*L*np.sin(eta))/L1) 

       
        drive_cmd.drive.steering_angle = delta
        drive_cmd.drive.speed = self.speed
        self.drive_pub.publish(drive_cmd)
        

def main(args=None):
    rclpy.init(args=args)
    pc = DrivingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
