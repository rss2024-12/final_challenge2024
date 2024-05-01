#!/usr/bin/env python
#this is pulled straight from Lab 4, name changed to driving_controller
#changes will be needed

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        self.parking_distance = .6 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.get_logger().info("Parking Controller Initialized")

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        
        L = .32 # distance between front and back wheels

        ## we will need to consider the case where the lidar does not detect the cone (?)
        L1 = np.sqrt((self.relative_x)**2 + (self.relative_y)**2) - self.parking_distance # distance to parking spot
        theta = np.arctan(self.relative_x/self.relative_y)
        eta = np.pi/2 - theta
        delta = np.arctan((2*L*np.sin(eta))/L1) 

        # this may need to be changed whenever we synthesize this to follow a line
        if self.relative_y > 0.05:
            drive_cmd.drive.steering_angle = delta
        elif self.relative_y < -0.05:
            drive_cmd.drive.steering_angle = -delta
        else:
            drive_cmd.drive.steering_angle = 0.0
        # Go forward
        if L1 > 0:
            if L1 > self.parking_distance: #full steam ahead if you are far
                drive_cmd.drive.speed = 1.25
            elif self.parking_distance >= L1 > 0.05:
                drive_cmd.drive.speed = max(1.25*L1/self.parking_distance, .75) #the car should slow down as it approaches
            elif L1 <= 0.05:
                drive_cmd.drive.speed = 0.

        # Go Backward
        elif L1 < 0:
            if L1 < -0.05:
                drive_cmd.drive.speed = -1.0
            elif L1 >= -0.05:
                if np.abs(self.relative_y) > 0.025: # keep going backward if robot not aligned
                    drive_cmd.drive.speed = -1.0
                else:
                    drive_cmd.drive.speed = 0.

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()
        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = np.sqrt((self.relative_x)**2 + (self.relative_y)**2)-self.parking_distance
        #################################
        
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
