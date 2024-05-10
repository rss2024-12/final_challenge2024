#!/usr/bin/env python
#this is pulled straight from Lab 4, name changed to driving_controller
#changes will be needed

import rclpy
from rclpy.node import Node
import numpy as np

from cs_msgs.msg import LineLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from builtin_interfaces.msg import Time

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
            
        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
       
    
        
        self.create_subscription(LineLocation, "/relative_line", 
            self.relative_line_callback, 1)

        self.speed = 4.0  
        self.relative_x = 0
        self.relative_y = 0
        self.previous_delta = None
        self.angle_step = 1/100
        self.max_angle = 0.05
        self.correction_factor = 0.019
        self.get_logger().info("Driving Controller Initialized")

    def relative_line_callback(self, msg):
        lane_width = 34 * 0.0254 #gives Johnson track lane width in m
        half_width = lane_width/2

        self.relative_x =  msg.x_pos #should be adjusted for speed. Fixed offset accouts for camera to vehicle center
        self.relative_y = msg.y_pos  #potential error, I think x is forward
        #self.get_logger().info(f'x distance, y distance: {self.relative_x, self.relative_y}')
        drive_cmd = AckermannDriveStamped()
        
        L = .32 # distance between front and back wheels im m
        
        L1 = np.sqrt((self.relative_x)**2 + (self.relative_y)**2) 
        eta = np.arctan(self.relative_y/self.relative_x)
        delta = 1*np.arctan((2*L*np.sin(eta))/L1) 

        
        if self.previous_delta is not None:
            
            if delta>self.previous_delta+self.angle_step:
                # self.get_logger().info('Angle step!')
                delta = self.previous_delta+self.angle_step
            elif delta < self.previous_delta -self.angle_step:
                # self.get_logger().info('Angle step!')
                delta = self.previous_delta-self.angle_step
       
        if delta < -self.max_angle:
            # self.get_logger().info('Max angle!')
            delta = -self.max_angle
        elif delta > self.max_angle:
            # self.get_logger().info('Max angle!')
            delta = self.max_angle

        drive_cmd.drive.steering_angle = delta - self.correction_factor #correction factor
        self.previous_delta = delta
        drive_cmd.drive.speed = self.speed
    

    
        # self.get_logger().info(f'Delta, speed {delta,self.speed}')
        self.drive_pub.publish(drive_cmd)
        

def main(args=None):
    rclpy.init(args=args)
    pc = DrivingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



