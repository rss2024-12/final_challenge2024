import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from stop_detector.detector import StopSignDetector
from stop_detector.stop_detector import SignDetector # Not sure if this is correct


class StopDrive(Node):
    '''
    ROS2 node to stop other nodes due to stop light and stop signs.

    Req:

    Returns: 
    '''

    def __init__(self):
        super().__init__("stop_drive")
        self.declare_parameter('drive_topic', 'default')
        self.declare_parameter('odom_topic', 'default')
        self.declare_parameter('path_topic', 'default')

        # TODO: Change topics when complete
        # self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = "/drive"
        # self.DRIVE_TOPIC = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.ODOM_TOPIC = "/odom"
        # self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.PATH_TOPIC = "/path"

        self.publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.subscriber = self.create_subscription(Odometry, self.ODOM_TOPIC, self.odom_cb, 1)
        self.subscriber = self.create_subscription(PoseArray, self.PATH_TOPIC, self.path_cb, 1)
        self.get_logger().info("==================== READY ====================")

        self.path = None

        self.get_logger().info("Point Publisher Initialized")

    def path_cb(self, msg:PoseArray):
        pass



    #### Given midline trajectory and stop sign detector 

    #### Create offset midline, offset by wheelbase_length (robot will follow this)

    #### TAs pick three points

    #### iterate through each point on the trajectory, finding the closest one to goal (shell) *make sure not to pass

    #### run path planner from this point to goal point, and inverse to get goal to point

    #### store each iterated and new-created trajectory to a new final trajectory (possibly a merge function)
    
    #### follow this new trajectory & ensure it never passes midline (perhaps increase num samples for path planner)

    #### implement stop sign and pedestrian detectors for safety

    ####### NOTE ########
    #### will have to fine tu

def main(args=None):
    rclpy.init(args=args)
    node = StopDrive()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
