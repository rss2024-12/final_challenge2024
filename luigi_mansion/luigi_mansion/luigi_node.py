import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray


class CityDrive(Node):
    '''
    stuff
    '''

    def __init__(self):
        super().__init__("city_drive")
        self.publisher = self.create_publisher(PoseArray, "/shell_points", 1)
        self.subscriber = self.create_subscription(PointStamped, "/clicked_point", self.callback, 1)

        self.array = []

        self.get_logger().info("Point Publisher Initialized")



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