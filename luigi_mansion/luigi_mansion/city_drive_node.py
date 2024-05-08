import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from .utils import LineTrajectory

class CityDrive(Node):
    '''
    ROS2 Node in charge of taking the trajectories to create a path and drive commands
    '''

    def __init__(self):
        super().__init__("city_drive")
        #self.publisher = self.create_publisher(PoseArray, "/shell_points", 1)
        #self.subscriber = self.create_subscription(PointStamped, "/clicked_point", self.callback, 1)
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
        #self.subscriber = self.create_subscription(Odometry, self.ODOM_TOPIC, self.odom_cb, 1)
        #self.subscriber = self.create_subscription(PoseArray, self.PATH_TOPIC, self.path_cb, 1)
        self.get_logger().info("==================== READY ====================")

        self.path = None

        self.get_logger().info("Point Publisher Initialized")

        #### Given midline trajectory
        self.midline_subscriber = self.create_subscription(PoseArray, "/trajectory/current" , self.midline_cb, 1)
        self.wheelbase_length = .32*1.5

        self.trajectory = LineTrajectory(self, "followed_trajectory")
        self.offset = LineTrajectory(self, "offset_trajectory")
        self.offset_publisher = self.create_publisher(PoseArray, "/offset_path", 10)

    def midline_cb(self, msg):
        """
        msg: PoseArray object representing points along a midline piecewise line segment.
        This function is called once when the midline trajectory is loaded.
        It publishes a new PoseArray object, "offset_path," which represents the midline trajectory offset by the wheelbase length.
        """
        self.get_logger().info(f"Receiving new trajectory with {len(msg.poses)} points")

        if len(msg.poses) < 3:  # Requires at least three points to calculate angles
            self.get_logger().error("Received trajectory is too short to compute offset.")
            return

        offset_points = []
        side = -1.0  # -1.0 is left, 1.0 is right

        for i in range(1, len(msg.poses) - 1):
            p_prev = np.array([msg.poses[i - 1].position.x, msg.poses[i - 1].position.y])
            p_curr = np.array([msg.poses[i].position.x, msg.poses[i].position.y])
            p_next = np.array([msg.poses[i + 1].position.x, msg.poses[i + 1].position.y])

            # Calculate and normalize direction vectors
            v1 = p_curr - p_prev
            v2 = p_next - p_curr
            v1 /= np.linalg.norm(v1)
            v2 /= np.linalg.norm(v2)

            # Calculate perpendicular vector
            perpendicular_vector = np.array([-v1[1], v1[0]])
            regular_offset = p_curr + side * self.wheelbase_length * perpendicular_vector

            # Calculate dot product to find the angle between segments
            dot_product = np.dot(v1, v2)
            angle_cos = dot_product  # Since vectors are normalized, this gives cos(theta)

            # Check if the angle is close to 90 degrees
            if np.abs(angle_cos) < 0.3:
                # Apply additional shift in the direction of the segment
                additional_shift = v1 * self.wheelbase_length
            else:
                additional_shift = np.array([0, 0])

            # Combine the perpendicular and additional shifts
            offset_point = regular_offset + additional_shift
            offset_points.append(offset_point)

        # Add the first and last points with only the regular offset
        first_point = np.array([msg.poses[0].position.x, msg.poses[0].position.y])
        last_point = np.array([msg.poses[-1].position.x, msg.poses[-1].position.y])
        first_direction = np.array([msg.poses[1].position.x, msg.poses[1].position.y]) - first_point
        first_direction /= np.linalg.norm(first_direction)
        first_perpendicular = np.array([-first_direction[1], first_direction[0]])
        offset_points.insert(0, first_point + side * self.wheelbase_length * first_perpendicular)

        last_direction = last_point - np.array([msg.poses[-2].position.x, msg.poses[-2].position.y])
        last_direction /= np.linalg.norm(last_direction)
        last_perpendicular = np.array([-last_direction[1], last_direction[0]])
        offset_points.append(last_point + side * self.wheelbase_length * last_perpendicular)

        # Creating a new PoseArray for the offset trajectory
        offset_pose_array = PoseArray()
        offset_pose_array.header = msg.header
        for point in offset_points:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.orientation.w = 1.0  # No rotation applied
            offset_pose_array.poses.append(pose)

        # Publishing the offset trajectory
        self.offset_publisher.publish(offset_pose_array)
        self.offset.fromPoseArray(offset_pose_array)
        self.offset.publish_viz(offset=0.0)
        self.get_logger().info("Published offset trajectory.")



    #### TAs pick three points

    #### iterate through each point on the trajectory, finding the closest one to goal (shell) *make sure not to pass

    #### run path planner from this point to goal point, and inverse to get goal to point

    #### store each iterated and new-created trajectory to a new final trajectory (possibly a merge function)
    
    #### follow this new trajectory & ensure it never passes midline (perhaps increase num samples for path planner)

    #### implement stop sign and pedestrian detectors for safety


def main(args=None):
    rclpy.init(args=args)
    node = CityDrive()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
