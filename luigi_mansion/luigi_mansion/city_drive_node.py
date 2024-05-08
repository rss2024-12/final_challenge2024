import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, OccupancyGrid

from .utils import LineTrajectory
from trajectory_planner_local import PathPlan
class CityDrive(Node):
    '''
    ROS2 Node in charge of taking the midline trajectory to create a path and drive commands
    '''

    def __init__(self):
        super().__init__("city_drive")
        #self.publisher = self.create_publisher(PoseArray, "/shell_points", 1)
        #self.subscriber = self.create_subscription(PointStamped, "/clicked_point", self.callback, 1)
        self.declare_parameter('drive_topic', 'default')
        self.declare_parameter('odom_topic', 'default')
        self.declare_parameter('path_topic', 'default')
        self.declare_parameter('shell_topic','default')

        # TODO: Change topics when complete
        # self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = "/drive"
        # self.DRIVE_TOPIC = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.ODOM_TOPIC = "/odom"
        # self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.PATH_TOPIC = "/path"
        # self.SHELL_TOPIC = self.get_parameter('shell_topic').get_parameter_value().string_value
        self.SHELL_TOPIC = "/shell_points"

        self.publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        #self.subscriber = self.create_subscription(Odometry, self.ODOM_TOPIC, self.odom_cb, 1)
        #self.subscriber = self.create_subscription(PoseArray, self.PATH_TOPIC, self.path_cb, 1)
        self.get_logger().info("==================== READY ====================")

        self.path = None
        self.shell_points = None

        self.get_logger().info("City Drive Node Initialized")

        #### Given midline trajectory
        self.midline_subscriber = self.create_subscription(PoseArray, "/trajectory/current" , self.midline_cb, 1)
        self.shell_sub = self.create_subscription(PoseArray, self.SHELL_TOPIC, self.shell_cb, 1)
        self.wheelbase_length = .32*1.5

        self.trajectory = LineTrajectory(self, "followed_trajectory")
        self.offset = LineTrajectory(self, "offset_trajectory")
        self.offset_publisher = self.create_publisher(PoseArray, "/offset_path", 10)
        self.planner = PathPlan(self)

        self.map_sub = self.create_subscription(OccupancyGrid, self.planner.map_topic, self.planner.map_cb,1)
        self.interp = LineTrajectory(self, "interp_trajectory")
        self.interp_publisher = self.create_publisher(PoseArray, '/interpolated_path', 10)

        self.offset_sub = self.create_subscription(PoseArray, "/offset_path", self.offset_cb, 1)
        self.offset_points = None

    #### 
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
        
        # Interpolate the trajectory
        interpolated_trajectory = self.interpolate_trajectory(msg)
        self.get_logger().info(f"Interpolated trajectory with {len(interpolated_trajectory.poses)} points")

        # Publishing the interpolated trajectory
        self.interp_publisher.publish(interpolated_trajectory)
        self.interp.fromPoseArray(interpolated_trajectory)
        self.interp.publish_viz(offset=0.0)
        self.get_logger().info("Published interp trajectory.")

        offset_points = []
        side = -1.0  # -1.0 is left, 1.0 is right

        for i in range(1, len(interpolated_trajectory.poses) - 1):
            p_prev = np.array([interpolated_trajectory.poses[i - 1].position.x, interpolated_trajectory.poses[i - 1].position.y])
            p_curr = np.array([interpolated_trajectory.poses[i].position.x, interpolated_trajectory.poses[i].position.y])
            p_next = np.array([interpolated_trajectory.poses[i + 1].position.x, interpolated_trajectory.poses[i + 1].position.y])

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
                additional_shift = v1 * -side * self.wheelbase_length
            else:
                additional_shift = np.array([0, 0])

            # Combine the perpendicular and additional shifts
            offset_point = regular_offset + additional_shift
            offset_points.append(offset_point)

        # Add the first and last points with only the regular offset
        first_point = np.array([interpolated_trajectory.poses[0].position.x, interpolated_trajectory.poses[0].position.y])
        last_point = np.array([interpolated_trajectory.poses[-1].position.x, interpolated_trajectory.poses[-1].position.y])
        first_direction = np.array([interpolated_trajectory.poses[1].position.x, interpolated_trajectory.poses[1].position.y]) - first_point
        first_direction /= np.linalg.norm(first_direction)
        first_perpendicular = np.array([-first_direction[1], first_direction[0]])
        offset_points.insert(0, first_point + side * self.wheelbase_length * first_perpendicular)

        last_direction = last_point - np.array([interpolated_trajectory.poses[-2].position.x, interpolated_trajectory.poses[-2].position.y])
        last_direction /= np.linalg.norm(last_direction)
        last_perpendicular = np.array([-last_direction[1], last_direction[0]])
        offset_points.append(last_point + side * self.wheelbase_length * last_perpendicular)

        # Creating a new PoseArray for the offset trajectory
        offset_pose_array = PoseArray()
        offset_pose_array.header = interpolated_trajectory.header
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

    def interpolate_trajectory(self, pose_array, point_spacing=1.0):
        """
        Interpolates additional points between each pair of consecutive points in the provided PoseArray based on segment length.
        
        Args:
            pose_array (PoseArray): The original PoseArray to interpolate.
            point_spacing (float): Desired approximate spacing between interpolated points in meters.
        
        Returns:
            PoseArray: A new PoseArray containing the original points and the interpolated points.
        """
        interpolated_poses = []

        # Iterate over each segment
        for i in range(len(pose_array.poses) - 1):
            start_pose = pose_array.poses[i]
            end_pose = pose_array.poses[i + 1]

            # Start and end points
            start_point = np.array([start_pose.position.x, start_pose.position.y])
            end_point = np.array([end_pose.position.x, end_pose.position.y])

            # Calculate segment length
            segment_length = np.linalg.norm(end_point - start_point)

            # Calculate the number of points to interpolate based on the segment length and point spacing
            points_per_segment = max(int(segment_length / point_spacing), 1)  # Ensure at least one point

            # Add the start pose of the segment
            interpolated_poses.append(start_pose)

            # Compute and add interpolated points
            for j in range(1, points_per_segment):
                t = j / points_per_segment
                interpolated_point = (1 - t) * start_point + t * end_point
                new_pose = Pose()
                new_pose.position.x = interpolated_point[0]
                new_pose.position.y = interpolated_point[1]
                new_pose.orientation = start_pose.orientation  # Assuming no change in orientation
                interpolated_poses.append(new_pose)

        # Add the last pose of the original trajectory
        interpolated_poses.append(pose_array.poses[-1])

        # Create a new PoseArray with interpolated poses
        new_pose_array = PoseArray()
        new_pose_array.header = pose_array.header
        new_pose_array.poses = interpolated_poses
        return new_pose_array

    def offset_cb(self,array:PoseArray):
        """
        Callback that receives the offset trajectory points.
        """
        self.offset_points = array.poses
    
    def shell_cb(self,msg):
        """
        Callback to process the shells placed by TAs and path plan them into
        the offsetted trajectory. Will directly modify offsetted trajectory object.

        Args:
            msg: (PoseArray) PoseArray object of the shell poses
        
        """
        self.shell_points = np.array(msg)
        ### insert path planning to each shell here 
        self.shell_path = LineTrajectory(self, "shell_path")
        #### iterate through each point on the trajectory, finding the closest one to goal (shell) *make sure not to pass
        ## first: extract points from /shell_points
        ## second: perform vectorized distance calculation with each one of the points to find closest point 'p' on
                ## offsetted trajectory
        if self.offset_points is not None:
            offset_traj = self.offset.toPoseArray() # full offset trajectory
            # for all 3 shells
            for point in enumerate(self.shell_points):
                # path planning
                start_pt,end_pt = self.find_intersection_points(self.offset_traj_point)
                start_PS = self.numpy_to_PoseStamped(start_pt)
                shell_PS = self.numpy_to_PoseStamped(point)
                end_PS = self.numpy_to_PoseStamped(end_pt)
                start_to_shell = self.planner.plan_path(start_PS,shell_PS)
                shell_to_end = self.planner.plan_path(shell_PS,end_PS)

                # pose arrays of trajectories
                st_to_shPS = start_to_shell.toPoseArray() # start to shell
                sh_to_ePS = shell_to_end.toPoseArray() # shell to end

                # Find the start index in offset_traj
                start_index = np.where(np.all(offset_traj == start_pt, axis=1))[0][0]
                end_index = np.where(np.all(offset_traj == end_pt, axis=1))[0][0]

                # Create a new PoseArray object for the shell path
                new_pose_array = PoseArray()
                new_pose_array.poses = []

                # Append poses from offset_traj to the start index
                new_pose_array.poses.extend(offset_traj.poses[:start_index])

                # Append poses from start_to_shell
                new_pose_array.poses.extend(start_to_shell.poses)

                # Append poses from shell_to_end
                new_pose_array.poses.extend(shell_to_end.poses)

                # Append poses from offset_traj from end index onwards
                new_pose_array.poses.extend(offset_traj.poses[end_index:])

                # Assign the new PoseArray object to shell_path
                self.shell_path.fromPoseArray(new_pose_array)
                self.shell_path.publish_viz(offset=0.0)
                self.get_logger().info("Published shell trajectory.")
                

                ## third: once closest points found, somehow call path plan on each pair of start_point 'p' and shell 's_x', x=1,2,3.
                ## will have to somehow do a local path plan to do this, maybe even better to do it on a smaller region of the map if possible
        ## fourth: add each of these paths to the offsetted trajectory 
            #### store each iterated and new-created trajectory to a new final trajectory (possibly a merge function)


    ### HELPER FUNCTIONS ###
    # def pose_array_to_numpy(self,array:PoseArray):
    #     """
    #     Helper function that takes in a PoseArray and converts it into
    #     a np.array of x,y coordinates.
    #     """
    #     points = np.zeros(len(array))
    #     for idx,pose in enumerate(array.poses):
    #         points[idx] = np.array([pose.position.x,pose.position.y])
    #     return points
    def find_intersection_points(self,trajectory_points:np.ndarray,circle_center:np.ndarray,radius:float):
        """
        Function that will find the intersection points between a circle of size radius
        around the shell.
        """
        distances = np.sqrt((trajectory_points[:,0]-circle_center[0])**2 + (trajectory_points[:,1]-circle_center[1])**2)
        intersection_mask = distances <= radius
        intersection_pts = trajectory_points[intersection_mask]
        start_pt = intersection_pts[0]
        end_pt = intersection_pts[-1]

        return (start_pt,end_pt)

    def numpy_to_PoseStamped(self,array:np.ndarray):
        """
        Helper function that turns an np.ndarray into a PoseStamped 
        object.
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = array[0]
        pose.pose.position.y = array[1]
        pose.pose.position.z = 0
        return pose
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
