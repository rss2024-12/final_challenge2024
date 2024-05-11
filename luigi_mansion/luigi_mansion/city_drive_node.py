import numpy as np
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, OccupancyGrid

from .utils import LineTrajectory
from .trajectory_planner_local import PathPlan

class CityDrive(Node):
    '''
    ROS2 Node in charge of taking the midline trajectory to create a path and drive commands
    '''

    def __init__(self):
        super().__init__("city_drive")
        #self.publisher = self.create_publisher(PoseArray, "/shell_points", 1)
        #self.subscriber = self.create_subscription(PointStamped, "/clicked_point", self.callback, 1)
        # self.declare_parameter('drive_topic', 'default')
        # self.declare_parameter('odom_topic', 'default')
        # self.declare_parameter('path_topic', 'default')
        # self.declare_parameter('shell_topic','default')

        # TODO: Change topics when complete
        # self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        # self.DRIVE_TOPIC = "/drive"
        self.DRIVE_TOPIC = "/vesc/high_level/input/nav_1"
        # self.ODOM_TOPIC = self.get_parameter('odom_topic').get_parameter_value().string_value
        # self.ODOM_TOPIC = "/odom"
        self.ODOM_TOPIC = "/vesc/odom"
        # self.PATH_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.PATH_TOPIC = "/path"
        # self.SHELL_TOPIC = self.get_parameter('shell_topic').get_parameter_value().string_value
        self.SHELL_TOPIC = "/shell_points"

        self.publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        #self.subscriber = self.create_subscription(Odometry, self.ODOM_TOPIC, self.odom_cb, 1)
        #self.subscriber = self.create_subscription(PoseArray, self.PATH_TOPIC, self.path_cb, 1)
        self.get_logger().info("==================== READY ====================")

        self.path = None
        self.shell_points = None
        self.map = None

        self.get_logger().info("City Drive Node Initialized")

        #### Given midline trajectory
        self.midline_subscriber = self.create_subscription(PoseArray, "/trajectory/current" , self.midline_cb, 1)
        self.get_logger().info("Created midline sub")
        self.shell_sub = self.create_subscription(PoseArray, self.SHELL_TOPIC, self.shell_cb, 1)
        self.get_logger().info("Created shell sub")
        self.wheelbase_length = .32*1.5

        self.trajectory = LineTrajectory(self, "followed_trajectory")
        self.get_logger().info("Created trajectory traj class instance")
        self.offset = LineTrajectory(self, "offset_trajectory")
        self.get_logger().info("Created offset traj class intsance")
        self.offset_publisher = self.create_publisher(PoseArray, "/offset_path", 10)
        self.get_logger().info("Created offset publisher")
        self.planner = PathPlan(self)
        self.get_logger().info("Created planner class instance")

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.planner.map_cb,1)
        self.get_logger().info("Created map sub")
        self.interp = LineTrajectory(self, "interp_trajectory")
        self.get_logger().info("Created interp traj class instance")
        self.interp_publisher = self.create_publisher(PoseArray, '/interpolated_path', 10)
        

        self.offset_sub = self.create_subscription(PoseArray, "/offset_path", self.offset_cb, 1)
        self.get_logger().info("Created offset subscriber")
        self.offset_points = None
        self.get_logger().info("Created offset points object None")

        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.get_logger().info("Created point publisher")

        self.right_offset_publisher = self.create_publisher(PoseArray, '/right_offset_path', 10)
        self.right_offset = LineTrajectory(self, "right_offset_trajectory")
        self.merged_trajectory_publisher = self.create_publisher(PoseArray, '/merged_path', 10)
        self.merged_offset = LineTrajectory(self, "merge_offset_trajectory")


    def midline_cb(self, msg):
        """
        msg: PoseArray object representing points along a midline piecewise line segment.
        This function is called once when the midline trajectory is loaded.
        It publishes new PoseArray objects for both left and right offsets based on the midline trajectory.
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
        self.get_logger().info("Published interpolated trajectory.")

        left_offset_points = []
        right_offset_points = []
        side_offsets = {'left': -1.0, 'right': 1.0}  # -1.0 for left, 1.0 for right

        for direction, side in side_offsets.items():
            offset_points = []
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
            if direction == 'left':
                lpa = offset_pose_array
                self.offset_publisher.publish(offset_pose_array)
                self.offset.fromPoseArray(offset_pose_array)
                self.offset.publish_viz(offset=0.0)
                self.get_logger().info("Published left offset trajectory.")
            else:
                rpa = offset_pose_array
                self.right_offset_publisher.publish(offset_pose_array)
                self.right_offset.fromPoseArray(offset_pose_array)
                self.right_offset.publish_viz(offset=0.0)
                self.get_logger().info("Published right offset trajectory.")
        self.publish_merged_trajectory(self.merge_trajectories(lpa, rpa))

    def merge_trajectories(self, left_trajectory, right_trajectory):
        """
        Merges the left and right trajectories into a single trajectory. The left trajectory is followed by
        the reversed right trajectory.

        Args:
            left_trajectory (PoseArray): The PoseArray containing the left offset trajectory.
            right_trajectory (PoseArray): The PoseArray containing the right offset trajectory.

        Returns:
            PoseArray: The merged trajectory.
        """
        # Create a new PoseArray for the merged trajectory
        merged_trajectory = PoseArray()
        merged_trajectory.header = left_trajectory.header  # Assuming both trajectories have the same header

        # Add all poses from the left trajectory
        merged_trajectory.poses.extend(left_trajectory.poses)

        # Add poses from the right trajectory in reverse order
        reversed_right_poses = list(reversed(right_trajectory.poses))
        merged_trajectory.poses.extend(reversed_right_poses)

        return merged_trajectory

    def publish_merged_trajectory(self, merged_trajectory):
        """
        Publishes the merged trajectory to a topic.

        Args:
            merged_trajectory (PoseArray): The merged trajectory PoseArray to be published.
        """
        # Assuming you have a publisher set up for merged trajectories
        self.merged_trajectory_publisher.publish(merged_trajectory)
        self.merged_offset.fromPoseArray(merged_trajectory)
        self.merged_offset.publish_viz(offset=0.5)
        self.get_logger().info("Published merged trajectory.")


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
        if array.poses:
            self.offset_points = array.poses
            self.get_logger().info(f"Offset points received: {len(self.offset_points)} points")
        else:
            self.get_logger().warn("Received empty offset trajectory")
    
    def create_marker(self, point, marker_id, color=(1.0, 0.0, 0.0)):
        """Create a marker for visualization in RViz."""
        marker = Marker()
        marker.header.frame_id = "map"  # Set the relevant coordinate frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "shell_path"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.0  # Adjust as needed for the z-axis
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Marker size on the x-axis
        marker.scale.y = 0.2  # Marker size on the y-axis
        marker.scale.z = 0.2  # Marker size on the z-axis
        marker.color.a = 1.0  # Alpha (opacity)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        return marker

    def shell_cb(self,msg):
        """
        Callback to process the shells placed by TAs and path plan them into
        the offsetted trajectory. Will directly modify offsetted trajectory object.

        Args:
            msg: (PoseArray) PoseArray object of the shell poses
        
        """
        self.get_logger().info("Shell callback initiated.")
        #self.get_logger().info(f"{msg=}")
        self.shell_points = np.array(msg.poses)
        # self.get_logger().info(f"{self.shell_points=}")
        ### insert path planning to each shell here 
        self.shell_path = LineTrajectory(self, "shell_path")
        #### iterate through each point on the trajectory, finding the closest one to goal (shell) *make sure not to pass
        ## first: extract points from /shell_points
        ## second: perform vectorized distance calculation with each one of the points to find closest point 'p' on
                ## offsetted trajectory
        if self.offset_points is not None:
            self.get_logger().info("Recieved offset trajectory.")
            offset_traj = self.merged_offset.toPoseArray() # full offset trajectory

            # for all 3 shells
            # Create a new PoseArray object for the shell path
            final_pose_array = PoseArray()
            final_pose_array.header = offset_traj.header
            final_pose_array.poses = offset_traj.poses[:]

            for index, point in enumerate(self.shell_points):
                self.get_logger().info(f"Shell {index+1}")

                # obtain points of intersection
                start_pt,end_pt = self.find_intersection_points(offset_traj, point, 4.)

                # # Publish start point marker
                # start_marker = self.create_marker(start_pt, marker_id=1, color=(0.0, 1.0, 1.0))
                # self.marker_pub.publish(start_marker)
                # self.get_logger().info("Published marker for start point.")

                #self.get_logger().info(f"{start_pt=}, {end_pt=}")
                start_PS = self.numpy_to_PoseStamped(start_pt)
                #self.get_logger().info(f"{start_PS=}")
                shell_PS = self.numpy_to_PoseStamped(np.array([point.position.x,point.position.y]))
                #self.get_logger().info(f"{shell_PS=}")
                end_PS = self.numpy_to_PoseStamped(end_pt)
                #self.get_logger().info(f"{end_PS=}")

                # plan paths
                # self.get_logger().info(f"{self.map=}")
                start_to_shell = self.planner.plan_path(start_PS, shell_PS, self.map).toPoseArray()
                self.get_logger().info("STS path plan success")
                shell_to_end = self.planner.plan_path(shell_PS, end_PS, self.map).toPoseArray()
                self.get_logger().info("STE path plan success")

                # Find the start index in offset_traj
                numpy_final_poses = self.pose_array_to_numpy(final_pose_array)
                # self.get_logger().info(f"{numpy_final_poses=}")
                start_index = np.where(np.all(numpy_final_poses == start_pt, axis=1))[0][0]
                end_index = np.where(np.all(numpy_final_poses == end_pt, axis=1))[0][0]

                # Create a new temporary PoseArray to merge changes
                temp_pose_array = PoseArray()
                temp_pose_array.poses = []

                # Append poses from offset_traj to the start index
                temp_pose_array.poses.extend(final_pose_array.poses[:start_index])

                # Append poses from start_to_shell
                temp_pose_array.poses.extend(start_to_shell.poses)

                # Append poses from shell_to_end
                temp_pose_array.poses.extend(shell_to_end.poses)

                # Append poses from offset_traj from end index onwards
                temp_pose_array.poses.extend(final_pose_array.poses[end_index+1:])

                self.get_logger().info(f"Shell {index+1} Trajectory calculated.")

                final_pose_array.poses = temp_pose_array.poses


            # Assign the new PoseArray object to shell_path
            self.shell_path.fromPoseArray(final_pose_array)
            self.shell_path.publish_viz(offset=0.0)
            self.get_logger().info("Published combined shell trajectory.")
                

                ## third: once closest points found, somehow call path plan on each pair of start_point 'p' and shell 's_x', x=1,2,3.
                ## will have to somehow do a local path plan to do this, maybe even better to do it on a smaller region of the map if possible
        ## fourth: add each of these paths to the offsetted trajectory 
            #### store each iterated and new-created trajectory to a new final trajectory (possibly a merge function)


    ### HELPER FUNCTIONS ###
    def pose_array_to_numpy(self,array:PoseArray):
        """
        Helper function that takes in a PoseArray and converts it into
        a np.array of x,y coordinates.
        """
        points = np.zeros((len(array.poses), 2))
        for idx, pose in enumerate(array.poses):
            # Assign x and y coordinates directly to the row
            points[idx] = [pose.position.x, pose.position.y]
        return points
        

    def find_intersection_points(self, trajectory_points, circle_center, radius: float):
        """
        Function that will find the intersection points between a circle of size radius
        around the shell and determine the closest segment to the center.
        """
        traj_points_np = self.pose_array_to_numpy(trajectory_points)
        circle_center_np = np.array([circle_center.position.x, circle_center.position.y])

        # Calculate distances from all trajectory points to the circle center
        distances = np.sqrt((traj_points_np[:, 0] - circle_center_np[0]) ** 2 +
                            (traj_points_np[:, 1] - circle_center_np[1]) ** 2)
        intersection_mask = (distances <= radius) & (distances >= radius - 1.5)
        intersection_pts = traj_points_np[intersection_mask]

        # Ensure there are at least two points to form a segment
        if len(intersection_pts) < 2:
            self.get_logger().info("Not enough intersection points to form segments.")
            return None, None

        min_distance = float('inf')
        start_pt_index = None
        end_pt_index = None

        # Iterate through each pair of consecutive points
        for i in range(len(intersection_pts) - 1):
            point_a = intersection_pts[i]
            point_b = intersection_pts[i + 1]
            segment_vector = point_b - point_a
            vector_to_center = circle_center_np - point_a

            # Project vector_to_center onto segment_vector
            segment_length_squared = np.dot(segment_vector, segment_vector)
            if segment_length_squared == 0:
                continue  # Avoid division by zero if points are coincident
            projection_length = np.dot(vector_to_center, segment_vector) / segment_length_squared
            projection_vector = projection_length * segment_vector
            projection_point = point_a + projection_vector

            # Check if the projection point is within the segment
            if 0 <= projection_length <= 1:
                perpendicular_distance = np.linalg.norm(vector_to_center - projection_vector)
                if perpendicular_distance < min_distance:
                    min_distance = perpendicular_distance
                    start_pt_index = i
                    end_pt_index = i + 1

        # Check if a valid segment was found
        if start_pt_index is not None and end_pt_index is not None:
            start_pt = intersection_pts[start_pt_index]
            end_pt = intersection_pts[end_pt_index]
            return start_pt, end_pt
        else:
            self.get_logger().info("No suitable segment found.")
            return None, None


    def numpy_to_PoseStamped(self,array:np.ndarray):
        """
        Helper function that turns an np.ndarray into a PoseStamped 
        object.
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = array[0]
        pose.pose.position.y = array[1]
        pose.pose.position.z = 0.
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
