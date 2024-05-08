import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from .utils import LineTrajectory

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

        offset_points = []
        side = 1.0  # -1.0 is left, 1.0 is right

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
                additional_shift = v1 * -side * self.wheelbase_length
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

    def shell_cb(self,msg):
        """
        Callback to process the shells placed by TAs and path plan them into
        the offsetted trajectory. Will directly modify offsetted trajectory object.

        Args:
            msg: (PoseArray) PoseArray object of the shell poses
        
        """
        self.shell_points = self.pose_array_to_numpy(msg.poses)
        ### insert path planning to each shell here 

        #### iterate through each point on the trajectory, finding the closest one to goal (shell) *make sure not to pass
        ## first: extract points from /shell_points
        ## second: perform vectorized distance calculation with each one of the points to find closest point 'p' on
                ## offsetted trajectory
        for point in self.shell_points:
            x_s = point[0]*np.ones(len(self.offset_traj_points))
            y_s = point[1]*np.ones(len(self.offset_traj_points))
            min_idx_to_shell = np.argmin((np.sqrt((self.offset_traj_points[:,0]-x_s)**2 + (self.offset_traj_points[:,1]-y_s)**2)))
            min_pt = self.offset_traj_points[min_idx_to_shell]
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
        points = np.zeros(len(array))
        for idx,pose in enumerate(array.poses):
            points[idx] = np.array([pose.position.x,pose.position.y])
        return points
    


    # path planning functions from lab 6 to find optimal path to each shell from 
    # closet point in offset trajectory
    def plan_path(self, start_point, end_point, map):
        """
        Plan a path from the start point to the end point on the given map graph.

        Parameters:
        start_point (PoseStamped): The start point in the map frame.
        end_point (PoseStamped): The end point in the map frame.
        map (OccupancyGrid): An instance of the nav_msgs/OccupancyGrid message,
                              representing the map graph on which the path is to be planned.

        Returns:
        list: A list representing the planned path from the start point to the end point.
              Each element in the list is a tuple containing the (x, y) coordinates of a point along the path.
              Example: [(x1, y1), (x2, y2), ..., (xn, yn)]

        Notes:
        - The path planning process involves the following steps:
          1. Sample random points in grid space.
          2. Keep points that are viable.
          3. Connect each viable point to all other points, creating edges.
          4. Remove edges that intersect obstacles.
          5. Run the A* search algorithm on the viable trajectories.
        """
        start_node = (start_point.pose.position.x, start_point.pose.position.y)
        end_node = (end_point.position.x, end_point.position.y)

         # Find nearest nodes to start and end in the graph
        _, start_index = self.kd_tree.query(start_node)
        _, end_index = self.kd_tree.query(end_node)

        # Run A* on the graph
        path = self.astar(self.graph, start_index, end_index) # outputs a list of indices for what node the path has

        # Turn indices into their coordinates
        path_coord = self.path_index_coord(path)

        path_array = self.path_index_posearray(path)
        # self.vis_path.publish(path_array)

        self.trajectory.points = path_coord
        # self.get_logger().info('Path %s' % path_coord)
        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()

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
