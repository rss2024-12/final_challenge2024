import rclpy
from rclpy.node import Node

assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
from nav_msgs.msg import OccupancyGrid, Path
from .utils import LineTrajectory

from nav_msgs.msg import Odometry


### Utilized in Path planning
import random
import numpy as np
import math
from scipy.spatial import KDTree
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
import heapq


import cv2
import struct
from cv_bridge import CvBridge

from geometry_msgs.msg import Quaternion
import tf_transformations
from tf_transformations import euler_from_quaternion
###


class PathPlan():
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self,node):
        #super().__init__("trajectory_planner")
        # node.declare_parameter('odom_topic', "default")
        node.declare_parameter('map_topic', "default")
        node.declare_parameter('initial_pose_topic', "default")

        # self.odom_topic = node.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = node.get_parameter('map_topic').get_parameter_value().string_value
        self.initial_pose_topic = node.get_parameter('initial_pose_topic').get_parameter_value().string_value
        self.node = node

        # SUBSCRIPTIONS
        # Gets called when the map is received
        self.map_sub = node.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            1)

        # Gets called when the goal is received
        # self.goal_sub = node.create_subscription(
        #     PoseStamped,
        #     "/goal_pose",
        #     self.goal_cb,
        #     10
        # )

        # Sets the current pose to the particle filter pose
        # self.pf_sub = node.create_subscription(
        #     Odometry,
        #     "/pf/pose/odom",
        #     self.pf_cb,
        #     10
        # )

        # Needed for path visualization
        # self.start_sub = node.create_subscription(
        #     Marker,
        #     '/planned_trajectory/start_point',
        #     self.start_cb,
        #     10
        # )

        # Gets called when the initial pose is received from Rviz
        # self.pose_sub = node.create_subscription(
        #     PoseWithCovarianceStamped,
        #     self.initial_pose_topic,
        #     self.pose_cb,
        #     10
        # )

        ## PUBLISHERS
        # self.traj_pub = node.create_publisher(
        #     PoseArray,
        #     "/trajectory/current",
        #     10
        # )

        # Used to visualize the nodes of the PRM graph
        self.graph_vis_pub = self.node.create_publisher(
            PointCloud2,
            "/graph_points",
            10
        )

        # Publisher for testing pixel_to_map_coordinate method
        # self.test_pub = node.create_publisher(
        #     PoseStamped,
        #     "/test_pose",
        #     10
        # )

        # self.test_pub_1 = node.create_publisher(
        #     PoseStamped,
        #     "/test_pose_1",
        #     10
        # )

        # self.vis_path = node.create_publisher(
        #     PoseArray,
        #     "/vis_path",
        #     10
        # )
        
        self.trajectory = LineTrajectory(node=node, viz_namespace="/planned_trajectory")


       
        ### Create a subscriber to Odom Topic ###unsure: for testing what should I subscribe to 
       
        ###Questions###
        #Do I need to make an odom subscriber? nowhere is the pf/topic called in
        #if so, which callback should it be in/do i need to make a new one?
        #do i just swtich out the pose_sub topic?


        ### Important variables for PRM
        self.obstacle_threshold = 50  # all map values are 0 or 100
        self.num_samples = 20000  # Adjust as needed
        self.connect_radius = 5.0  # gives distance maximum distance of a "neighbor"
        self.graph = []

    ### CALLBACKS START ###
    def map_cb(self, msg):
        self.node.get_logger().info("Map callback initiated.")
        #of the form of occupancy grid. Should just store the map I think
        ###pre process the map, map is only received once
        ## need to dilate and erode (absolutely necessary), see README
        radius = 10  # 10 px radius
        occupancy_data = np.array(msg.data, dtype=np.uint8).reshape((msg.info.height, msg.info.width))
        self.node.get_logger().info(f"{msg.info.height=}, {msg.info.width=}")
        # occupancy_image = cv2.flip(occupancy_data, 0)  # Flip the image vertically to match the ROS convention

        # Crop the first crop pixels of the height
        crop_y = 350
        if msg.info.height > crop_y:
            # Crop the first crop_y rows and first crop_x columns
            cropped_occupancy_data = occupancy_data[crop_y:, :]

            # Update the map info height and width
            new_height = msg.info.height - crop_y
            msg.info.height = new_height

            # Convert the cropped data back to a list and ensure it is in the correct int8 format
            cropped_data_flattened = cropped_occupancy_data.astype(np.int8).flatten().tolist()
            msg.data = cropped_data_flattened
        else:
            self.node.get_logger().warning("Crop dimensions invalid.")


        occupancy_image = ((occupancy_data / 255.0) * 100).astype('uint8')
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
        dilated_image = cv2.dilate(occupancy_image, kernel)

         # Convert dilated image back to OccupancyGrid message
        dilated_occupancy_grid_msg = msg
        dilated_occupancy_grid_msg.data = dilated_image.flatten().tolist()

        self.node.map = dilated_occupancy_grid_msg

        self.node.get_logger().info('Generating')
        self.generate_prm()
        self.node.get_logger().info('Generated')
        self.visualize_prm(self.valid_points)
        self.node.get_logger().info('Visualized')
        #raise NotImplementedError

    # def pose_cb(self, pose):
    #     #type is PoseWithCovarianceStamped

    #     self.current_pose = pose.pose
    #     self.node.get_logger().info('Current X %s' % self.current_pose.pose.position.x)
    #     self.node.get_logger().info('Current Y %s' % self.current_pose.pose.position.y)
        
        #this is where you pass through the particle filter pose
        # raise NotImplementedError

    # def pf_cb(self, pose):
    #     # type of Odometry message
    #     self.current_pose = pose.pose


    # def goal_cb(self, msg):
    #     #type is PoseStamped
    #     self.goal_pose = msg.pose
    #     self.node.get_logger().info('Goal X %s' % self.goal_pose.position.x)
    #     self.node.get_logger().info('Goal Y %s' % self.goal_pose.position.y)
    #     #should have a call to pathfinding here
        
    #     self.plan_path(self.current_pose, self.goal_pose, self.map)
        #raise NotImplementedError

    def start_cb(self, msg):
        pass
    ### CALLBACKS END ###
    
    
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
        end_node = (end_point.pose.position.x, end_point.pose.position.y)


         # Find nearest nodes to start and end in the graph
        _, start_index = self.node.kd_tree.query(start_node)
        _, end_index = self.node.kd_tree.query(end_node)

        # Run A* on the graph
        path = self.astar(self.graph, start_index, end_index) # outputs a list of indices for what node the path has

        # Turn indices into their coordinates
        path_coord = self.path_index_coord(path)

        path_array = self.path_index_posearray(path)
        # self.vis_path.publish(path_array)

        self.trajectory.points = path_coord
        # self.get_logger().info('Path %s' % path_coord)
        path_to_shell = self.trajectory
        return path_to_shell
        # self.traj_pub.publish(self.trajectory.toPoseArray())
        # self.trajectory.publish_viz()


    def generate_prm(self):
        """
        Generates a Probabilistic Roadmap (PRM) graph for path planning.

        Returns:
            list: A list representing the generated PRM graph.
              Each element in the list is a tuple representing a node in the graph.
              The tuple format is (x, y, neighbors), where:
              - x (float): X-coordinate of the node in the map frame.
              - y (float): Y-coordinate of the node in the map frame.
              - neighbors (list of int): Indices of neighboring nodes in the graph.


        """
        # Initialize graph
        self.graph = []

        # Step 1: Sample random points in grid space and check validity
        self.valid_points = []
        random.seed(42)
        num_samples = 0
        while num_samples < self.num_samples:
            # Sample a random point
            rand_x = random.uniform(0, 1)
            scale_rand_x = rand_x * self.node.map.info.width
            pixel_x = math.floor(scale_rand_x)

            rand_y = random.uniform(0, 1)
            scale_rand_y = rand_y * self.node.map.info.height
            pixel_y = math.floor(scale_rand_y)
            
            random_x, random_y = self.pixels_to_map_coordinates(pixel_x, pixel_y)

            # Check validity of the sampled point
            if self.node.map.data[pixel_y * self.node.map.info.width + pixel_x] < self.obstacle_threshold: #error probable here
                self.valid_points.append((random_x, random_y))
            num_samples += 1
       # Step 2: Connect each valid point to all other valid points within a certain radius
        for i, point1 in enumerate(self.valid_points):
            neighbors = []
            for j, point2 in enumerate(self.valid_points):
                if i != j:
                    distance = np.linalg.norm(np.array(point1) - np.array(point2))
                    if distance < self.connect_radius:
                        # Check for obstacle intersection
                        edge_valid = True
                        for t in np.linspace(0, 1, num=10):  # Divide the line segment into 10 intervals
                            x = (1 - t) * point1[0] + t * point2[0]
                            y = (1 - t) * point1[1] + t * point2[1]
                            pixel_x, pixel_y = self.map_coordinates_to_pixels(x, y)

                            if self.node.map.data[pixel_y * self.node.map.info.width + pixel_x] >= self.obstacle_threshold:
                                edge_valid = False
                                break
                        if edge_valid:
                            neighbors.append(j)  # Add index of neighbor to neighbor list
            self.graph.append((point1[0], point1[1], neighbors))

        # Build KDTree for nearest neighbor lookup
        # self.node.get_logger().info(f"{self.valid_points=}")
        self.node.kd_tree = KDTree([(point[0], point[1]) for point in self.valid_points])
        

    def visualize_prm(self, points):
        ''' 
        Used to visualized the nodes and edges that are created by 'generate_prm'.
        '''
        msg = PointCloud2()

        # Set the header
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()

        # Set the fields (x, y, z coordinates)
        msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))

        # Set the point step (size of a single point in bytes)
        msg.point_step = 12  # 3 float32 values for x, y, z

        # Set the data layout
        msg.row_step = msg.point_step * len(points)
        points_z = [(point[0], point[1], 0.0) for point in points]
        #msg.data = struct.pack('<' + 'fff' * len(points), *(point + (0.0,) for point in points))  # Adding 0.0 for z-coordinate
        
        msg.data = struct.pack('<' + 'fff' * len(points_z), *((coord for point in points_z for coord in point)))

        # Set other metadata
        msg.height = 1
        msg.width = len(points_z)
        msg.is_bigendian = False
        msg.is_dense = True

        self.graph_vis_pub.publish(msg)

    def path_index_posearray(self, path):
        """ Takes the list of indices returned by A-star and turns it into a list of coordinates.
        """
        path_coord = []
        for node in path:
            tup = self.graph[node]
            pose = Pose()
            pose.position.x = tup[0]
            pose.position.y = tup[1]
            pose.position.z = 0.
            pose.orientation.x = 0.
            pose.orientation.y = 0.
            pose.orientation.z = 0.
            pose.orientation.w = 0.
            path_coord.append(pose)

        path_array = PoseArray()
        path_array.header = Header()
        path_array.header.stamp = self.node.get_clock().now().to_msg()
        path_array.header.frame_id = 'map'
        path_array.poses = path_coord
        
        return path_array

    def path_index_coord(self, path):
        """ Takes the list of indices returned by A-star and turns it into a list of coordinates.
        """
        path_coord = []
        for node in path:
            path_coord.append(self.graph[node][:2])
        return path_coord

        
    def find_neighbors(self, point, points):
        """
        Find neighboring points within a certain radius of a given point.

        Args:
            point (tuple): The coordinates of the reference point (x, y).
            points (list): A list of points to search for neighbors.

        Returns:
            list: A list of neighboring points within the specified radius of the reference point.
        """
        neighbors = []
        for other_point in points:
            if other_point != point:
                distance = np.linalg.norm(np.array(point) - np.array(other_point))
                if distance < self.connect_radius:
                    neighbors.append(other_point)
        return neighbors
    
    def connect_neighbors(self):
    # Iterate over each point in the graph
        for i, point1 in enumerate(self.graph):
            for j, point2 in enumerate(self.graph):
                if i != j:
                    distance = np.linalg.norm(np.array(point1[:2]) - np.array(point2[:2]))
                    if distance < self.connect_radius:
                        self.graph[i][2].append(j)  # Add index of neighbor to neighbor list  


    def euclidean_distance(self, point1, point2):
        """
        Calculate the Euclidean distance between two points.

        Args:
            point1 (tuple): Coordinates of the first point (x, y).
            point2 (tuple): Coordinates of the second point (x, y).

        Returns:
            float: Euclidean distance between the two points.
        """
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def reconstruct_path(self, came_from, current):
        """
        Reconstruct the path from the start node to the current node.

        Args:
            came_from (dict): A dictionary containing the previous node for each node in the path.
            current: The current node in the path.

        Returns:
            list: The reconstructed path from the start node to the current node.
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def astar(self, graph, start_index, goal_index):
        """
        Perform the A* algorithm to find the shortest path from the start node to the goal node.

        Args:
            graph (list): A list representing the graph where each element is a tuple
                        containing the coordinates of a node and its neighbors.
            start_index (int): Index of the start node in the graph.
            goal_index (int): Index of the goal node in the graph.

        Returns:
            list: The shortest path from the start node to the goal node.
        """
        # Initialize sets for open and closed nodes
        open_set = [(0, start_index)]
        closed_set = set()

        # Initialize dictionaries to store costs and previous nodes
        g_score = {index: float('inf') for index in range(len(graph))}
        g_score[start_index] = 0
        came_from = {}

        while open_set:
            # Get the node with the lowest f_score from the open set
            current_score, current_index = heapq.heappop(open_set)

            # Check if the current node is the goal node
            if current_index == goal_index:
                return self.reconstruct_path(came_from, current_index)

            # Add the current node to the closed set
            closed_set.add(current_index)

            # Explore neighbors of the current node
            for neighbor_index in graph[current_index][2]:
                # Skip neighbors in the closed set
                if neighbor_index in closed_set:
                    continue

                # Calculate the tentative g_score for the neighbor
                tentative_g_score = g_score[current_index] + self.euclidean_distance(graph[current_index][:2],
                                                                                graph[neighbor_index][:2])

                # Update the g_score and came_from if the tentative g_score is better
                if tentative_g_score < g_score[neighbor_index]:
                    came_from[neighbor_index] = current_index
                    g_score[neighbor_index] = tentative_g_score

                    # Add the neighbor to the open set
                    heapq.heappush(open_set, (tentative_g_score + self.euclidean_distance(
                        graph[neighbor_index][:2], graph[goal_index][:2]), neighbor_index))

        # If the goal node is unreachable
        return []

    
    def pixels_to_map_coordinates(self, pixel_x, pixel_y):
        """
        Convert pixel coordinates in the OccupancyGrid map to map coordinates.

        Parameters:
            pixel_x (int): Pixel column index in the OccupancyGrid map.
            pixel_y (int): Pixel row index in the OccupancyGrid map.

         Returns:
            tuple: A tuple containing the map coordinates (map_x, map_y).
               - map_x (float): X-coordinate in the map frame.
               - map_y (float): Y-coordinate in the map frame.

        Notes:
        - pixel_x and pixel_y represent the coordinates of a pixel in the OccupancyGrid map.
        - map_x and map_y represent the corresponding coordinates in the map frame.
        - map.info is an instance of nav_msgs/OccupancyGrid's info field,
          which contains information about the map's resolution and origin.
    
        """

        map_resolution = self.node.map.info.resolution
        map_origin_x = self.node.map.info.origin.position.x
        map_origin_y = self.node.map.info.origin.position.y

        # Convert orientation quaternion to Euler angles
        orientation_quaternion = (
        self.node.map.info.origin.orientation.x,
        self.node.map.info.origin.orientation.y,
        self.node.map.info.origin.orientation.z,
        self.node.map.info.origin.orientation.w
        )
        roll, pitch, yaw = euler_from_quaternion(orientation_quaternion)

        # Apply resolution
        map_x = pixel_x * map_resolution
        map_y = pixel_y * map_resolution

        # Apply rotation
        map_x = map_x * np.cos(yaw) - map_y * np.sin(yaw)
        map_y = map_x * np.sin(yaw) + map_y * np.cos(yaw)

        # Apply translation
        map_x += map_origin_x
        map_y += map_origin_y

        return map_x, map_y
    
    def map_coordinates_to_pixels(self, map_x, map_y):
        """
        Convert map coordinates to pixel coordinates in the occupancy grid.

        Parameters:
            map_x (float): X-coordinate in the map frame.
            map_y (float): Y-coordinate in the map frame.

        Returns:
            tuple: A tuple containing the pixel coordinates (pixel_x, pixel_y).
               - pixel_x (int): Pixel column index in the occupancy grid.
               - pixel_y (int): Pixel row index in the occupancy grid.
        """
        map_resolution = self.node.map.info.resolution
        map_origin_x = self.node.map.info.origin.position.x
        map_origin_y = self.node.map.info.origin.position.y
        map_orientation = self.node.map.info.origin.orientation

        # Convert Quaternion to Euler angles (roll, pitch, yaw)
        quaternion = (map_orientation.x, map_orientation.y, map_orientation.z, map_orientation.w)#
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Apply rotation and translation
        rotated_map_x = (map_x - map_origin_x) * np.cos(yaw) - (map_y - map_origin_y) * np.sin(yaw)
        rotated_map_y = (map_x - map_origin_x) * np.sin(yaw) + (map_y - map_origin_y) * np.cos(yaw)

        # Convert to pixel coordinates
        pixel_x = int(rotated_map_x / map_resolution)
        pixel_y = int(rotated_map_y / map_resolution)

        return pixel_x, pixel_y


if __name__ == "main":
    planner = PathPlan()

# def main(args=None):
#     # rclpy.init(args=args)
#     planner = PathPlan()
    # rclpy.spin(planner)
    # rclpy.shutdown()
