#!/usr/bin/env python3
#
# Copyright (c) 2026, Jen-Hung Ho 
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#


"""
pointcloud_avoidance.py logic versions:

Version 1: PointCloud2 only
    - Uses depth camera (vision) for obstacle detection and avoidance.
    - No LIDAR or costmap integration.

Version 2: PointCloud2 + Costmap
    - Uses both depth camera and local costmap for obstacle detection.
    - Costmap is prioritized for close-proximity hazards.
    - Note: If the costmap is published from a remote NAV2 stack, network latency or packet loss may affect obstacle avoidance performance. Ensure reliable, low-latency networking for safe operation.

Version 3: PointCloud2 + LIDAR
    - Vision (depth camera) is the primary sensor.
    - LIDAR assists in close/dead zone scenarios (e.g., when vision is blind or object is very close).
    - Fusion logic chooses the best data source for safe navigation.
    - **Recommended:** Version 3 is preferred for most applications, as LIDAR covers vision dead zones and improves overall obstacle detection and navigation performance.

See each version's logic in the __init__() and callback methods.
"""

import statistics
import rclpy  # Python library for ROS 2
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, SetParametersResult
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

import math
import statistics


class PointCloudAvoidance(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start' and param.type_ == Parameter.Type.BOOL:
                self.start = param.value
                self.get_logger().info('start= {}'.format(bool(param.value)))
            elif param.name == 'stop_distance' and param.type_ == Parameter.Type.DOUBLE:
                self.get_logger().info('stop distance= {}'.format(str(param.value)))
                self.stop_distance = param.value
                self.get_logger().info('distance: {}'.format(self.stop_distance))
        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('pointcloud_avoidance')

        # Declare parameters (add more as needed)
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.pointcloud_topic = self.declare_parameter('pointcloud_topic', '/camera/depth/points').get_parameter_value().string_value
        self.local_costmap_topic = self.declare_parameter('local_costmap_topic', '/local_costmap/costmap').get_parameter_value().string_value
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').get_parameter_value().string_value
        self.QosReliability = self.declare_parameter('qos_reliability', False).get_parameter_value().bool_value
        self.start = self.declare_parameter('start', False).get_parameter_value().bool_value
        self.stop_distance = self.declare_parameter('stop_distance', 0.7).get_parameter_value().double_value
        self.fov_deg = self.declare_parameter('fov_deg', 60.0).get_parameter_value().double_value
        self.height_filter = self.declare_parameter('height_filter', -0.1).get_parameter_value().double_value
        self.speed_gain = self.declare_parameter('speed', 0.1).get_parameter_value().double_value
        self.speed_up = self.declare_parameter('speed_up', 0.03).get_parameter_value().double_value
        self.twist_z = self.declare_parameter('twist_z', 0.06).get_parameter_value().double_value
        self.version = self.declare_parameter('version', 1).get_parameter_value().integer_value
        self.min_valid_distance_lidar = self.declare_parameter('min_valid_distance_lidar', 0.25).get_parameter_value().double_value
        self.min_valid_distance_vision = self.declare_parameter('min_valid_distance_vision', 0.6).get_parameter_value().double_value
        self.flip_costmap_sectors = self.declare_parameter('flip_costmap_sectors', True).get_parameter_value().bool_value
        self.costmap_trigger_threshold = self.declare_parameter('costmap_trigger_threshold', 15).get_parameter_value().integer_value
        self.movement_threshold = self.declare_parameter('movement_threshold', 2.0).get_parameter_value().double_value 

        # Initialize LIDAR parameters
        self.lidar_init()

        self.get_logger().info('-------------------------------------------')
        self.get_logger().info('cmd_vel_topic       : {}'.format(self.cmd_vel_topic))
        self.get_logger().info('pointcloud_topic    : {}'.format(self.pointcloud_topic))
        self.get_logger().info('local_costmap_topic : {}'.format(self.local_costmap_topic))
        self.get_logger().info('scan_topic          : {}'.format(self.scan_topic))
        self.get_logger().info('QosReliability      : {}'.format(self.QosReliability))
        self.get_logger().info('start               : {}'.format(self.start))
        self.get_logger().info('stop_distance       : {}'.format(self.stop_distance))
        self.get_logger().info('fov_deg             : {}'.format(self.fov_deg))
        self.get_logger().info('height_filter       : {}'.format(self.height_filter))
        self.get_logger().info('speed               : {}'.format(self.speed_gain))
        self.get_logger().info('speed_up            : {}'.format(self.speed_up))
        self.get_logger().info('twist_z             : {}'.format(self.twist_z))
        self.get_logger().info('version             : {}'.format(self.version))
        self.get_logger().info('flip_costmap_sectors: {}'.format(self.flip_costmap_sectors))
        self.get_logger().info('costmap_trigger_threshold: {}'.format(self.costmap_trigger_threshold))
        self.get_logger().info('min_valid_distance_lidar: {}'.format(self.min_valid_distance_lidar))
        self.get_logger().info('min_valid_distance_vision: {}'.format(self.min_valid_distance_vision))
        self.get_logger().info('movement_threshold  : {}'.format(self.movement_threshold))
        self.get_logger().info('-------------------------------------------')

        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # QoS settings
        qos_profile = QoSProfile(depth=10)
        if self.QosReliability:
            qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        else:
            qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        # Publisher for robot movement
        self.pub_twist = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.empty_pointcloud_count = 0
        self.latest_costmap = None
        self.costmap_lock = threading.Lock()
        self.last_move = 0

        # Check if /local_costmap topic exists
        topic_list = [name for name, _ in self.get_topic_names_and_types()]

        if self.version == 2 and self.local_costmap_topic not in topic_list:
            self.get_logger().info(f"Available topics: {topic_list}")
            self.get_logger().warn(f"Topic {self.local_costmap_topic} not found. Wait for Local Cost message.")
            # self.version = 1  # Fallback to version 1 if topic not found
            # topic_info = self.get_topic_names_and_types()
            # for name, types in topic_info:
            #     self.get_logger().info(f'Topic: {name}, Types: {types}')

        self.get_logger().info('version            : {}'.format(self.version))

        if self.version ==1:
            self.get_logger().info("Subscribed to PointCloud2 for version 1")
            self.get_logger().info("Subscribed to {}".format(self.pointcloud_topic))

            # Subscriber for PointCloud2
            self.subscription = self.create_subscription(
                PointCloud2,
                self.pointcloud_topic,
                self.pointcloud_callback,
                qos_profile
            )

        elif self.version == 2:
            self.get_logger().info("Subscribed to PointCloud2 and Local Costmap for version 2")
            self.get_logger().info("Subscribed to {}".format(self.pointcloud_topic))
            self.get_logger().info("Subscribed to {}".format(self.local_costmap_topic))

            # Create message_filters subscribers (do not replace your existing subscriptions)
            self.pc2_filter_sub = Subscriber(self, PointCloud2, self.pointcloud_topic)
            self.costmap_filter_sub = Subscriber(self, OccupancyGrid, self.local_costmap_topic)

            # ApproximateTimeSynchronizer: queue size 10, slop 0.2s (adjust as needed)
            self.ts = ApproximateTimeSynchronizer(
                [self.pc2_filter_sub, self.costmap_filter_sub], queue_size=10, slop=0.2
            )
            self.ts.registerCallback(self.sync_callback)

        elif self.version == 3:
            self.get_logger().info("Subscribed to PointCloud2 and LaserScan for version 3")
            self.get_logger().info("Subscribed to {}".format(self.pointcloud_topic))
            self.get_logger().info("Subscribed to {}".format(self.scan_topic))
            self.pc2_filter_sub = Subscriber(self, PointCloud2, self.pointcloud_topic)
            self.lidar_filter_sub = Subscriber(self, LaserScan, self.scan_topic)
            self.ts = ApproximateTimeSynchronizer(
                [self.pc2_filter_sub, self.lidar_filter_sub], queue_size=10, slop=0.2
            )
            self.ts.registerCallback(self.sync_lidar_callback)


        # Timer for periodic actions (optional)
        self.create_timer(0.1, self.timer_callback)


    #
    # Initialize LIDAR sector parameters for RPLIDAR S2
    #
    def lidar_init(self):
        # only for version 3
        if self.version != 3:
            return

        # Declare LIDAR sector parameters
        self.lidar_min_measure_distance = self.declare_parameter('lidar_min_measure_distance', 0.05).get_parameter_value().double_value
        self.lidar_sector_left_min = self.declare_parameter('lidar_sector_left_min', -170.0).get_parameter_value().double_value
        self.lidar_sector_left_max = self.declare_parameter('lidar_sector_left_max', -150.0).get_parameter_value().double_value
        self.lidar_sector_center_min1 = self.declare_parameter('lidar_sector_center_min1', -180.0).get_parameter_value().double_value
        self.lidar_sector_center_max1 = self.declare_parameter('lidar_sector_center_max1', -170.0).get_parameter_value().double_value
        self.lidar_sector_center_min2 = self.declare_parameter('lidar_sector_center_min2', 170.0).get_parameter_value().double_value
        self.lidar_sector_center_max2 = self.declare_parameter('lidar_sector_center_max2', 180.0).get_parameter_value().double_value
        self.lidar_sector_right_min = self.declare_parameter('lidar_sector_right_min', 150.0).get_parameter_value().double_value
        self.lidar_sector_right_max = self.declare_parameter('lidar_sector_right_max', 170.0).get_parameter_value().double_value

        self.get_logger().info('LIDAR sector parameters initialized:')
        self.get_logger().info('-------------------------------------------')
        self.get_logger().info('lidar_min_measure_distance: {}'.format(self.lidar_min_measure_distance))
        self.get_logger().info('lidar_sector_left_min  : {}'.format(self.lidar_sector_left_min))
        self.get_logger().info('lidar_sector_left_max  : {}'.format(self.lidar_sector_left_max))
        self.get_logger().info('lidar_sector_center_min1: {}'.format(self.lidar_sector_center_min1))
        self.get_logger().info('lidar_sector_center_max1: {}'.format(self.lidar_sector_center_max1))
        self.get_logger().info('lidar_sector_center_min2: {}'.format(self.lidar_sector_center_min2))
        self.get_logger().info('lidar_sector_center_max2: {}'.format(self.lidar_sector_center_max2))
        self.get_logger().info('lidar_sector_right_min : {}'.format(self.lidar_sector_right_min))
        self.get_logger().info('lidar_sector_right_max : {}'.format(self.lidar_sector_right_max))


    #
    # Version 1: Process PointCloud2 data for obstacle avoidance
    #
    def pointcloud_callback(self, msg):
        # If not started, no action to process robot movement
        if self.start != True:
            return
        else:
            self.last_move = 0

        if self.version == 2:
            self.get_logger().info("Received PointCloud2 message - IGNORE in version 2")
            return  # Ignore if in version 2 mode, handled by sync_callback

        # Example: parse point cloud, detect obstacles, set self.twist
        self.get_logger().info("Received PointCloud2 message")

        left_min, center_min, right_min, num_points = self.filter_pointcloud_sectors(msg)

        # if no data to process contine than 5 frames then stop the robot
        if num_points is None or num_points < 100:
            self.empty_pointcloud_count += 1
            self.get_logger().info("Not enough points to process")
            if self.empty_pointcloud_count >= 5:
                twist = Twist()  # All values default to zero
                self.pub_twist.publish(twist)
                self.get_logger().warn("Stopping robot due to consecutive empty pointclouds")
                self.empty_pointcloud_count = 0  # Reset counter if valid data
            return

        self.get_logger().info(f"Left min z: {left_min:.2f}, Center min z: {center_min:.2f}, Right min z: {right_min:.2f}")

        # Movement decision based on point cloud data
        twist = self.pointcloud_move_decision(left_min, center_min, right_min, 'Vision')
        if twist is not None:
            self.pub_twist.publish(twist)


    #
    # Version 2: Synchronized callback for PointCloud2 and OccupancyGrid
    #
    def sync_callback(self, pc2_msg, costmap_msg):

        # If not started, no action to process robot movement
        if self.start != True:
            return
        else:
            self.last_move = 0


        # For testing: just log the timestamps and message types
        self.get_logger().info(
            f"SYNC CALLBACK: PointCloud2 stamp={pc2_msg.header.stamp.sec}.{pc2_msg.header.stamp.nanosec}, "
            f"OccupancyGrid stamp={costmap_msg.header.stamp.sec}.{costmap_msg.header.stamp.nanosec}"
        )

        left_min, center_min, right_min, num_points = self.filter_pointcloud_sectors(pc2_msg)
        if num_points is None or num_points < 100:
            self.empty_pointcloud_count += 1
            self.get_logger().info("Not enough points to process")
            if self.empty_pointcloud_count >= 2:
                twist = Twist()
                self.pub_twist.publish(twist)
                self.get_logger().warn("Stopping robot due to consecutive empty pointclouds")
                self.empty_pointcloud_count = 0
            return

        self.get_logger().info(f"Left min z: {left_min:.2f}, Center min z: {center_min:.2f}, Right min z: {right_min:.2f}")

        # Movement decision based on combined point cloud and costmap data
        self.cost_map_move_decision(left_min, center_min, right_min, costmap_msg)

    #    
    # Synchronized callback for PointCloud2 and LaserScan
    # Vision 3 (depth camera) is the primary sensor; LIDAR assists in close/dead zone scenarios.
    #
    def sync_lidar_callback(self, pc2_msg, lidar_msg):
        """
        Synchronized callback for PointCloud2 and LaserScan.
        Vision (depth camera) is used as the primary sensor for obstacle detection.
        LIDAR is used as an assistant for close-range or dead zone scenarios where vision data is insufficient.
        """

        # If not started, no action to process robot movement
        if self.start != True:
            return
        else:
            self.last_move = 0

        self.get_logger().info(
            f"[Vision+Lidar] SYNC CALLBACK: PointCloud2 stamp={pc2_msg.header.stamp.sec}.{pc2_msg.header.stamp.nanosec}, "
            f"LaserScan stamp={lidar_msg.header.stamp.sec}.{lidar_msg.header.stamp.nanosec}"
        )
        

        left_min, center_min, right_min, num_points = self.filter_pointcloud_sectors(pc2_msg)
        if num_points is None or num_points < 100:
            self.empty_pointcloud_count += 1
            self.get_logger().info("Not enough points to process")
            if self.empty_pointcloud_count >= 2:
                twist = Twist()
                self.pub_twist.publish(twist)
                self.get_logger().warn("Stopping robot due to consecutive empty pointclouds")
                self.empty_pointcloud_count = 0
            return

        lidar_left, lidar_center, lidar_right = self.process_lidar_sectors(lidar_msg)

        self.get_logger().info(f"Depths: L={left_min:.2f}, C={center_min:.2f}, R={right_min:.2f}")
        self.get_logger().info(f"LIDAR:  L={lidar_left:.2f}, C={lidar_center:.2f}, R={lidar_right:.2f}")

        # Combine logic: if LIDAR confirms clearance, override depth
        twist = self.lidar_fusion_decision(left_min, center_min, right_min,
                                        lidar_left, lidar_center, lidar_right)
        if twist:
            self.pub_twist.publish(twist)


    #
    # Filter PointCloud2 into left, center, right sectors
    #
    def filter_pointcloud_sectors(self, msg):
        # Parse PointCloud2 into directional depth sectors
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        left_depths, center_depths, right_depths = [], [], []

        # if no data to process contine than 5 frames then stop the robot
        if len(points) < 100:
            return None, None, None, len(points)

        # Only consider points within FOV (±angle limit from center)
        # (60 degree Field Of View ±30° from center)
        angle_limit = math.radians(self.fov_deg / 2)

        # Check the height of data because the the robot only care the obstacle height < 0.5m
        for x, y, z in points:
            if z <= 0.0:
                continue

            # RED-103 y down(+), up(-) -> camera is 0.1m above the ground
            if y < self.height_filter:
                continue

            angle = math.atan2(x, z)
            if abs(angle) > angle_limit:
                continue

            # Divide FOV into left, center, right
            view = self.fov_deg / 3 / 2
            if angle < -math.radians(view):
                left_depths.append(z)
            elif angle > math.radians(view):
                right_depths.append(z)
            else:
                center_depths.append(z)

        def safe_mean(depths):
            return sum(depths) / len(depths) if depths else 0.0

        left_min = safe_mean(left_depths)
        center_min = safe_mean(center_depths)
        right_min = safe_mean(right_depths)

        return left_min, center_min, right_min, len(points)


    #
    # Movement logic for PointCloud2 only (version 1)
    #
    def pointcloud_move_decision(self, left_min, center_min, right_min, sensor_source="Vision"):
        # Set min threshold based on sensor
        if sensor_source.lower() == "lidar":
            # 25cm for LIDAR
            min_valid_distance = self.min_valid_distance_lidar
        else:
            # 60cm for depth camera
            min_valid_distance = self.min_valid_distance_vision

        # Movement logic for version 1 (PointCloud2 only) or 3 (PointCloud2 + LIDAR)
        self.get_logger().debug(f"Making movement decision based on {sensor_source}")
        sector_values = {
            'left': left_min,
            'center': center_min,
            'right': right_min
        }
        max_sector = max(sector_values, key=sector_values.get)
        min_sector = min(sector_values, key=sector_values.get)
        min_value = sector_values[min_sector]

        twist = Twist()
        stop_dist = self.stop_distance
        blind_zone = False

        if sector_values[max_sector] < stop_dist:
            twist.linear.x = 0.0
            twist.angular.z = -self.twist_z
            self.get_logger().info(f"[{sensor_source}]: All sectors blocked (L={left_min:.2f}, R={right_min:.2f}): Default rotating RIGHT to unblock")
        else:
            # Default forward speed 0.1 m/s
            twist.linear.x = self.speed_gain
            if min_value < min_valid_distance:
                # Too close to obstacle, stop forward movement
                twist.linear.x = 0.0
                blind_zone = True
                self.get_logger().info(f"[{sensor_source}]: Blind zone or too close (<{min_valid_distance}m): STOP")
            
            if min_value > self.movement_threshold + 0.5:
                # Path is clear, speed_gain + speed up 0.07 m/s
                twist.linear.x = self.speed_gain + self.speed_up
                twist.angular.z = 0.0
                self.get_logger().info(f"[{sensor_source}]: Most close obstacle is far (>{self.movement_threshold + 0.5:.1f}m): Moving FORWARD")
            elif min_sector == 'center':
                if left_min < right_min:
                    twist.angular.z = -self.twist_z
                    self.get_logger().info(f"[{sensor_source}]: Obstacle closest ahead: Turning RIGHT")
                else:
                    twist.angular.z = self.twist_z
                    self.get_logger().info(f"[{sensor_source}]: Obstacle closest ahead: Turning LEFT")
            elif min_sector == 'left':
                if center_min < self.movement_threshold or blind_zone:
                    # Too close to obstacle on left or blind zone slow down and turn right
                    twist.linear.x = self.speed_gain - self.speed_up
                    twist.angular.z = -self.twist_z
                    self.get_logger().info(f"[{sensor_source}]: Obstacle very close on LEFT or blind zone: Turning RIGHT")
                else:
                    # Path clear and obstacle on left, move forward
                    twist.angular.z = 0.0
                    self.get_logger().info(f"[{sensor_source}]: Obstacle closest on LEFT: Moving FORWARD")
            elif min_sector == 'right':
                if center_min < self.movement_threshold or blind_zone:
                    # Too close to obstacle on right or blind zone slow down and turn left
                    twist.linear.x = self.speed_gain - self.speed_up
                    twist.angular.z = self.twist_z
                    self.get_logger().info(f"[{sensor_source}]: Obstacle very close on RIGHT or blind zone: Turning LEFT")
                else:
                    # Path clear and obstacle on right, move forward
                    twist.angular.z = 0.0
                    self.get_logger().info(f"[{sensor_source}]: Obstacle closest on RIGHT: Moving FORWARD   ")
            else:
                twist.angular.z = 0.0
                self.get_logger().warn(f"[{sensor_source}]: Obstacle unknown: Moving FORWARD")

        #self.pub_twist.publish(twist)
        return twist

    #
    # Movement logic combining PointCloud2 and Costmap (version 2)
    #
    def cost_map_move_decision(self, left_min, center_min, right_min, costmap_msg):
        self.get_logger().info("Making movement decision based on costmap and pointcloud")

        # Parse costmap sectors within 1.5m radius
        left_cost, center_cost, right_cost = self.process_costmap(costmap_msg)

        self.get_logger().info(f"Depths: L={left_min:.2f}, C={center_min:.2f}, R={right_min:.2f}")
        self.get_logger().info(f"Costs:  L={left_cost}, C={center_cost}, R={right_cost}")

        # Combine depth and costmap cues
        twist = Twist()
        cost_values = {
            'left': left_cost,
            'center': center_cost,
            'right': right_cost
        }
        max_sector = max(cost_values, key=cost_values.get)
        max_cost = max(cost_values.values())
        min_cost = min(cost_values.values())
        
        # Step 1: Prioritize Costmap for Close-Proximity Hazards
        # Close obstacle detected
        if max_cost >= self.costmap_trigger_threshold:
            if min_cost > self.costmap_trigger_threshold:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("All sectors high cost -> STOP")
            else:
                twist.linear.x = 0.07  # gentle forward motion
                if max_sector == 'center':
                    # Choose the clearer side to turn toward
                    if cost_values['left'] < cost_values['right']:
                        twist.angular.z = 0.1  # turn left
                        self.get_logger().warn("Center blocked → Turning LEFT (lower cost on left)")
                    else:
                        twist.angular.z = -0.1  # turn right
                        self.get_logger().warn("Center blocked → Turning RIGHT (lower cost on right)")
                else:
                    # Turn away from the most dangerous sector
                    twist.angular.z = {
                        'left': -0.1,
                        'right': 0.1
                    }[max_sector]
                    self.get_logger().warn(f"High cost detected in {max_sector.upper()} → Turning away")

            self.pub_twist.publish(twist)
            return

        # Step 2:If Costmap Is Safe, Use Depth for Navigation
        # Safe to use depth
        twist = self.pointcloud_move_decision(left_min, center_min, right_min, 'Vision')
        if twist:
            self.pub_twist.publish(twist)

    #
    # Process OccupancyGrid costmap data
    # 1. Filter costmap into left, center, right sectors
    #
    def process_costmap(self, msg):
        # Process costmap data for obstacle avoidance
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Convert 1D costmap list data into 2D grid
        grid = [msg.data[i:i+width] for i in range(0, len(msg.data), width)]

        # Robot is at the center of the costmap
        cx, cy = width // 2, height // 2  # Robot is centered if rolling_window = true

        # Count cost cells in left, center, right sectors within 1.5m radius
        # Counts the number of "lethal" or "inflated" cells (cost ≥ 80) in a vertical slice of the grid,
        # within a certain distance (1.5 meters) from the robot.
        def count_cost_cells(dx_range):
            count = 0
            for dy in range(-3, 4):  # Vertical slice
                for dx in dx_range:
                    i, j = cx + dx, cy + dy
                    if 0 <= i < width and 0 <= j < height:
                        dist = math.sqrt((dx * resolution)**2 + (dy * resolution)**2)
                        if dist < 1.5:
                            cost = grid[j][i]
                            # if cost > 30:
                            #     self.get_logger().info(f"Checking cell ({i},{j}) → cost={grid[j][i]}, dist={dist:.2f}")
                            if cost >= 50:  # Lethal or inflated
                                count += 1
            return count

        if self.flip_costmap_sectors:
            # Flipped logic: dx > 0 is left, dx < 0 is right
            left_cost   = count_cost_cells(range(1, 7))
            center_cost = count_cost_cells(range(-1, 2))
            right_cost  = count_cost_cells(range(-6, -1))
        else:
            # Default logic: dx < 0 is left, dx > 0 is right
            # left of the robot (from 6 cells left up to just left of center).
            left_cost   = count_cost_cells(range(-6, -1))
            # center (from 1 cell left to 1 cell right of center).
            center_cost = count_cost_cells(range(-1, 2))
            # right of the robot (from 1 cell right to 6 cells right of center).
            right_cost  = count_cost_cells(range(1, 7))

        self.get_logger().debug(f"Costmap density - Left: {left_cost}, Center: {center_cost}, Right: {right_cost}")

        return left_cost, center_cost, right_cost


    #
    # Movement logic combining PointCloud2 and LIDAR (version 3)
    #
    def process_lidar_sectors(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)

        def sector_avg(start_deg, end_deg):
            start_rad = math.radians(start_deg)
            end_rad = math.radians(end_deg)
            start_idx = int((start_rad - angle_min) / angle_increment)
            end_idx = int((end_rad - angle_min) / angle_increment)
            selected = [
                r for r in ranges[start_idx:end_idx]
                if r > self.lidar_min_measure_distance and not math.isinf(r) and not math.isnan(r)
            ]
            self.get_logger().debug(f"Sector {start_deg} to {end_deg}: idx {start_idx}-{end_idx}, count={len(selected)}")
            return sum(selected) / len(selected) if selected else 0.0

        # Adjust for LIDAR viewing range if nessary
        # The current value is setup for RPLIDAR S2
        # lidar_min_measure_distance', 0.05
        # lidar_sector_left_min', -170.0,   lidar_sector_left_max', -150.0
        # lidar_sector_center_min1', -180.0 lidar_sector_center_max1', -170.0
        # lidar_sector_center_min2', 170.0, lidar_sector_center_max2', 180.0
        # lidar_sector_right_min', 150.0,   lidar_sector_right_max', 170.0

        # Left: -170° to -150°
        left = sector_avg(self.lidar_sector_left_min, self.lidar_sector_left_max) 
        # Center: -180° to -170° and 170° to 180°
        center1 = sector_avg(self.lidar_sector_center_min1, self.lidar_sector_center_max1)
        center2 = sector_avg(self.lidar_sector_center_min2, self.lidar_sector_center_max2)
        center_vals = []
        if center1 > 0:
            center_vals.append(center1)
        if center2 > 0:
            center_vals.append(center2)
        center = sum(center_vals) / len(center_vals) if center_vals else 0.0
        # Right: 150° to 170°
        right = sector_avg(self.lidar_sector_right_min, self.lidar_sector_right_max)
        

        return left, center, right

    #
    # Combine PointCloud2 and LIDAR data for movement decision
    #
    def lidar_fusion_decision(self, d_left, d_center, d_right, l_left, l_center, l_right):

        # Step 1: Use LIDAR data for close obstacles
        lidar_values = {
            'left': l_left,
            'center': l_center,
            'right': l_right
        }
        max_sector = max(lidar_values, key=lidar_values.get)
        min_sector = min(lidar_values, key=lidar_values.get)
        max_dist = lidar_values[max_sector]
        min_dist = lidar_values[min_sector]

        twist = Twist()
        stop_dist = self.stop_distance

        depth_blind = (d_left == 0.0 or d_center == 0.0 or d_right == 0.0)
        lidar_close = min(l_left, l_center, l_right) < self.min_valid_distance_vision

        # Prioritize LIDAR for immediate obstacles
        # Close obstacle detected by LIDAR within 0.6m or depth blind zone (0.0: < 0.5m)
        if depth_blind or lidar_close:
            # Step 1: If Vision has dead zones, use Lidar decision
            return self.pointcloud_move_decision(l_left, l_center, l_right, 'Lidar')
        else:
            # Step 2: If Vision is clear (min > 0.6m), use PointCloud2 decision
            return self.pointcloud_move_decision(d_left, d_center, d_right, 'Vision')


    def timer_callback(self):

        # If not started, ensure robot is stopped
        if self.start != True:
            if self.last_move < 2:
                self.last_move += 1
            elif self.last_move == 2:
                twist = Twist()
                self.pub_twist.publish(twist)
                self.get_logger().info("PointCloudAvoidance stop the robot.")
                self.last_move = 7

            return
        
        self.last_move = 0

def main(args=None):
    rclpy.init(args=args)

    global executor
    executor = rclpy.executors.MultiThreadedExecutor()

    node = PointCloudAvoidance()
    executor.add_node(node)

    try:
        #rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        print('\ncontrol-c: laser_avoidance_node shutting down')
        pass
    finally:
        # Destroy the node explictly - don't depend on garbage collector
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()