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

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import message_filters
import numpy as np
import math


from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, SetParametersResult
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class DepthVisionAvoidance(Node):

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
        super().__init__('depth_vision_avoidance')
        # Declare parameters (add more as needed)
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.camera_depth_topic = self.declare_parameter('camera_depth_topic', '/camera/depth/image_raw_decompressed').get_parameter_value().string_value
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').get_parameter_value().string_value
        self.QosReliability = self.declare_parameter('qos_reliability', False).get_parameter_value().bool_value
        self.start = self.declare_parameter('start', False).get_parameter_value().bool_value
        self.stop_distance = self.declare_parameter('stop_distance', 0.7).get_parameter_value().double_value
        self.fov_deg = self.declare_parameter('fov_deg', 60.0).get_parameter_value().double_value
        self.height_filter = self.declare_parameter('height_filter', -0.1).get_parameter_value().double_value
        self.speed_gain = self.declare_parameter('speed', 0.1).get_parameter_value().double_value
        self.speed_up = self.declare_parameter('speed_up', 0.03).get_parameter_value().double_value
        self.twist_z = self.declare_parameter('twist_z', 0.1).get_parameter_value().double_value
        self.version = self.declare_parameter('version', 1).get_parameter_value().integer_value
        self.min_valid_distance_lidar = self.declare_parameter('min_valid_distance_lidar', 0.25).get_parameter_value().double_value
        self.min_valid_distance_vision = self.declare_parameter('min_valid_distance_vision', 0.6).get_parameter_value().double_value
        self.movement_threshold = self.declare_parameter('movement_threshold', 2.0).get_parameter_value().double_value 

        # Initialize LIDAR parameters
        self.lidar_init()

        self.get_logger().info('-------------------------------------------')
        self.get_logger().info('cmd_vel_topic       : {}'.format(self.cmd_vel_topic))
        self.get_logger().info('camera_depth_topic  : {}'.format(self.camera_depth_topic))
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
        self.get_logger().info('min_valid_distance_lidar: {}'.format(self.min_valid_distance_lidar))
        self.get_logger().info('min_valid_distance_vision: {}'.format(self.min_valid_distance_vision))
        self.get_logger().info('movement_threshold  : {}'.format(self.movement_threshold))
        self.get_logger().info('-------------------------------------------')

        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)


        self.last_move = 0
        self.empty_depthvision_count = 0

        # QoS settings
        qos_profile = QoSProfile(depth=10)
        if self.QosReliability:
            qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        else:
            qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        # Publisher for robot movement
        self.pub_twist = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        if self.version == 1:
            # Subscribers for depth vision (version 1)
            self.get_logger().info('Version 1: Depth image only mode initialized')
            self.get_logger().info("Subscribed to depth vision for version {}".format(self.version))
            self.get_logger().info("Subscribed to {}".format(self.camera_depth_topic))
            # Subscriber for depth image (vision primary)
            self.depth_sub = self.create_subscription(
                Image,
                self.camera_depth_topic,
                self.depth_callback,
                qos_profile
            )
            
        elif self.version == 2:
            # Subscribers for depth vision and LIDAR (version 2)
            self.get_logger().info('Version 2: Depth image + LIDAR fusion mode initialized')
            self.get_logger().info("Subscribed to depth vision and LaserScan for version {}".format(self.version))
            self.get_logger().info("Subscribed to {}".format(self.camera_depth_topic))
            self.get_logger().info("Subscribed to {}".format(self.scan_topic))
            self.depth_filter_sub = message_filters.Subscriber(self, Image, self.camera_depth_topic)
            self.lidar_filter_sub = message_filters.Subscriber(self, LaserScan, self.scan_topic)
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [self.depth_filter_sub, self.lidar_filter_sub], queue_size=10, slop=0.2
            )
            self.ts.registerCallback(self.sync_depth_lidar_callback)

        # Timer for periodic actions (optional)
        self.create_timer(0.1, self.timer_callback)


        self.get_logger().info('DepthVisionAvoidance node initialized')

    # Add your methods here
    #
    # Initialize LIDAR sector parameters for RPLIDAR S2
    #
    def lidar_init(self):
        # only for version 1
        if self.version != 2:
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

    #
    # Vision 1: Depth image only callback
    #
    def depth_callback(self, msg):
        # Process the depth Image message here
        # Example: Convert to numpy array, detect obstacles, compute Twist
        # For now, just log receipt
        # self.get_logger().info('Received depth image')
        # Add your vision-based avoidance logic

         # If not started, no action to process robot movement
        if self.start != True:
            return
        else:
            self.last_move = 0

        if self.version == 2:
            self.get_logger().info("Received Lidar message - IGNORE in version 2")
            return  # Ignore if in version 2 mode, handled by sync_callback

        # Example: parse point cloud, detect obstacles, set self.twist
        self.get_logger().debug("Received depth vision message")

        left_min, center_min, right_min, num_points = self.filter_depthimage_sectors(msg)

        # if no data to process contine than 5 frames then stop the robot
        if num_points is None or num_points < 100:
            self.empty_depthvision_count += 1
            self.get_logger().info("Not enough points to process")
            if self.empty_depthvision_count >= 5:
                twist = Twist()  # All values default to zero
                self.pub_twist.publish(twist)
                self.get_logger().warn("Stopping robot due to consecutive empty pointclouds")
                self.empty_depthvision_count = 0  # Reset counter if valid data
            return

        self.get_logger().info(f"Left min z: {left_min:.2f}, Center min z: {center_min:.2f}, Right min z: {right_min:.2f}")

        # Movement decision based on point cloud data
        twist = self.depthvision_move_decision(left_min, center_min, right_min, 'Vision')
        if twist is not None:
            self.pub_twist.publish(twist)


    #
    # Version 2: Synchronized callback for depth image and LIDAR
    #
    def sync_depth_lidar_callback(self, depth_msg, lidar_msg):
        # If not started, no action to process robot movement
        if self.start != True:
            return
        else:
            self.last_move = 0

        self.get_logger().debug("Received synchronized depth vision and LIDAR messages")
        self.get_logger().info(
            f"[Vision+Lidar] SYNC CALLBACK: depth vision stamp={depth_msg.header.stamp.sec}.{depth_msg.header.stamp.nanosec}, "
            f"LaserScan stamp={lidar_msg.header.stamp.sec}.{lidar_msg.header.stamp.nanosec}"
        )

        # Process depth image
        left_min_vision, center_min_vision, right_min_vision, num_points_vision = self.filter_depthimage_sectors(depth_msg)

        # Process LIDAR data
        left_min_lidar, center_min_lidar, right_min_lidar = self.filter_lidar_sectors(lidar_msg)

        # Combined validity check (only stop if BOTH sensors are invalid)
        depth_invalid = (num_points_vision is None or num_points_vision < 100)
        lidar_invalid = (left_min_lidar == 0.0 and center_min_lidar == 0.0 and right_min_lidar == 0.0)
        
        if depth_invalid:
            self.empty_depthvision_count += 1
        else:
            self.empty_depthvision_count = 0
        
        if depth_invalid and lidar_invalid:
            if self.empty_depthvision_count >= 2:
                twist = Twist()
                self.pub_twist.publish(twist)
                self.get_logger().warn("Both depth vision and LIDAR invalid: Stopping robot")
                return

        # Proceed to fustion movement decision
        self.get_logger().info(f"Depths: L={left_min_vision:.2f}, C={center_min_vision:.2f}, R={right_min_vision:.2f}")
        self.get_logger().info(f"LIDAR:  L={left_min_lidar:.2f}, C={center_min_lidar:.2f}, R={right_min_lidar:.2f}")

        # Movement decision based on combined data
        twist = self.fusion_move_decision(left_min_vision, center_min_vision, right_min_vision,
                                           left_min_lidar, center_min_lidar, right_min_lidar)
        
        if twist is not None:
            self.pub_twist.publish(twist)


    #
    # Depth image sector filtering for version 1 and 2
    #
    def filter_depthimage_sectors(self, msg):
        """
        Improved version: Filters the depth image into 3 angular sectors based on ~60° FOV.
        Sectors: left (-30° to -10°), center (-10° to 10°), right (10° to 30°).
        Uses mean depth for robustness to noise, includes configurable height filtering (via height_filter parameter) based on ROS Z (up).
        Returns the mean valid depth in each sector (in meters), and total valid points.
        """
        try:
            import numpy as np
            import cv2
            from cv_bridge import CvBridge
            
            bridge = CvBridge()
            # Convert ROS Image to OpenCV (assuming 16UC1 depth in mm)
            depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            
            # Convert to meters
            depth_array = depth_image.astype(np.float32) / 1000.0
            # Get image dimensions
            height, width = depth_array.shape
            
            # Camera intrinsics
            fx = 579.023681640625
            fy = 579.023681640625
            cx = width // 2
            cy = height // 2
            
            # Create coordinate grids
            x_coords = np.arange(width)
            y_coords = np.arange(height)
            X, Y = np.meshgrid(x_coords, y_coords)

            # Vectorized angle and height calculations
            angles_deg = np.degrees(np.arctan((X - cx) / fx))
            # Calculate real-world Y (height) coordinates 
            real_world_y = - (Y - cy) * depth_array / fy 

            # Y < 0 → above the camera
            # Y = 0 → camera center
            # Y > 0 → below the camera
            # Debug: Print real_world_z range
            self.get_logger().debug(f"real_world_y range: min={np.min(real_world_y):.3f}, max={np.max(real_world_y):.3f}")

            # NumPy's & is overloaded for element-wise logical AND on arrays).
            # Valid mask: depth > 0, not NaN/inf, depth <= 10m, height (Z) <= height_filter
            valid = (depth_array > 0) & (~np.isnan(depth_array)) & (~np.isinf(depth_array)) & (depth_array <= 10.0) & (real_world_y <= self.height_filter)

            # Add FOV filter: only keep points within ±30° (assuming 60° FOV)
            fov_limit_deg = self.fov_deg / 2  # 30.0 degrees
            valid = valid & (np.abs(angles_deg) <= fov_limit_deg)

            # Divide FOV into three equal sectors
            sector_width_deg = self.fov_deg / 3  # 20° each
            half_sector_deg = sector_width_deg / 2  # 10°

            # Sector masks (equal width, centered on 0°)
            # -30 to -10: left
            # -10 to 10: center
            # 10 to 30: right
            left_mask = valid & (angles_deg < -half_sector_deg)
            center_mask = valid & (-half_sector_deg <= angles_deg) & (angles_deg <= half_sector_deg)
            right_mask = valid & (angles_deg > half_sector_deg)


            # Extract depths for each sector
            left_depths = depth_array[left_mask]
            center_depths = depth_array[center_mask]
            right_depths = depth_array[right_mask]

            num_points = np.sum(valid)

            # Helper for safe mean
            def safe_mean(depths):
                return np.mean(depths) if len(depths) > 0 else 0.0

            left_min = safe_mean(left_depths)
            center_min = safe_mean(center_depths)
            right_min = safe_mean(right_depths)

            return left_min, center_min, right_min, num_points

        except Exception as e:
            self.get_logger().error(f"Error in filter_depthimage_sectors: {e}")
            # return float('inf'), float('inf'), float('inf'), 0
            return None, None, None, 0


    #
    # LIDAR sector filtering for version 2
    #
    def filter_lidar_sectors(self, lidar_msg):
        """
        Filters LIDAR data into 3 sectors: left, center, right.
        Returns the minimum valid distance in each sector (in meters).
        """
        ranges = np.array(lidar_msg.ranges)
        angles = np.linspace(lidar_msg.angle_min, lidar_msg.angle_max, len(lidar_msg.ranges))
        # Convert angles to degrees
        angles_deg = np.degrees(angles)

        # Valid mask: distance > min_measure_distance and not inf/NaN
        valid = (ranges > self.lidar_min_measure_distance) & (~np.isnan(ranges)) & (~np.isinf(ranges))

        # Adjust for LIDAR viewing range if nessary
        # The current value is setup for RPLIDAR S2
        # lidar_min_measure_distance', 0.05
        # lidar_sector_left_min', -170.0,   lidar_sector_left_max', -150.0
        # lidar_sector_center_min1', -180.0 lidar_sector_center_max1', -170.0
        # lidar_sector_center_min2', 170.0, lidar_sector_center_max2', 180.0
        # lidar_sector_right_min', 150.0,   lidar_sector_right_max', 170.0

        # Sector masks
        # Combine the valid boolean mask with angle range conditions for each sector using element-wise logical & AND
        # Left: -170° to -150°
        left_mask = valid & (angles_deg >= self.lidar_sector_left_min) & (angles_deg <= self.lidar_sector_left_max)
        # Center: -180° to -170° and 170° to 180°
        center_mask = valid & (
            ((angles_deg >= self.lidar_sector_center_min1) & (angles_deg <= self.lidar_sector_center_max1)) |
            ((angles_deg >= self.lidar_sector_center_min2) & (angles_deg <= self.lidar_sector_center_max2))
        )
        # Right: 150° to 170°
        right_mask = valid & (angles_deg >= self.lidar_sector_right_min) & (angles_deg <= self.lidar_sector_right_max)

        # Extract distances for each sector
        left_distances = ranges[left_mask]
        center_distances = ranges[center_mask]
        right_distances = ranges[right_mask]

        # Helper for safe min
        def safe_min(distances):
            return np.min(distances) if len(distances) > 0 else 0.0

        left_min = safe_min(left_distances)
        center_min = safe_min(center_distances)
        right_min = safe_min(right_distances)

        return left_min, center_min, right_min

    #
    # Combine depth vision and LIDAR data for movement decision
    #
    def fusion_move_decision(self, d_left, d_center, d_right, l_left, l_center, l_right):

        # NEW: If depth vision is invalid (None), immediately use LIDAR
        if d_left is None or d_center is None or d_right is None:
            self.get_logger().info("Depth vision invalid, falling back to LIDAR")
            return self.depthvision_move_decision(l_left, l_center, l_right, 'Lidar')

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
            return self.depthvision_move_decision(l_left, l_center, l_right, 'Lidar')
        else:
            # Step 2: If Vision is clear (min > 0.6m), use PointCloud2 decision
            return self.depthvision_move_decision(d_left, d_center, d_right, 'Vision')

    #
    # Movement logic for depth vision only (version 1)
    #
    def depthvision_move_decision(self, left_min, center_min, right_min, sensor_source="Vision"):
        # Set min threshold based on sensor
        if sensor_source.lower() == "lidar":
            # 25cm for LIDAR
            min_valid_distance = self.min_valid_distance_lidar
        else:
            # 60cm for depth camera
            min_valid_distance = self.min_valid_distance_vision

        # Movement logic for version 1 (depth vision only) or 2 (depth vision + LIDAR)
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
            # No forward movement default to turning right
            twist.linear.x = 0.0
            twist.angular.z = -self.twist_z
            self.get_logger().info(f"[{sensor_source}]: All sectors blocked (L={left_min:.2f}, R={right_min:.2f}): Default rotating RIGHT to unblock")
        else:
            # Default forward movement 0.1 m/s
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
                    # Too close on left or blind zone, slowing down and turning right
                    # twist.linear.x = 0.0 if blind_zone else self.speed_gain - self.speed_up
                    twist.linear.x = self.speed_gain - self.speed_up
                    twist.angular.z = -self.twist_z
                    self.get_logger().info(f"[{sensor_source}]: Obstacle very close on LEFT or blind zone: Turning RIGHT")
                else:
                    # Path clear and obstacle on left use default speed gain move forward
                    twist.angular.z = 0.0
                    self.get_logger().info(f"[{sensor_source}]: Obstacle closest on LEFT: Moving FORWARD")
            elif min_sector == 'right':
                if center_min < self.movement_threshold or blind_zone:
                    # Too close on right or blind zone, slowing down and turning left
                    # twist.linear.x = 0.0 if blind_zone else self.speed_gain - self.speed_up
                    twist.linear.x = self.speed_gain - self.speed_up
                    twist.angular.z = self.twist_z
                    self.get_logger().info(f"[{sensor_source}]: Obstacle very close on RIGHT or blind zone: Turning LEFT")
                else:
                    # Path clear and obstacle on right use default speed gain move forward
                    twist.angular.z = 0.0
                    self.get_logger().info(f"[{sensor_source}]: Obstacle closest on RIGHT: Moving FORWARD")
            else:
                twist.angular.z = 0.0
                self.get_logger().warn(f"[{sensor_source}]: Obstacle unknown: Moving FORWARD")

        #self.pub_twist.publish(twist)
        return twist


def main(args=None):
    rclpy.init(args=args)

    global executor
    executor = rclpy.executors.MultiThreadedExecutor()
    node = DepthVisionAvoidance()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print('\ncontrol-c: depth_vision_avoidance shutting down')
        pass
    finally:
        # Destroy the node explicitly - don't depend on garbage collector
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()