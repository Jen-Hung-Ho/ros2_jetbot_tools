#!/usr/bin/env python3

import rclpy
import numpy as np
import warnings
import threading
import copy
from sensor_msgs.msg import LaserScan

from math import radians, copysign, pi, degrees
import PyKDL

from sklearn.cluster import KMeans
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

#
# This class provides simple lidar functionalities to a ROS robot 
#
class LidarTools():

    def __init__(self, logger):

        self.logger = logger
        self.mutex = threading.Lock()
        self.sonar_raw_data = LaserScan()
        self.sonar_samples = []
        self.Angle = 30

        logger.info('LidarTools() initialize : collect lidar sample at [{}] degrees'.format(self.Angle))


    #
    # Filter out sonar range data
    #
    def get_sonar_range_data(self, ranges, min_value, max_value):
        ranges = np.array(ranges)
        # ranges[ranges == 0.0] = min_value
        ranges[ranges == 0.0] = max_value
        ranges[np.isinf(ranges)] = max_value
        # Debug here
        # ranges = np.clip(ranges, min_value, max_value, out=None)
        ranges = ranges.reshape(-1, 1)

        return ranges

    #
    # Form sonar data find the oblstacle distance
    #
    def get_sonar_data_distance(self, kmeans, angle, data, max_value):
        min_value = max_value

        try:
            sample_size = len(data)
            # suppress kmeans.fit distinct culters less than 3 warning
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                kmeans_model = kmeans.fit(data)
            # collect the fit array of cluster labels
            labels = kmeans_model.labels_
            unique, counts = np.unique(labels, return_counts=True)
            largest_cluster = unique[np.argmax(counts)]
            # largest_cluster_count = np.max(counts)
            largest_cluster_values = data[labels == largest_cluster]
            min_value = np.average(largest_cluster_values)
            self.logger.debug('K-- angle:{} min:{} len:{} raw:{}'.format(angle, min_value, len(data), data))

        except ValueError as e:
            self.logger.info('get sonar data error:{}'.format(e))
            self.logger.info('data:{}'.format(data))
        finally:
            if min_value == max_value:
                str_angle = '%.2f' % angle
                self.logger.debug('K--- lidar angle:{} min:{} data:{} '.format(str_angle, min_value, data))

        return min_value

    #
    # Get robot angle from TF2 transform
    #
    def get_odom_angle(self, tf_buffer, odom_frame, base_frame):

        try:
            # Get the current transform from the odom to base frames
            t = tf_buffer.lookup_transform(
                odom_frame, 
                base_frame, 
                rclpy.time.Time())
            self.logger.debug('{}'.format(t.transform.rotation))
        except TransformException as ex:
            # self.logger.info('TF2 transform exception')
            self.logger.info(f'TF2 transform exception {self.base_frame} to {self.odam_frame}: {ex}')
            return 0.0

        # Convert the rotation from a quaternion to an Euler angle
        return self.quat_to_angle(t.transform.rotation)

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self, angle):
        res = angle
        # > 180  -> -360
        while res > pi:
            res -= 2.0 * pi
        # < -180 -> + 360
        while res < -pi:
            res += 2.0 * pi
        return res

    #
    # Deep copy LaserScan Data
    #
    def collect_lidar_raw_data(self, msg):
        with self.mutex:
            self.sonar_raw_data = copy.deepcopy(msg)

    #
    # KMeans version laser subscription callback
    #
    def collect_lidar_data(self, degree, msg):
        sonar_sample_list = []

        # data len
        length = len(msg.ranges)

        # Regions 180 / 6 = 30 
        self.Angle = degree

        # half angle of range data
        Index = int((self.Angle/2)*(length/360))

        # Start from angle_min ~ -180
        sonar_samples = []
        range1 = msg.ranges[0:Index]
        range2 = msg.ranges[(length - Index):]
        sonar_samples = list(range1) + list(range2)
        sample_size = len(sonar_samples)
        if sample_size > 3:
            sample_size = 3
        kmeans = KMeans(n_clusters = sample_size, n_init='auto')
        k_sonar_samples = self.get_sonar_range_data(sonar_samples, msg.range_min, msg.range_max)
        
        # lidar angle - -180 to 180  or 0 to 360  
        angle = degrees(msg.angle_min)
        min_value = self.get_sonar_data_distance(kmeans, angle, k_sonar_samples, msg.range_max)
        sonar_sample_list.append([min_value, angle])
        
        # Increment angle to next range 30
        angle += self.Angle
        IndexSize = int(self.Angle*(length/360))
        # collect rest of 11 regions - increment by 30 degree
        for i in range(1, int(360/self.Angle)):
            # self.logger.info('i:{}, Index:{}'.format(i, Index))
            range1 = msg.ranges[int(Index):int(Index+IndexSize)]
            sonar_samples = list(range1)
            sample_size = len(sonar_samples)
            if sample_size > 3:
                sample_size = 3
            kmeans = KMeans(n_clusters = sample_size, n_init='auto')
            k_sonar_samples = self.get_sonar_range_data(sonar_samples, msg.range_min, msg.range_max)
            min_value = self.get_sonar_data_distance(kmeans, angle, k_sonar_samples, msg.range_max)
            sonar_sample_list.append([min_value, angle])
            # Update range index and angle
            angle += self.Angle
            Index += IndexSize

            # self.logger.info('lidar index:{} angle:{} distance:[{}]'.format(i, angle, min_value))

        # Lock the code 
        # The mutex is automatically release when the with bolock is exited
        with self.mutex: 
            self.sonar_samples = []
            self.sonar_samples = sonar_sample_list.copy()

        # self.logger.info('lidar smple:{}'.format(self.sonar_samples))
    

    #
    # Find out the max distacne between min to max angle
    #
    def max_lidar_distance(self, lidar_samples, min_angle, max_angle, debug=False):
        # choose between min - max angle
        filter_180 = lidar_samples[(lidar_samples[:,1] > min_angle) & (lidar_samples[:,1] < max_angle)]
        # find the index of maximum elemt in the first column
        max_index = np.argmax(filter_180[:,0])
        # return the whol row values
        max_row = filter_180[max_index,:]

        if debug:
            with np.printoptions(precision=3, suppress=True):
                self.logger.info('sample min-max:[{}]-[{}] max-row:{} raw-data:{}'.format(min_angle, max_angle, max_row, filter_180))


        return max_row
    
    #
    # get lidar samples in numpy array
    #
    def get_lidar_samples(self):
        # Lock the code
        with self.mutex:
            lidar_samples = np.array(self.sonar_samples)

        return lidar_samples

    #
    # get lidar raw data
    #
    def get_lidar_raw_data(self):
        # Lock the code
        with self.mutex:
            lidar_data = copy.deepcopy(self.sonar_raw_data)

        return lidar_data
    
    #
    # Find out front distance 
    #
    def front_lidar_distance(self, lidar_samples):
        # Find the front obstacle distance
        array_length = lidar_samples.shape

        # Assume the min to max range from -180 to 180
        index = int(array_length[0]/2)
        (distance, angle) = lidar_samples[index]

        return distance, angle, index

    #
    # Find out front distance v2
    #
    def front_lidar_distance_2(self, lidar_raw_data):
        # Find out the lidar front angle 
        angle = degrees(lidar_raw_data.angle_min + ((lidar_raw_data.angle_max - lidar_raw_data.angle_min) / 2.0))
        distance = self.get_distance_lidar_raw_data(angle, 10.0, lidar_raw_data)

        return distance, angle

    #
    # get distance at angle 
    #
    def get_distance_from_lidar_data(self, angle, tolerance):
        # Retrieve LaserScan
        lidar_raw_data = self.get_lidar_raw_data()

        return self.get_distance_lidar_raw_data(angle, tolerance, lidar_raw_data)

    #
    # get distance at angle from lidar raw data
    #
    def get_distance_lidar_raw_data(self, angle, tolerance, lidar_raw_data):
        # data length
        length = len(lidar_raw_data.ranges)

        w_range = degrees(lidar_raw_data.angle_max - lidar_raw_data.angle_min)
        s_range = (angle - tolerance) - degrees(lidar_raw_data.angle_min)
        # increment = length / degrees(lidar_raw_data.angle_max - lidar_raw_data.angle_min)
        start_index = int((s_range/w_range) * length)
        tolerance_index = int(((tolerance*2.0)/w_range) * length)
        ranges = lidar_raw_data.ranges[start_index:start_index+tolerance_index+1]
        sonar_ranges = list(ranges)
        sample_size = len(ranges)
        if sample_size > 3:
            sample_size = 3
        kmeans = KMeans(n_clusters = sample_size, n_init='auto')
        k_sonar_samples = self.get_sonar_range_data(sonar_ranges, lidar_raw_data.range_min, lidar_raw_data.range_max)
        min_value = self.get_sonar_data_distance(kmeans, angle, k_sonar_samples, lidar_raw_data.range_max)
        
        return min_value
        