#!/usr/bin/env python3
#
# Copyright (c) 2024, Jen-Hung Ho 
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

# https://github.com/opencv/opencv/issues/14884
# Importing scikit-learn related library first before importing any other libraries like opencv
from sklearn.cluster import KMeans

import rclpy  # Python library for ROS 2
import threading
import time
import asyncio
import ast    # Parse string into a 2D array

from rclpy.node import Node # Handles the creation of nodes
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, SetParametersResult
from vision_msgs.msg import Detection2DArray

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from collections import namedtuple
from math import radians, degrees
# import PyKDL

from jetbot_tools.include.lidar_utilities import LidarTools
from jetbot_tools.include.tf2_utilities import NamespaceTransformListener, get_tf2_namespace
from jetbot_tools.include.robot_acton_utility import RobotCommand, RobotActionTool
from jetbot_tools.include.node_parameter_utility import NodeParamTools
from jetbot_tools.include.detectnet_result_utility import DetectNetResult

#
# Jetbot tools voice command copiliot
#
class JetbotToolCopilot(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start' and param.type_ == Parameter.Type.BOOL:
                self.start = param.value
                self.get_logger().info('start= {}'.format(bool(param.value)))
            elif param.name == 'command' and param.type_ == Parameter.Type.STRING:
                self.cmd = param.value
                self.get_logger().info('command= {}'.format(str(param.value)))
                self.invoke_command(self.cmd)

        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('Jetbot_tool_voice_copilot')

        self.cmd = self.declare_parameter('command', 'jetbot').get_parameter_value().string_value
        self.start = self.declare_parameter('start', True).get_parameter_value().bool_value
        self.TTS_enable = self.declare_parameter('TTS_enable', False).get_parameter_value().bool_value
        self.ASR_node = self.declare_parameter('ASR_node', '/Riva_ASR_processor').get_parameter_value().string_value
        self.ASR_topic = self.declare_parameter('ASR_topic', '/voice/transcripts').get_parameter_value().string_value
        self.TTS_topic = self.declare_parameter('TTS_topic', '/chatbot/response').get_parameter_value().string_value
        self.command_nodes = self.declare_parameter('command_nodes', ["/laser_avoidance"]).get_parameter_value().string_array_value
        self.jetbot_commands = self.declare_parameter('jetbot_commands', "[['left','turn:90'],['right','turn:-90']]").get_parameter_value().string_value
        self.jetbot_tasks = self.declare_parameter('jetbot_tasks', "['follow','2:follow_detect']").get_parameter_value().string_value
        # jetbot client cmd
        self.QosReliability = self.declare_parameter('qos_reliability', False).get_parameter_value().bool_value
        self.laser_topic = self.declare_parameter('laser_topic', '/scan').get_parameter_value().string_value
        self.Angle = self.declare_parameter('angle', 30).get_parameter_value().integer_value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.global_ns= self.declare_parameter('global_ns', False).get_parameter_value().bool_value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').get_parameter_value().string_value
        self.angular = self.declare_parameter('angular', 0.2).get_parameter_value().double_value
        self.angular_tolerance = self.declare_parameter('angular_tolerance', 5.0).get_parameter_value().double_value
        self.linear_tolerance = self.declare_parameter('linear_tolerance', 0.03).get_parameter_value().double_value
        self.odom_angular_scale_correction = self.declare_parameter('odom_angular_scale_correction', 1.0).get_parameter_value().double_value
        self.odom_linear_scale_correction = self.declare_parameter('odom_linear_scale_correction', 1.0).get_parameter_value().double_value
        #  Jetson DNN inference ROS2 detectnet node
        self.node_name = self.declare_parameter('node_name', '/detectnet/detectnet').get_parameter_value().string_value
        self.class_labels = self.declare_parameter('class_labels', 'class_labels_3185375291238328062').get_parameter_value().string_value
        self.overlay_topic = self.declare_parameter('overlay_topic', '/detectnet/overlay').get_parameter_value().string_value
        self.detect_topic = self.declare_parameter('detect_topic', '/detectnet/detections').get_parameter_value().string_value
        self.FoV = self.declare_parameter('fov', 90.0).get_parameter_value().double_value
        self.score = self.declare_parameter('score', 0.50).get_parameter_value().double_value
        self.detect_timeout = self.declare_parameter('detect_timeout', 7).get_parameter_value().integer_value
        self.tracking_objects = self.declare_parameter('tracking_objects', ['person']).get_parameter_value().string_array_value

        # Build ASR command to robot action / task mappings
        self.build_ASR_to_action_mappings()

        # Print "Jetbot_tool_voice_copilot" node parameters
        self.get_logger().info('start           :{}'.format(self.start))
        self.get_logger().info('command         :{}'.format(self.cmd))
        self.get_logger().info('TTS_enable      :{}'.format(self.TTS_enable))
        self.get_logger().info('ASR_node        :{}'.format(self.ASR_node))
        self.get_logger().info('ASR_topic       :{}'.format(self.ASR_topic))
        self.get_logger().info('TTS_topic       :{}'.format(self.TTS_topic))
        self.get_logger().info('jetbot_commands :{}'.format(self.cmd_dict_array.keys()))
        self.get_logger().info('jetbot_tasks    :{}'.format(self.task_dict_array.keys()))
        self.get_logger().info('command_nodes   :{}'.format(self.command_nodes))
        # jetbot client cmd
        self.get_logger().info('cmd_vel_topic  :{}'.format(self.cmd_vel_topic))
        self.get_logger().info('laser_topic    :{}'.format(self.laser_topic))
        self.get_logger().info('angle          :{}'.format(self.Angle))
        self.get_logger().info('tf2_global broadcast- :{}'.format(self.global_ns))
        self.get_logger().info('odom_frame     :{}'.format(self.odom_frame))
        self.get_logger().info('base_frame     :{}'.format(self.base_frame))
        self.get_logger().info('QOS reliability:{}'.format(self.QosReliability))
        self.get_logger().info('angular        :{}'.format(self.angular))
        self.get_logger().info('angular_tolerance:{}'.format(self.angular_tolerance))
        self.get_logger().info('linear_tolerance :{}'.format(self.linear_tolerance))
        self.get_logger().info('odom_angular_scale_correction :{}'.format(self.odom_angular_scale_correction))
        self.get_logger().info('odom_linear_scale_correction  :{}'.format(self.odom_linear_scale_correction))
        self.get_logger().info('node name       :{}'.format(self.node_name))
        self.get_logger().info('overlay topic   :{}'.format(self.overlay_topic))
        self.get_logger().info('detect_topic    :{}'.format(self.detect_topic))
        self.get_logger().info('class labels    :{}'.format(self.class_labels))
        self.get_logger().info('FoV             :{}'.format(self.FoV))
        self.get_logger().info('score           :{}'.format(self.score))
        self.get_logger().info('detect timeout  :{}'.format(self.detect_timeout))
        self.get_logger().info('tracking_objects:{}'.format(self.tracking_objects))

        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # self.init_ros_nodes()
        self.node_param_util = NodeParamTools(self, executor)
        self.mutex = threading.Lock()
        # self.class_label_names = []
        self.detect_net_util = DetectNetResult(self, self.node_param_util, self.node_name, self.class_labels, self.FoV)

        # Initialize the tf2 listener
        self.tf_buffer = Buffer()
        if not self.global_ns:
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            namespace = get_tf2_namespace(self.base_frame)
            self.get_logger().info('namespace : {}'.format(namespace))
            self.tf_listener = NamespaceTransformListener(namespace, self.tf_buffer, self)

        self.lidarlib = LidarTools(self.get_logger())
        self.collect_lidar_data = 0
        self.event = threading.Event()
        self.collect_detectnet_data = 0
        self.detect_target = False

        qos_profile = QoSProfile(depth=10)
        if self.QosReliability:
            qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        else:
            qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        # Create the subscriber. This subscriber will receive ASR message
        self.subscription = self.create_subscription(
            String, 
            self.ASR_topic,
            self.ASR_callback,
            10)

        # Create the publiser to TTS
        self.pub_TTS = self.create_publisher(
            String,
            self.TTS_topic,
            10
        )

        # Create the subscriber. This subscriber will receive lidar message
        self.subscription = self.create_subscription(
            LaserScan, 
            self.laser_topic,
            self.laser_callback,
            qos_profile)

        self.pub_twist = self.create_publisher(
            Twist, self.cmd_vel_topic, 10)

        # Create the subscriber. This subscriber will receive an Image
        # from the detectnet overlay video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, 
            self.overlay_topic, 
            self.overlay_callback, 
            10)
        
        self.subscription = self.create_subscription(
            Detection2DArray,
            self.detect_topic,
            self.detection_callback,
            10)

        self.robot_action = RobotActionTool(
            self.get_logger(), 
            self.lidarlib, 
            self.pub_twist, 
            self.odom_angular_scale_correction, 
            self.odom_linear_scale_correction)

        # Creating a single event loop in a background thread and have it service all async needs.
        self.async_loop = asyncio.new_event_loop()
        threading.Thread(target=self.async_loop.run_forever).start()

        # Need to implement multi thread node 
        self.thread = threading.Thread(target=self.command_callback, args=(self.async_loop,), daemon=True)
        self.thread.start()


    #
    # Build ASR command to robot action / task mappings
    #
    def build_ASR_to_action_mappings(self):
        # Parse the string into a 2D array
        self.cmd_two_dim_array = ast.literal_eval(self.jetbot_commands)
        self.cmd_dict_array = {row[0]: row[1] for row in self.cmd_two_dim_array}
        self.task_two_dim_array = ast.literal_eval(self.jetbot_tasks)
        self.task_dict_array = {row[0]: row[1] for row in self.task_two_dim_array}

        # define the command parameter
        self.stop = False
        self.rotate = False
        self.move = False
        # RobotCommand = namedtuple('command', ['done', 'turn_angle', 'distance', 'delta'])

        # define the task parameter
        self.follow = False
        self.self_driving = False
        robot_task = namedtuple('task', ['run', 'node_index', 'command'])

        # define command dictionary
        turn_status = RobotCommand(False, 90, 0.0, self.angular_tolerance)
        move_status = RobotCommand(False,  0, 0.5, self.linear_tolerance)
        stop_status = RobotCommand(False,  0, 0.0, 0.0)
        self_driving_status = robot_task(False, 0, 'start')
        follow_status       = robot_task(False, 2, 'follow_detect')
        self.command_dict = {
            'turn':turn_status,
            'move':move_status,
            'stop':stop_status,
            'self-driving':self_driving_status,
            'follow':follow_status
        }


    #
    # Remove nodes for get/set parameter service call
    #
    def cleanup(self):
        # Stop the async event loop
        self.async_loop.call_soon_threadsafe(self.async_loop.stop)

        self.node_param_util.cleanup()
        pass


    #
    # NVIDIA jetson-voice ASR ROS2 topic subscription
    #
    def ASR_callback(self, msg):
        self.get_logger().info('ASR: "%s"' % msg.data)


    #
    # Mute ASR processor and wait until LLM chat reponse to TTS task complete
    #
    def mute_ASR_processor(self, node_name):
        # Turn off ASR and wait until LLM chat response to TTS task complete
        passfail, value = self.node_param_util.try_get_node_parameters(node_name, 'start')
        if passfail == True:
            self.get_logger().info('Jetbot chat node start:{}'.format(value.bool_value))
            if value.bool_value:
                self.node_param_util.try_set_node_parameters(node_name, 'start', type=ParameterType.PARAMETER_BOOL, value=False)
        else:
            self.get_logger().info('Jetbot chat {} node not exit, skip this task'.format(node_name))

    #
    # Invoke from ROS2 node parameter_callback()
    #
    def invoke_command(self, cmd):
        self.get_logger().info('Cmd: {}'.format(cmd))
        TTS_echo = False

        # start == false --> ignore jetbot ASR client request
        if not self.start:
            self.get_logger().info("invoke_command() ignore command:{}".format(cmd))
            return

        # update self.cmd after done with this command
        # if cmd in self.jetbot_commands:
        if cmd in self.cmd_dict_array.keys():

            if self.TTS_enable:
                TTS_echo = True

            # Clean up any running task before proceed any new command
            self.reset_running_tasks(cmd)

            # Retrieve node name and command value [command:value]
            parts = self.cmd_dict_array[cmd].split(':')
            self.get_logger().info("jetbot-commands:{} --> {}".format(parts[0], parts[1]))
            if parts[0] == 'turn':
                rotate_angle = int(parts[1])
                self.command_dict[parts[0]] = self.command_dict[parts[0]]._replace(done=False)
                self.command_dict[parts[0]] = self.command_dict[parts[0]]._replace(turn_angle=rotate_angle)
                self.command_dict[parts[0]] = self.command_dict[parts[0]]._replace(distance=0.0)
                self.rotate = True
            elif parts[0] == 'move':
                move_distance = float(parts[1])
                self.command_dict[parts[0]] = self.command_dict[parts[0]]._replace(done=False)
                self.command_dict[parts[0]] = self.command_dict[parts[0]]._replace(turn_angle=0)
                self.command_dict[parts[0]] = self.command_dict[parts[0]]._replace(distance=move_distance)
                self.move = True
                self.get_logger().info("move distance :{}".format(move_distance))
            elif parts[0] == 'stop':
                movie_distance = 0;
                self.command_dict[parts[0]] = self.command_dict[parts[0]]._replace(done=False)
                self.command_dict[parts[0]] = self.command_dict[parts[0]]._replace(turn_angle=0)
                self.command_dict[parts[0]] = self.command_dict[parts[0]]._replace(distance=0.0)
                self.stop = True
                self.get_logger().info("stop robot")

        elif cmd in self.task_dict_array.keys():

            if self.TTS_enable:
                TTS_echo = True

            # Clean up any running task before proceed any new task
            self.reset_running_tasks(cmd)

            # retruieve node index and parameter name [node_index:parameter_name]
            parts = self.task_dict_array[cmd].split(':')
            node_index = int(parts[0])
            self.get_logger().info("jetbot_tasks: {} --> {}".format(self.command_nodes[node_index], parts[1]))
            if cmd == 'follow':
                self.command_dict[cmd] = self.command_dict[cmd]._replace(run=True)
                self.command_dict[cmd] = self.command_dict[cmd]._replace(node_index=node_index)
                self.command_dict[cmd] = self.command_dict[cmd]._replace(command=parts[1])
                self.follow = True
            elif cmd == 'self-driving':
                self.command_dict[cmd] = self.command_dict[cmd]._replace(run=True)
                self.command_dict[cmd] = self.command_dict[cmd]._replace(node_index=node_index)
                self.command_dict[cmd] = self.command_dict[cmd]._replace(command=parts[1])
                self.self_driving = True

        if TTS_echo == True:
            # ASR_client node mute ASR_node before invokde the command parameter
            # Publish to TTS node to play audio streaming
            TTS_string = String()
            TTS_string.data = "Jetbot process: " + cmd
            self.pub_TTS.publish(TTS_string)


    #
    # Clean up running task before start a new one
    #
    def reset_running_tasks(self, command):
        self.robot_action.cancel_action()
        self.reset_robot_running_task('self-driving', command=command)
        self.reset_robot_running_task('follow', command=command)

    #
    # KMeans version laser subscription callback
    #
    def laser_callback(self, msg):

        # Only collect data when client need find out obatacle near by
        if self.collect_lidar_data > 0:
            self.get_logger().info("collect lidar data count: {}".format(self.collect_lidar_data))
            self.lidarlib.collect_lidar_raw_data(msg)
            # Collect lidar sample at 30 degrees
            self.lidarlib.collect_lidar_data(self.Angle, msg)
            self.collect_lidar_data = self.collect_lidar_data -1
            if self.collect_lidar_data == 0:
                self.event.set()
                self.get_logger().info('collect_lidar_data complete --> DONE')
            pass

    #
    # Retrieve image center point from /detectnet/detectnet node overlay output
    #
    def overlay_callback(self, data):

        if self.collect_detectnet_data > 0:
            self.detect_net_util.overlay_image(data)

    #
    # detectnet node detections subscription call_back
    #
    def detection_callback(self, msg):

        # Turn on detection data colletion only when robot follow is turned on
        if self.collect_detectnet_data > 0:
            (passfail, label, score, angle) = self.detect_net_util.target_in_detections(self.tracking_objects, self.score, msg)
            if passfail:
                self.collect_detectnet_data += 1
                self.get_logger().info('Detect:{}, score:{} angle:{} count:{}'.format(label, score, angle, self.collect_detectnet_data))
                if self.collect_detectnet_data > 3:
                    self.get_logger().info("Detect chasing target cancel rotate robot")
                    self.detect_target = True
                    self.collect_detectnet_data = 0
                    self.event.set()


    #
    # thread: command_callback trigger by parameter invoke_command()
    #
    def command_callback(self, loop):
        self.get_logger().info('command_callback()')
        asyncio.set_event_loop(loop)
        self.get_logger().info('asyncio.set_event_loop()')

        while rclpy.utilities.ok():

            # Synchronous command call, 
            # It is assumed that the execution time for all commands is sufficiently short.
            # Please note that we do not support the cancellation of currently running command calls.
            # Task run is used for long running job such as robot_self_driving(), robot_follow()
            # A new voice command will reset the currently running task in the method InvokeCommand(cmd). 
            # - This is done through the method ResetRunningTasks(cmd)

            reset_command_value = False
            if self.rotate == True and self.command_dict['turn'].done == False:
                # turn robot clockwise/counterclockwise
                self.robot_action.rotate_robot(self.tf_buffer, self.odom_frame, self.base_frame, self.command_dict['turn'])
                self.rotate = False
                self.command_dict['turn'] = self.command_dict['turn']._replace(done=True)
                reset_command_value = True
            elif self.move == True and self.command_dict['move'].done == False:
                # move robot forward/backward
                self.robot_action.move_robot(self.tf_buffer, self.odom_frame, self.base_frame, self.command_dict['move'])
                self.move = False
                self.command_dict['move'] = self.command_dict['move']._replace(done=True)
                reset_command_value = True
            elif self.stop == True and self.command_dict['stop'].done == False:
                # stop the robot
                self.stop = False
                self.command_dict['stop'] = self.command_dict['stop']._replace(done=True)
                reset_command_value = True

            elif self.self_driving == True:
                self.robot_self_driving()
                self.command_dict['self-driving'] = self.command_dict['self-driving']._replace(run=True)
                self.self_driving = False
                reset_command_value = True
            elif self.follow == True:
                self.robot_follow()
                self.command_dict['follow'] = self.command_dict['follow']._replace(run=True)
                self.follow = False
                reset_command_value = True

            # Echoing' in ASR occurs
            # when the microphone picks up the system's own text-to-speech output, 
            # creating a recursive voice recognition loop.
            # Reset command value for Jetbot_ASR_client send same voice command
            if reset_command_value:
                # Set the parameter value
                self.set_parameters([rclpy.parameter.Parameter('command', rclpy.parameter.Parameter.Type.STRING, 'jetbot')])

            time.sleep(0.5)

    #
    # This method is thread-safe and is intended to be called from a different thread than the one where the event loop is running.
    #
    def thread_safe_async_method(self, coroutine):
        loop = asyncio.get_event_loop()
        # Schedules a coroutine to be run in the event loop.
        task = asyncio.run_coroutine_threadsafe(coroutine, loop)
        # Returns a concurrent.futures.Future object representing the execution of the coroutine.
        return task

    #
    # robot follow task -> Jetbot ROS2 follow_copilot node
    #
    def robot_follow(self):
        self.get_logger().info("robot_follow")
        pass

        follow_status = self.command_dict['follow']
        self.get_logger().info("Robot follow status:{}".format(follow_status))

        # robot follow detection procedure
        # 1) Ensure the follow copilot node exist
        # 2) Stop laser_avoidance and follow_copilot node if is is in running state
        # 3) Turn on the detectnet detection data collecting flag to look fo target vision
        # 4) Rotate the robot until the tracking target is in sight
        # 5) Stop rotate robot
        # 6) Start follow_copilot node to start fobot follow task

        # Step 1) Ensure the follow_copilot ROS2 node exist
        # ros2 param get /follow_copilot follow_detect -- false
        node_index = follow_status.node_index
        command = follow_status.command
        passfail, value = self.node_param_util.try_get_node_parameters(self.command_nodes[node_index], command)
        if passfail == False:
            self.mute_ASR_processor(self.ASR_node)
            TTS_string = String()
            TTS_string.data = "follow_copilot node not exist, skip the task"
            self.pub_TTS.publish(TTS_string)
            self.get_logger().info(TTS_string.data)
            return

        # Step 3) Turn on the detectnet detection data collecting flag to look fo target vision
        self.event.clear()
        self.collect_detectnet_data = 1
        self.detect_target = False

        # Step 4) Rotate the robot until the tracking target is in sight
        turn_status = RobotCommand(False, 360, 0.0, self.angular_tolerance)
        task = self.thread_safe_async_method(self.robot_action.async_rotate_robot(self.tf_buffer, self.odom_frame, self.base_frame, turn_status))
        time.sleep(0.5)

        # Step 5) Stop rotate robot tracking target is in sight
        self.get_logger().info('Rotate robot until target in sight')
        if not self.event.wait(timeout=self.detect_timeout):
            self.get_logger().error('ERROR: Failed to detect detectnet message, cancal rotate to find target')

        TTS_string = String()
        if self.detect_target:
            self.get_logger().info('Detect tracing target in sight')
            self.collect_detectnet_data = 0
            self.robot_action.cancel_action()
            time.sleep(0.5)
            # loop.run_until_complete(asyncio.sleep(1))
            if not task.done():
                self.get_logger().info("task.done() is false cancel the rotate robot action")
                task.cancel()

            TTS_string.data = "Detect person reardy to start following task"
            self.get_logger().info("xxxxxxx {} xxxxxxx".format(TTS_string.data))
        elif (self.command_dict['follow'].run == False):
            TTS_string.data = "Cancel robot follow task"
        else:
            TTS_string.data = "TIMEOUT: Failed to detect detectnet message"
            self.get_logger().error("{}".format(TTS_string.data))
        self.mute_ASR_processor(self.ASR_node)
        self.pub_TTS.publish(TTS_string)

        # Step 6) Start follow_copilot node to start fobot follow task
        # ROS2 param set /follow_copilot follow_detect true
        if (self.command_dict['follow'].run == True):
            self.node_param_util.set_node_parameters(self.command_nodes[node_index], command, type=ParameterType.PARAMETER_BOOL, value=True)


    #
    # self driving task -> JetBot ROS2 laser_avoidance node
    #
    def robot_self_driving(self):
        self.get_logger().info("robot_self_driving")

        self_driving_status = self.command_dict['self-driving']
        self.get_logger().info("Self-driving status:{}".format(self_driving_status))

        # self driving procedure
        # 1) Ensure the laser_avoidance ROS2 node exist
        # 2) Stop the laser_avoidance node if it is in running state
        # 3) Trun on lader data collecting flag to find out the open area 
        # 4) Roate the robot to face open area
        # 5) Start laser_avoidance node to start self driving task

        # Step 1) Ensure the laser_avoidance ROS2 node exist
        # ros2 param get /laser_avoidance start -- false
        node_index = self_driving_status.node_index
        command = self_driving_status.command
        passfail, value = self.node_param_util.try_get_node_parameters(self.command_nodes[node_index], command)
        if passfail == False:
            self.get_logger().info("laster avoidance node not exist, skip the task")
            return

        if self.rotate != True:
            # Step 3) Trun on lader data collecting flag to find out the open area
            self.event.clear()
            self.collect_lidar_data = 2
            # Wait for the lidar data is ready
            if self.event.wait(timeout=5):
                lidar_raw_data = self.lidarlib.get_lidar_raw_data()
                lidar_samples = self.lidarlib.get_lidar_samples()
                # Find out the open area  
                (distance, angle) = self.lidarlib.max_lidar_distance(lidar_samples, degrees(lidar_raw_data.angle_min), degrees(lidar_raw_data.angle_max), debug=False)
                # (distance, angle) = self.lidarlib.front_lidar_distance_2(lidar_raw_data)
                self.get_logger().info("min:{} max:{}".format(degrees(lidar_raw_data.angle_min), degrees(lidar_raw_data.angle_max)))
                self.get_logger().info("distance:{} angle:{}".format(distance, angle))

                # Step 4) Roate the robot to face open area 
                rotate_angle = angle
                self.command_dict['turn'] = self.command_dict['turn']._replace(done=False)
                self.command_dict['turn'] = self.command_dict['turn']._replace(turn_angle=rotate_angle)
                self.command_dict['turn'] = self.command_dict['turn']._replace(distance=0.0)
                # turn robot clockwise/counterclockwise
                self.robot_action.rotate_robot(self.tf_buffer, self.odom_frame, self.base_frame, self.command_dict['turn'])
            else:
                self.get_logger().error('ERROR: Failed to detect lidar signal, cancal rotate')

        # Step 5) Start laser_avoidance node to start self driving task
        # ROS2 param set /laser_avoidance start true
        if (self.command_dict['self-driving'].run == True):
            self.node_param_util.set_node_parameters(self.command_nodes[node_index], command, type=ParameterType.PARAMETER_BOOL, value=True)


    #
    # Reset robot long running task by set parameter value to false
    #
    def reset_robot_running_task(self, task_key='self-driving', command='self-driving'):

        self.get_logger().info("Reset_running task: {} by command:{}".format(task_key, command))

        # Update robot_task run = False
        if (self.command_dict[task_key].run == True):
            self.command_dict[task_key] = self.command_dict[task_key]._replace(run=False)

        # Signal the event to unblock any waiting task such as 'follow'
        if command == 'follow':
            self.collect_detectnet_data = 0
        elif command == 'self-driving':
            self.collect_lidar_data == 0
        self.event.set()

        # Get running task ROS2 node parameter setting value
        robot_task_status = self.command_dict[task_key]
        node_index = robot_task_status.node_index
        command = robot_task_status.command
        passfail, value = self.node_param_util.try_get_node_parameters(self.command_nodes[node_index], command)

        # If ROS2 node parameter setting not exist skip reset the task
        if passfail == False:
            self.get_logger().info("{} node not exist, skip the task".format(self.command_nodes[node_index]))
            return passfail

        # Stop the task - ROS2 node if it is in running state
        self.get_logger().info('{}: parameter start status: {}'.format(self.command_nodes[node_index],value.bool_value))
        # If task ROS2 node command parameter is true, stop the node running status
        if value.bool_value == True:
            self.node_param_util.set_node_parameters(self.command_nodes[node_index], command, type=ParameterType.PARAMETER_BOOL, value=False)

        return passfail


def main(args=None):
    
    rclpy.init(args=args)

    global executor

    executor = rclpy.executors.MultiThreadedExecutor()
    JetbotToolCopilit_node = JetbotToolCopilot()
    executor.add_node(JetbotToolCopilit_node)

    try:
        # rclpy.spin(JetbotToolCopilit_node)
        executor.spin()
    except KeyboardInterrupt:
        print('\ncontrol-c: JetbotToolCopilit_node shutting down')
    finally:
        # Destroy the node explictly - don't depend on garbage collector
        JetbotToolCopilit_node.cleanup()
        JetbotToolCopilit_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()