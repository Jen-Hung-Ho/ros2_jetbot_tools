#!/usr/bin/env python3
#
# Copyright (c) 2023, Jen-Hung Ho 
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

import rclpy  # Python library for ROS 2
import threading
from rcl_interfaces.msg import ParameterType, Parameter, ParameterValue
from ros2param.api import call_get_parameters
from ros2param.api import call_set_parameters

from std_msgs.msg import String

class NodeParamTools():

    def __init__(self, node, executor):

        self.node = node
        self.logger = self.node.get_logger()
        self.executor = executor
        self.lock = threading.Lock()
        node_name = self.node.get_name()

        self.init_ros_nodes(node_name)
        self.logger.info('NodeParamTools({}) initialize'.format(node_name))

    #
    # Initialize nodes for get/set parameter service call
    #
    def init_ros_nodes(self, node_name):
        # To get/set parameter on another node in ROS2 using Python, use the SetParameters service.
        # client = node.create_client(GetParameters, f'{node_name}/get_parameters')
        # client = node.create_client(SetParameters, f'{node_name}/set_parameters')

        self.get_param_node = rclpy.create_node(node_name + '_get_param_node')
        self.set_param_node = rclpy.create_node(node_name + '_set_param_node')
        self.executor.add_node(self.get_param_node)
        self.executor.add_node(self.set_param_node)

        # self.node_param_util = NodeParamTools(self.get_logger())

    #
    # Remove nodes for get/set parameter service call
    #
    def cleanup(self):

        # clean up set_param_node, get_param_node
        self.executor.remove_node(self.set_param_node)
        self.set_param_node.destroy_node()
        self.executor.remove_node(self.get_param_node)
        self.get_param_node.destroy_node()
        pass

    #
    # Try catch version of get_node_parameters
    #
    def try_get_node_parameters(self, node_name, param):
        try:
            value = self.get_node_parameters(node_name, param)
            return (True, value)
        except RuntimeError as e:
            # try to catch node not exist with service timer out error
            self.logger.info("get node parameter error: {}".format(str(e)))
            return (False, None)

    #
    # Try catch version of set_node_parameters
    #
    def try_set_node_parameters(self, node_name, param_name, type, value):
        try:
            self.set_node_parameters(node_name, param_name, type, value)
            return True
        except RuntimeError as e:
            # try to catch node not exist with service timer out error
            self.logger.info("set node parameter error: {}".format(str(e)))
            return False

    #
    # To get a parameter on another node in ROS2 using Python, use the GetParameters service.
    # client = node.create_client(GetParameters, f'{node_name}/get_parameters')
    #
    def get_node_parameters(self, node_name, param):
        self.logger.info('get node parameters : {} - {}'.format(node_name, param))

        # Block the next get node parameter invoke until the current finishes
        # ros2 param get /Jetbot_Param_Client command
        with self.lock:
            parameters = [param]
            response = call_get_parameters(node=self.get_param_node, 
                                #node_name='/detectnet/detectnet', 
                                node_name=node_name,
                                parameter_names=parameters)

        # print(response.values)
        if len(response.values) >= 1:
            # txtract type specific value
            pvalue = response.values[0]
            if pvalue.type == ParameterType.PARAMETER_BOOL:
                print(pvalue.bool_value) 
                self.logger.info('get node bool value: {}'.format(pvalue.bool_value))
            elif pvalue.type == ParameterType.PARAMETER_STRING:
                print(pvalue.string_value)
                self.logger.info('get node string value: {}'.format(pvalue.string_value))
            elif pvalue.type == ParameterType.PARAMETER_STRING_ARRAY:
                self.logger.info('get node string array value: {}'.format(pvalue.string_array_value))

        return pvalue
    
    #
    # To set a parameter on another node in ROS2 using Python, use the SetParameters service.
    # client = node.create_client(SetParameters, f'{node_name}/set_parameters')
    #
    def set_node_parameters(self, node_name, param_name, type, value):
        self.logger.info('set node parameters : {} - {} - {}'.format(node_name, param_name, value))

        # Block the next set node parameter invoke until the current finishes
        with self.lock:
            param = Parameter()
            param.name = param_name
            if type == ParameterType.PARAMETER_STRING:
                param.value = ParameterValue(string_value=value, type=ParameterType.PARAMETER_STRING)
            elif type == ParameterType.PARAMETER_BOOL:
                param.value = ParameterValue(bool_value=value, type=ParameterType.PARAMETER_BOOL)
            elif type == ParameterType.PARAMETER_STRING_ARRAY:
                param.value = ParameterValue(string_array_value=value, type=ParameterType.PARAMETER_STRING_ARRAY)

            parameters = [param]
            response = call_set_parameters(node=self.set_param_node,
                                    node_name=node_name,
                                    parameters=parameters)

        if response is not None:
            # SetParametersResult
            for result in response.results:
                self.logger.debug('set node: {} parameter: {} value:{}'.format(node_name, param_name, value))
                self.logger.info('Parameter set successful: {}'.format(result.successful))
                self.logger.debug('Reason: {}'.format(result.reason))