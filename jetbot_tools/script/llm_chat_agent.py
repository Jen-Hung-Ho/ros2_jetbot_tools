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
# Reference
# This code is inspired by the example from dusty-nv NanoLLM: 
# https://github.com/dusty-nv/NanoLLM/blob/main/nano_llm/chat/example.py

import cv2
import rclpy
import numpy as np
import threading
import time
import re

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, SetParametersResult
from ros2param.api import call_get_parameters
from std_msgs.msg import String
from nano_llm import NanoLLM, ChatHistory

from ..include.node_parameter_utility import NodeParamTools

class llm_text_chat(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'llm_chat' and param.type_ == Parameter.Type.BOOL:
                self.follow_detect = param.value
                self.get_logger().info('llm_chat= {}'.format(bool(param.value)))
            elif param.name == 'stop_count' and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info('stop count= {}'.format(str(param.value)))
                self.tolerance = param.value
        return SetParametersResult(successful=True)
    
    def __init__(self):
        super().__init__('llm_text_chat')

        # Detect parameters
        # meta-llama/Llama-2-7b-chat-hf  meta-llama/Meta-Llama-3-8B-Instruct
        self.llm_model = self.declare_parameter('model', 'meta-llama/Llama-2-7b-chat-hf').get_parameter_value().string_value
        self.quantization = self.declare_parameter('quantization', 'q4f16_ft').get_parameter_value().string_value
        self.max_tokens = self.declare_parameter('max-new-tokens', 256).get_parameter_value().integer_value
        self.llm_chat = self.declare_parameter('llm_chat', True).get_parameter_value().bool_value
        self.llm_input_topic = self.declare_parameter('llm_input', '/jetbot_llm_input').get_parameter_value().string_value
        self.llm_output_topic = self.declare_parameter('llm_output', '/chatbot/response').get_parameter_value().string_value

        # Display settings informatio
        self.get_logger().info('model          : {}'.format(self.llm_model))
        self.get_logger().info('quantization   : {}'.format(self.quantization))
        self.get_logger().info('max new tokens : {}'.format(self.max_tokens))
        self.get_logger().info('llm_chat start : {}'.format(self.llm_chat))
        self.get_logger().info('llm input topic :{}'.format(self.llm_input_topic))
        self.get_logger().info('llm output topic:{}'.format(self.llm_output_topic))

        self.mutex = threading.Lock()
        self.model = None
        self.chat_history = None
        self.llm_ready = False


        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # self.init_ros_nodes()
        self.node_param_util = NodeParamTools(self, executor)

        # Create the subscriber. This subscriber will receive an Image
        # from the detectnet overlay video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            String, 
            self.llm_input_topic, 
            self.llm_input_callback, 
            10)

        self.llm_publication = self.create_publisher(
            String, self.llm_output_topic, 10)
        
        # Need to implement multi thread nodes
        self.model_loading_thread = threading.Thread(target=self.load_model)
        self.model_loading_thread.start()

        self.thread = threading.Thread(target=self.process_llm_chat , daemon=True)
        self.thread.start()

    #
    # Remove nodes for get/set parameter service call
    #
    def cleanup(self):
        self.node_param_util.cleanup()
        pass

    #
    # NanoLLM load model 
    #
    def load_model(self):
        self.get_logger().info('NanoLLM load model: {} please wait')
        self.model = NanoLLM.from_pretrained(
            model=self.llm_model, 
            quantization=self.quantization, 
            api='mlc'
        )

        self.chat_history = ChatHistory(self.model, system_prompt="You are a helpful and friendly AI assistant.")
        self.llm_ready = True
        self.get_logger().info('Model: {} vidion:{} loaded successfully'.format(self.llm_model, self.model.has_vision))

    def is_llm_ready(self):
        return self.llm_ready
    
    def llm_input_callback(self, msg):
        self.get_logger().info('LLM input:{}'.format(msg.data.strip()))
        
        if self.is_llm_ready():
            prompt = msg.data.strip()
            self.get_logger().debug(f'[jetbot]>> {prompt}')

            # Add user prompt and generate chat tokens/embeddings
            self.chat_history.append('user', prompt)
            embedding, position = self.chat_history.embed_chat()

            # Generate bot reply
            reply = self.model.generate(
                embedding, 
                streaming=True, 
                kv_cache=self.chat_history.kv_cache,
                stop_tokens=self.chat_history.template.stop,
                max_new_tokens=self.max_tokens,  # Adjust as needed
            )

            response = ""
            for token in reply:
                response += token

            # Remove special strings
            response = re.sub(r'</?s>', '', response)

            # Save the final output
            self.chat_history.append('bot', response)
            self.get_logger().info(f'<<jetbot>>: {response}')

            # Publish the response as a ROS2 message
            response_msg = String()
            response_msg.data = response
            self.llm_publication.publish(response_msg)
        
        else:
            self.get_logger().info('Nano load mode is not ready! please wait')


    def process_llm_chat(self):
        self.get_logger().info('process_llm_chat')

        while rclpy.utilities.ok():

            time.sleep(0.1)



def main(args=None):
    rclpy.init(args=args)
    
    global executor
    executor = rclpy.executors.MultiThreadedExecutor()

    llm_chat_node = llm_text_chat()
    executor.add_node(llm_chat_node)

    try:
        # rclpy.spin(parameter)
        executor.spin()
    except KeyboardInterrupt:
        print('\ncontrol-c: follow_copilot_node_node shutting down')
    finally:
        # Destroy the node explictly - don't depend on garbage collector
        llm_chat_node.cleanup()
        llm_chat_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()