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
import torch

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, SetParametersResult
from ros2param.api import call_get_parameters
from std_msgs.msg import String
from nano_llm import NanoLLM, ChatHistory

from nano_llm.plugins import VideoSource
from nano_llm.utils import cuda_image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from jetson_utils import (
    cudaMemcpy, cudaEventRecord, 
    cudaStreamCreate, cudaStreamSynchronize, cudaToNumpy,
)


from jetson_utils import cudaMemcpy, cudaToNumpy

from ..include.node_parameter_utility import NodeParamTools

class RosVideoSource(Node):
    """
    Captures or loads a video/camera stream or sequence of images
    """
    def __init__(self, video_input: str='/camera/image_raw', 
                 video_input_width: int=640, video_input_height: int=480, 
                 video_input_codec: str=None, video_input_framerate: float=None, 
                 video_input_save: str=None, loops: int=None, num_buffers: int=None, 
                 return_copy: bool=True, return_tensors: str='cuda', 
                 start: bool=False, **kwargs):
        """
        Creates a video input stream from MIPI CSI or V4L2 camera, RTP/RTSP/WebRTC stream, or video file (MP4, MKV, AVI, FLV)
        """
        super().__init__('video_source_node')

        self.lock = threading.Lock()  # Initialize the lock

        self.event = threading.Event()
        self.collect_image_data = 0
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            video_input,
            self.image_callback,
            10)

        self.start = start
        self.get_logger().info('VideoSource start = {}'.format(self.start))

        self.cuda_stream = kwargs.get('cuda_stream')

        if self.cuda_stream is None:
            self.cuda_stream = cudaStreamCreate(nonblocking=True)

        self.return_copy = return_copy
        self.return_tensors = return_tensors
        self.time_last = time.perf_counter()
        self.framerate = 0
        self.image = None

    def image_callback(self, msg):
        """
        Callback function to handle incoming ROS2 image messages
        """
        if self.start:
            try:
                if self.collect_image_data > 0:
                    self.collect_image_data += 1
                    self.get_logger().info('Capture image count:{}'.format(self.collect_image_data))
                cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
                # Acquire the lock before modifying self.image
                with self.lock:
                    self.image = cuda_image(cv_image)
            except Exception as e:
                self.get_logger().info(f"Failed to convert image: {e}")
            finally:
                # Deley signal client to process image
                if self.collect_image_data > 1:
                        self.get_logger().info('Capture image update complete')
                        self.collect_image_data = 0
                        self.event.set()


    def capture(self, timeout=3.0, cuda_stream=None, return_copy=None, return_tensors=None, **kwargs):
        """
        Capture images from the video source as long as it's streaming
        """
        if return_copy is None:
            return_copy = self.return_copy

        if return_tensors is None:
            return_tensors = self.return_tensors

        if cuda_stream is None:
            cuda_stream = self.cuda_stream

        image = None
        if self.start == False:
            self.start = True
            self.event.clear()
            self.collect_image_data = 1
            self.get_logger().info('Wait for image for {} seconds'.format(timeout))
            if self.event.wait(timeout):
                self.get_logger().info('Ready fo capture images')
            else:
                self.start = False
                self.get_logger().info('Wait for image 3 seconds timeout')
                return image

        with self.lock:  # Acquire the lock before accessing self.image
            if self.image is None:
                self.get_logger().info(f"Waiting for image failed")
                return None
            image = self.image
            shape = image.shape
            # Turn of screen capture to prevent 
            # CUDA error: operation failed due to a previous error during capture
            self.start = False

        if return_copy:
            image = cudaMemcpy(image, stream=cuda_stream)
        
        if cuda_stream:
            image.event = cudaEventRecord(stream=cuda_stream)
            image.stream = cuda_stream
            
        if return_tensors == 'pt':
            image = torch.as_tensor(image, device='cuda')
        elif return_tensors == 'np':
            image = cudaToNumpy(image)
            cudaStreamSynchronize(cuda_stream)
        elif return_tensors != 'cuda':
            raise ValueError(f"return_tensors should be 'np', 'pt', or 'cuda' (was '{return_tensors}')")

        curr_time = time.perf_counter()
        
        self.framerate = self.framerate * 0.9 + (1.0 / (curr_time - self.time_last)) * 0.1
        self.time_last = curr_time

        # Log image properties for debugging
        self.get_logger().info('Image shape: {}'.format(image.shape))
        self.get_logger().info('Image type: {}'.format(type(image)))
        # Log the summary
        self.get_logger().info('Summary: {}x{}, {:.1f} FPS'.format(shape[1], shape[0], self.framerate))

        return image

    def run(self):
        """
        Run capture continuously and attempt to handle disconnections
        """
        rclpy.spin(self)

    @property
    def streaming(self):
        """
        Returns true if the stream is currently open, false if closed or EOS.
        """
        return self.image is not None
     
    @property
    def eos(self):
        """
        Returns true if the stream is currently closed (EOS has been reached)
        """
        return not self.streaming
       
    @classmethod
    def type_hints(cls):
        """
        Return static metadata about the plugin settings.
        """
        return dict(
            video_input = dict(display_name='Input'),
            video_input_width = dict(display_name='Width'),
            video_input_height = dict(display_name='Height'),
            video_input_codec = dict(display_name='Codec'),
            video_input_framerate = dict(display_name='Framerate'),
            video_input_save = dict(display_name='Save'),
       )


class llm_vision_description(Node):

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
        super().__init__('llm_vision_description')

        # Detect parameters
        # meta-llama/Llama-2-7b-chat-hf  meta-llama/Meta-Llama-3-8B-Instruct
        self.llm_model = self.declare_parameter('model', 'Efficient-Large-Model/VILA1.5-3b').get_parameter_value().string_value
        self.quantization = self.declare_parameter('quantization', 'q4f16_ft').get_parameter_value().string_value
        self.max_context_len = self.declare_parameter('max-context-len', 256).get_parameter_value().integer_value
        self.max_tokens = self.declare_parameter('max-new-tokens', 50).get_parameter_value().integer_value
        self.llm_chat = self.declare_parameter('llm_chat', True).get_parameter_value().bool_value
        self.llm_input_topic = self.declare_parameter('llm_input', '/llm_vision_input').get_parameter_value().string_value
        self.llm_output_topic = self.declare_parameter('llm_output', '/chatbot/response').get_parameter_value().string_value
        self.video_input = self.declare_parameter('video_input', '/video_source/raw').get_parameter_value().string_value

        # Display settings informatio
        self.get_logger().info('model           : {}'.format(self.llm_model))
        self.get_logger().info('quantization    : {}'.format(self.quantization))
        self.get_logger().info('max context len : {}'.format(self.max_context_len))
        self.get_logger().info('max new tokens  : {}'.format(self.max_tokens))
        self.get_logger().info('llm_chat start  : {}'.format(self.llm_chat))
        self.get_logger().info('llm input topic : {}'.format(self.llm_input_topic))
        self.get_logger().info('llm output topic: {}'.format(self.llm_output_topic))
        self.get_logger().info('voide_input     : {}'.format(self.video_input))

        self.mutex = threading.Lock()
        self.model = None
        self.chat_history = None
        self.llm_ready = False


        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # self.init_ros_nodes()
        self.node_param_util = NodeParamTools(self, executor)

        # create RosVideoSource()
        self.video_source = RosVideoSource(video_input=self.video_input)
        executor.add_node(self.video_source)


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
        self.get_logger().info('RosVideoSource().destroy_node()')
        self.video_source.destroy_node()
        pass

    #
    # NanoLLM load model 
    #
    def load_model(self):
        self.get_logger().info('NanoLLM load model: {} please wait')
        self.model = NanoLLM.from_pretrained(
            model=self.llm_model, 
            quantization=self.quantization,
            max_context_len=self.max_context_len,
            api='mlc',
            vision_api='auto',
            vision_model= None,
            vision_scaling= None
        )
        self.chat_history = ChatHistory(self.model, system_prompt="You are a helpful and friendly AI assistant.")
        # Introduce a 3-second delay to ensure CUDA objects are initialized
        # self.get_logger().info('Introduce a {}-second delay to ensure CUDA objects are initialized'.format(3))
        # time.sleep(3)
        self.llm_ready = True
        self.get_logger().info('Model {} loaded successfully'.format(self.llm_model))

    def is_llm_ready(self):
        return self.llm_ready
    
    def llm_input_callback(self, msg):
        self.get_logger().info('LLM input:{}'.format(msg.data.strip()))
        
        if self.is_llm_ready():
            prompt = msg.data.strip()
            self.get_logger().debug(f'[jetbot]>> {prompt}')

            img = self.video_source.capture()
            if img is None:
                self.get_logger().info('Video source failed to caputre ROS2 image')
                return
            else:
                self.chat_history.append('user', image=img)

            # Add user prompt and generate chat tokens/embeddings
            self.chat_history.append('user', prompt)
            embedding, position = self.chat_history.embed_chat()

            self.get_logger().info('LLM.model.generate please wait...')

            # Generate bot reply
            reply = self.model.generate(
                embedding, 
                kv_cache=self.chat_history.kv_cache,
                max_new_tokens=self.max_tokens,  # Adjust as needed
                min_new_tokens=-1,
                #do_ample
                repetition_penalty=1.0,
                temperature=0.7,
                top_p=0.95
            )

            response = ""
            for token in reply:
                response += token

            # Log the raw response for debugging
            self.get_logger().info(f'Raw response: {response}')

            # Remove special strings and unexpected Unicode characters
            response = re.sub(r'</?s>', '', response)
            response = re.sub(r'[^\x00-\x7F]+', '', response)  # Remove non-ASCII characters

            # Save the final output
            self.chat_history.append('bot', response)
            self.get_logger().info(f'<<jetbot>>: {response}')

            # Publish the response as a ROS2 message
            response_msg = String()
            response_msg.data = response
            self.llm_publication.publish(response_msg)

            # Reset the chat history
            # TODO: Improve the reset chathistory as needed instead of resetting every time
            self.chat_history.reset()
        
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

    llm_vision_node = llm_vision_description()
    executor.add_node(llm_vision_node)

    try:
        # rclpy.spin(parameter)
        executor.spin()
    except KeyboardInterrupt:
        print('\ncontrol-c: follow_copilot_node_node shutting down')
    finally:
        # Destroy the node explictly - don't depend on garbage collector
        llm_vision_node.cleanup()
        llm_vision_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()