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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

class CompressedImageRepublisher(Node):

    def __init__(self):
        super().__init__('compressed_image_republisher')

        # Declare parameters for flexibility
        self.declare_parameter('input_topic', '/camera/depth/raw/compressedDepth')
        self.declare_parameter('output_topic', '/camera/depth/in/compressedDepth')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # QoS profile: BEST_EFFORT for streaming over WiFi
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self.listener_callback,
            qos_profile
        )

        # Publisher
        self.publisher = self.create_publisher(
            CompressedImage,
            output_topic,
            10
        )

        self.get_logger().info(f"Republishing from {input_topic} → {output_topic} with QoS BEST_EFFORT")

    def listener_callback(self, msg: CompressedImage):
        # Immediately republish the incoming message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\ncontrol-c: CompressedImage republisher_node shutting down')
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
