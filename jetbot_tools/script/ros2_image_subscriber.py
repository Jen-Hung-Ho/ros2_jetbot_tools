# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import cv2 # OpenCV library
import rclpy # Python library for ROS 2
# import ros2param
from rclpy.node import Node # Handles the creation of nodes
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from ros2param.api import call_get_parameters 
from sensor_msgs.msg import Image # Image is the message type
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
# import cv2 # OpenCV library

from ..include.object_tracker import * 

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')

    self.input_topic = self.declare_parameter('sub_topic','video_source/raw').get_parameter_value().string_value
    self.pub_topic = self.declare_parameter('pub_topic', 'image_raw').get_parameter_value().string_value
    self.scale_percent = self.declare_parameter('scale_percent', 25).get_parameter_value().integer_value
    
    self.get_logger().info('subscribe topic: {}'.format(self.input_topic))
    self.get_logger().info('publish topic: {}'.format(self.pub_topic))
    self.get_logger().info('scale percent: {}'.format(self.scale_percent))

    self.class_label_names = []

    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      self.input_topic, 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Create the publisher, This publish wil will publish an Image
    # to the image_row topic, The queue size is 10 messages
    self.image_pub = self.create_publisher(Image, self.pub_topic, 10)
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
     
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    scale = self.scale_percent
    width = int(current_frame.shape[1] * scale / 100)
    height = int(current_frame.shape[0] * scale / 100)
    dim = (width, height)
    resize_frame = cv2.resize(current_frame, dim, interpolation = cv2.INTER_AREA)
    # self.get_logger().info("Resized Diminsions:{}".format(resize_frame.shape))

    # Add central grid line
    resize_frame = draw_frame(resize_frame)
    
    # Display image
    # cv2.imshow("camera", current_frame)
    # cv2.imshow("camera", resize_frame)

    try:
      self.image_pub.publish(self.br.cv2_to_imgmsg(resize_frame, "bgr8"))
    except CvBridgeError as e:
      self.get_logger().error(e)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)

  global executor
  executor = rclpy.executors.MultiThreadedExecutor()
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  executor.add_node(image_subscriber)

  # Spin the node so the callback function is called.
  # rclpy.spin(image_subscriber)

  executor.spin()  
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()

  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
