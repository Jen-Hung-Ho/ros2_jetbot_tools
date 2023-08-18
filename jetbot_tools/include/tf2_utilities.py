#!/usr/bin/env python3

from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage

class NamespaceTransformLintener(TransformListener):
    def __init__(self, namespace, buffer, node, *, spin_thread=False, qos=None, static_qos=None):
        super().__init__(buffer, node, spin_thread=spin_thread)
        self._node = node

        # Unsubscribe to /tf /tf_statics
        super().unregister()

        if qos is None:
            qos = QoSProfile(
                depth=100,
                durability= DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )
        if static_qos is None:
            static_qos = QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                )

        if self._node.global_ns:
            self.tf_broacast = TransformBroadcaster(self._node)

        # self.group = super().group
        self.t_group = ReentrantCallbackGroup()
        self._subscription = self._node.create_subscription(
            TFMessage, f'{namespace}/tf', self._tf_callback,
            qos, callback_group=self.t_group)
        self.tf_static_sub = self._node.create_subscription(
            TFMessage, f'{namespace}/tf_static', self._tf_static_callback, 
            static_qos, callback_group=self.t_group)
    
    def _tf_callback(self, data):
        # pipe Transform message from namespace/tf to tf2 buffer
        # tf_buffer.lookup_transform() cable to lookup transform cross namespace
        super().callback(data)

        # broadcast Transform message to global /tf topic
        if self._node.global_ns:
            self.tf_broacast.sendTransform(data.transforms)

    def _tf_static_callback(self, data):
        super().static_callback(data)


#
# Get tf2 namespace  
#
def get_tf2_namespace(s):
    # Ex: 'jetbot1/base_footprint'
    namespace = s.split('/')[0]

    return namespace