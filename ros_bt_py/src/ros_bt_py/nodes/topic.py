from threading import Lock
import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'topic_type': type,
             'topic_name': str},
    inputs={},
    outputs={'message': OptionRef('topic_type')},
    max_children=0))
class Topic(Leaf):
    def _do_setup(self):
        self._lock = Lock()
        self._msg = None
        self._subscriber = rospy.Subscriber(self.options['topic_name'],
                                            self.options['topic_type'],
                                            self._callback)
        return NodeMsg.IDLE

    def _callback(self, msg):
        with self._lock:
            self._msg = msg

    def _do_tick(self):
        with self._lock:
            if self._msg is None:
                return NodeMsg.RUNNING
            self.outputs['message'] = self._msg
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        # If we initialized the node, unsubscribe from the topic. Otherwise,
        # there's nothing to do here.
        if self.state != NodeMsg.UNINITIALIZED:
            self._subscriber.unregister()

    def _do_reset(self):
        # discard the last received message, nothing else to reset
        self._msg = None
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
