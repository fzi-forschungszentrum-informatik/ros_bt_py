from threading import Lock
import rospy
from roslib.message import get_message_class

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'topic_type': type,
             'topic_name': str},
    inputs={},
    outputs={'message': OptionRef('topic_type')},
    max_children=0))
class TopicSubscriber(Leaf):
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
        # Unsubscribe from the topic so we don't receive further updates
        self._subscriber.unregister()

    def _do_reset(self):
        # discard the last received message, nothing else to reset
        self._msg = None
        self._subscriber.unregister()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        resolved_topic = rospy.resolve_name(self.options['topic_name'])

        for topic, topic_type_name in rospy.get_published_topics():
            topic_type = get_message_class(topic_type_name)
            if (topic == resolved_topic and
                    topic_type == self.options['topic_type']):
                # if the topic we want exists, our lower bound for
                # success is 0. We don't know the upper bound, but
                # that still compares favorably to the bounds we
                # report if the topic is missing (none)
                return UtilityBounds(
                    has_lower_bound_success=True,
                    lower_bound_success=0.0,
                    has_lower_bound_failure=True,
                    lower_bound_failure=0.0)
        return UtilityBounds()


@define_bt_node(NodeConfig(
    options={'topic_type': type,
             'topic_name': str},
    inputs={'message': OptionRef('topic_type')},
    outputs={},
    max_children=0))
class TopicPublisher(Leaf):
    def _do_setup(self):
        self._publisher = rospy.Publisher(self.options['topic_name'],
                                          self.options['topic_type'],
                                          latch=True,
                                          queue_size=1)
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.inputs['message'] is None:
            self.logwarn('Trying to publish to topic %s with no message set' %
                         self.options['topic_name'])
            return NodeMsg.FAILED

        # Only publish a new message if our input data has been updated - the
        # old one is latched anyway.
        if self.inputs.is_updated('message'):
            self._publisher.publish(self.inputs['message'])
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        # Unregister the publisher
        self._publisher.unregister()
        self._publisher = None

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
