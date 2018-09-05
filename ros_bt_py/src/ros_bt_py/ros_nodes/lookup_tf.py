import rospy
import tf2_ros

from geometry_msgs.msg import Pose, Point
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(NodeConfig(
    options={
        'parent_frame': str,
        'child_frame': str},
    inputs={},
    outputs={'transform_pose': Pose},
    max_children=0))
class LookupTFConst(Leaf):
    """Lookup the current tf between `parent_frame` and `child_frame`"""
    def _do_setup(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _do_tick(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.options['parent_frame'],
                                                    self.options['child_frame'],
                                                    rospy.Time())
            self.outputs['transform_pose'] = Pose(Point(trans.transform.translation.x,
                                                        trans.transform.translation.y,
                                                        trans.transform.translation.z),
                                                  trans.transform.rotation)
            return NodeMsg.SUCCEEDED
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return NodeMsg.FAILED

    def _do_shutdown(self):
        self.tf_listener.unregister()
        self.tf_buffer.clear()

    def _do_reset(self):
        self.tf_buffer.clear()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    options={'parent_frame': str},
    inputs={'child_frame': str},
    outputs={'transform_pose': Pose},
    max_children=0))
class LookupTF(Leaf):
    """Lookup the current tf between `parent_frame` and `child_frame`"""
    def _do_setup(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _do_tick(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.options['parent_frame'],
                                                    self.inputs['child_frame'],
                                                    rospy.Time())
            self.outputs['transform_pose'] = Pose(Point(trans.transform.translation.x,
                                                        trans.transform.translation.y,
                                                        trans.transform.translation.z),
                                                  trans.transform.rotation)
            return NodeMsg.SUCCEEDED
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return NodeMsg.FAILED

    def _do_shutdown(self):
        self.tf_listener.unregister()
        self.tf_buffer.clear()

    def _do_reset(self):
        self.tf_buffer.clear()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
