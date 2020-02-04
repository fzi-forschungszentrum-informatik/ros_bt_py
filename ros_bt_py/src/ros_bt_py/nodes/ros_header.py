import rospy

import std_msgs.msg

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

@define_bt_node(NodeConfig(
     options={'header_type': type},
     inputs={},
     outputs={'header':OptionRef('header_type')},
     max_children=0))
class GetStdHeader(Leaf):
     """Get ROS-Time in Header format"""
     def _do_setup(self):
         self.header = std_msgs.msg.Header()
         pass

     def _do_tick(self):
         self.header.stamp = rospy.Time.now()
         if isinstance(self.header, self.options['header_type']):
             self.outputs['header'] = self.header
             return NodeMsg.SUCCEEDED
         else:
             return NodeMsg.FAILED

     def _do_shutdown(self):
         pass

     def _do_reset(self):
         return NodeMsg.IDLE

     def _do_untick(self):
         return NodeMsg.IDLE

     def _do_calculate_utility(self):
         return UtilityBounds()
