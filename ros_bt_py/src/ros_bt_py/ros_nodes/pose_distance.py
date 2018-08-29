import rospy

from tf import transformations
from geometry_msgs.msg import Pose
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'succeed_on_stale_data': bool},
    inputs={'pose_a': Pose,
            'pose_b': Pose},
    outputs={'distance_pos': float,
             'distance_angle': float},
    max_children=0))
class PoseDistance(Leaf):
    """Calculates the absolute distance between `pose_a` and `pose_b`

    `distance_angle` is the magnitude of the vector of the euler
    angles between `pose_a` and `pose_b`.

    The option parameter `succeed_on_stale_data` determines whether
    the node returns SUCCEEDED or RUNNING if the two inputs haven't been
    updated since the last tick.

    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.inputs.is_updated('pose_a') or self.inputs.is_updated('pose_b'):
            a = self.inputs['pose_a']
            b = self.inputs['pose_b']
            self.outputs['distance_pos'] = transformations.vector_norm(
                [a.position.x - b.position.x,
                 a.position.y - b.position.y,
                 a.position.z - b.position.z])
            # Angular is a bit more tricky

            # We need to calculate the difference between the two
            # quaternions, convert that to Euler representation, then
            # take the magnitude of that vector

            def q_to_array(q):
                return [q.x, q.y, q.z, q.w]
            # Difference between two quaternions is q2 * inv(q1)
            b_inv = transformations.quaternion_inverse(q_to_array(b.orientation))
            diff = transformations.quaternion_multiply(q_to_array(a.orientation), b_inv)
            euler = transformations.euler_from_quaternion(diff)
            self.outputs['distance_angle'] = transformations.vector_norm(euler)
            return NodeMsg.SUCCEEDED
        else:
            if self.options['succeed_on_stale_data']:
                return NodeMsg.SUCCEEDED
            else:
                self.loginfo('No new data since last tick!')
                return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs['item'] = None
        self.outputs.reset_updated()
        self.inputs['list'] = None
        self._do_setup()
        self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
