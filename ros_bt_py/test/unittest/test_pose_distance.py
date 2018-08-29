import jsonpickle
import unittest

from geometry_msgs.msg import Point, Pose, Quaternion
from tf import transformations

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.ros_nodes.pose_distance import PoseDistance


class testPoseDistance(unittest.TestCase):
    def testPoseDistance(self):
        self.pose_distance = PoseDistance({
            'succeed_on_stale_data': False
        })
        self.pose_distance.setup()

        a = Pose()
        b = Pose()
        a.position = Point(0, 0, 0)
        a.orientation = Quaternion(*transformations.quaternion_from_euler(0, 0, 0))
        b.position = Point(1, 0, 0)
        b.orientation = Quaternion(*transformations.quaternion_from_euler(0, 1.5, 0))

        # Unset inputs, should throw
        with self.assertRaises(ValueError):
            self.pose_distance.tick()

        self.pose_distance.inputs['pose_a'] = a
        self.pose_distance.inputs['pose_b'] = b

        self.assertEqual(self.pose_distance.tick(), NodeMsg.SUCCEEDED)
        self.assertAlmostEqual(self.pose_distance.outputs['distance_pos'], 1.0)
        self.assertAlmostEqual(self.pose_distance.outputs['distance_angle'], 1.5)

        # Should return RUNNING on stale data
        self.assertEqual(self.pose_distance.tick(), NodeMsg.RUNNING)
