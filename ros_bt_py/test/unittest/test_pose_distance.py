#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#  -------- END LICENSE BLOCK --------
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

    def testPoseDistanceStaleData(self):
        self.pose_distance = PoseDistance({
            'succeed_on_stale_data': True
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

        self.assertEqual(self.pose_distance.tick(), NodeMsg.SUCCEEDED)

        self.assertEqual(self.pose_distance.untick(), NodeMsg.IDLE)
        self.assertEqual(self.pose_distance.reset(), NodeMsg.IDLE)
        self.assertEqual(self.pose_distance.shutdown(), NodeMsg.SHUTDOWN)
