#  -------- BEGIN LICENSE BLOCK --------
#  Copyright 2022 FZI Forschungszentrum Informatik
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#     * Neither the name of the {copyright_holder} nor the names of its
#        contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#  -------- END LICENSE BLOCK --------
from math import sqrt
from typing import Optional, List, Dict

import rospy
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from ros_bt_py_msgs.msg import UtilityBounds
from rospy import ServiceProxy, Subscriber

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.action import Action
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose

def eucledian_distance(pose1: Pose, pose2: Pose) -> float:
    return sqrt(
        pow((pose1.position.x - pose2.position.x), 2) +
        pow((pose1.position.y - pose2.position.y), 2)

    )

@define_bt_node(
    NodeConfig(
        inputs={},
        options={
            "amcl_pose_topic": str
        },
        outputs={},
        max_children=0,
        version="1.0.0"
    )
)
class MoveBaseAction(Action):
    def __init__(self,
                 options: Optional[Dict] = None,
                 debug_manager: Optional[DebugManager] = None,
                 name: str = None,
                 succeed_always: bool = False,
                 simulate_tick: bool = False
                 ):
        super(MoveBaseAction, self).__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick
        )
        self.__amcl_sub: Optional[Subscriber] = None
        self.__amcl_pose: PoseWithCovarianceStamped = PoseWithCovarianceStamped()
        self._amcl_pose_updated = False

    def _amcl_sub_callback(self, pose: PoseWithCovarianceStamped):
        self.__amcl_pose = pose
        self._amcl_pose_updated = True

    def _do_setup(self):
        amcl_topic = rospy.resolve_name(self.options["amcl_pose_topic"])
        self.__amcl_sub = Subscriber(
            amcl_topic,
            PoseWithCovarianceStamped,
            self._amcl_sub_callback,
            queue_size=1
        )
        return super(MoveBaseAction, self)._do_setup()

    def _do_calculate_utility(self):
        try:
            self._handle_inputs()
        except BehaviorTreeException:
            rospy.logerr("Failed to receive the input values")
            return UtilityBounds(
                can_execute=True,
                has_lower_bound_success=True,
                has_upper_bound_success=True,
                has_lower_bound_failure=True,
                has_upper_bound_failure=True
            )

        try:
            goal: MoveBaseGoal = self.inputs["goal"]
            goal_pose: PoseStamped = goal.target_pose
        except Exception as exc:
            rospy.logerr(f"Failed to get goal pose for utility value calc: {exc}!")
            return UtilityBounds(
                can_execute=True,
                has_lower_bound_success=True,
                has_upper_bound_success=True,
                has_lower_bound_failure=True,
                has_upper_bound_failure=True
            )

        tries = 0
        while not self._amcl_pose_updated:
            rospy.sleep(0.1)
            tries += 1
            if tries > 10:
                rospy.logerr(f"Failed to get amcl pose for utility value calc!")
                return UtilityBounds(
                    can_execute=True,
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failure=True,
                    has_upper_bound_failure=True
                )

        distance = eucledian_distance(self.__amcl_pose.pose.pose, goal_pose.pose)

        return UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            lower_bound_success=distance,
            has_upper_bound_success=True,
            upper_bound_success=distance * 1.2,
            has_lower_bound_failure=True,
            lower_bound_failure=0,
            has_upper_bound_failure=True,
            upper_bound_failure=distance
        )
@define_bt_node(
    NodeConfig(
        inputs={},
        options={},
        outputs={},
        max_children=0,
        version="1.0.0"
    )
)
class BebopMoveBaseAction(MoveBaseAction):
    def _do_calculate_utility(self):
        utilities = super(BebopMoveBaseAction, self)._do_calculate_utility()
        utilities.lower_bound_success *= 1.2
        utilities.upper_bound_success *= 1.2
        utilities.lower_bound_failure *= 1.2
        utilities.upper_bound_failure *= 1.2
        return utilities
