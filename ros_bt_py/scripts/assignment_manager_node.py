#!/usr/bin/env python3

# Copyright 2023-2023 FZI Forschungszentrum Informatik
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

"""Module containing a ros node for the AssignmentManager class."""
from typing import Type

import rospy

from ros_bt_py.assignment_manager.assignment_manager import (
    AssignmentManager,
    SimpleAssignmentManager,
)
from ros_bt_py.assignment_manager.parallel_auction_manager import ParallelAuctionManager


class AssignmentManagerNode:
    """
    Node to run a AssignmentManager class implementation.

    This sets up the rospy services and maps them to the
    respective methods in a AssignmentManager.
    """

    def __init__(
        self,
        local_topic_prefix: str,
        global_assignment_msg_topic_prefix: str,
        assignment_manager: Type[AssignmentManager],
    ):
        """
        Create a new AssignmentManagerNode.

        This method will directly setup rospy services.

        :param local_topic_prefix: The ros topic prefix used to communicate with
        local services like the mission control on the same robot.
        :param global_assignment_msg_topic_prefix: The ros topic used for communicating with
        other AssignmentManagers for example in an auction.
        :param assignment_manager: The assignment manager class implementation to use.
        """
        self.assignment_manager = assignment_manager(
            local_topic_prefix=local_topic_prefix,
            global_assignment_msg_topic_prefix=global_assignment_msg_topic_prefix,
        )


if __name__ == "__main__":
    rospy.init_node("assignment_manager")
    try:
        LOCAL_TOPIC_PREFIX: str = rospy.get_namespace()
        ASSIGNMENT_TOPIC_PREFIX: str = rospy.get_param(
            "assignment_topic_prefix", default="/assignments"
        )

    except rospy.ROSException as exc:
        rospy.logerr(f"Failed to retrieve parameters from parameter server: {exc}")
        raise exc
    ASSIGNMENT_MANAGER_NODE = AssignmentManagerNode(
        local_topic_prefix=LOCAL_TOPIC_PREFIX,
        global_assignment_msg_topic_prefix=ASSIGNMENT_TOPIC_PREFIX,
        assignment_manager=ParallelAuctionManager,
    )
    rospy.spin()
