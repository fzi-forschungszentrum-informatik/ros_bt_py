#!/usr/bin/env python3
# Copyright 2018-2023 FZI Forschungszentrum Informatik
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
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
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

"""
ROS Node running the MissionControl class.
"""

# pylint: disable=import-error, no-name-in-module

import rospy
import std_msgs
from ros_bt_py_msgs.srv import GetCapabilityInterfaces

from ros_bt_py.capability import (
    capability_node_class_from_capability_interface_callback,
)
from ros_bt_py.mission_control import MissionControl

if __name__ == "__main__":
    local_repository_prefix = rospy.get_param(
        "~local_repository_prefix", default="/capability_repository_node"
    )

    local_mc_prefix = f"{rospy.get_namespace()}/mission_control"

    get_capability_interfaces_topic = rospy.resolve_name(
        f"{rospy.get_namespace()}/capability_repository/capabilities/interfaces/get"
    )

    get_capability_interface_service = rospy.ServiceProxy(
        name=get_capability_interfaces_topic,
        service_class=GetCapabilityInterfaces,
        persistent=True,
    )

    interfaces_announcement_topic = rospy.resolve_name(
        f"{rospy.get_namespace()}/capability_repository/capabilities/interfaces"
    )

    capability_interface_subscription = rospy.Subscriber(
        name=interfaces_announcement_topic,
        data_class=std_msgs.msg.Time,
        callback=capability_node_class_from_capability_interface_callback,
        callback_args=(
            local_mc_prefix,
            get_capability_interface_service,
        ),
    )

    rospy.init_node("mission_control")

    MISSION_CONTROL = MissionControl()
    while not rospy.is_shutdown():
        rospy.rostime.wallsleep(0.5)

    MISSION_CONTROL.shutdown()
