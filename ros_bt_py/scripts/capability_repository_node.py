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

"""Module containing the node definition for the CapabilityRepository."""
# pylint: disable=import-error, no-name-in-module

import sys

import rospy

from std_msgs.msg import Time
from ros_bt_py_msgs.msg import CapabilityInterface
from ros_bt_py_msgs.srv import (
    LoadCapabilities,
    DeleteCapabilityImplementation,
    PutCapabilityImplementation,
    GetCapabilityImplementations,
    GetCapabilityInterfaces,
    PutCapabilityInterfaces,
    SaveCapabilities,
)

from ros_bt_py.capability_repository import CapabilityRepository


class CapabilityRepositoryNode:
    """The ros node running the CapabilityRepository class."""

    # pylint: disable=too-few-public-methods, too-many-instance-attributes

    def __init__(self):
        """
        Create a new capability repository node.

        The repository has to major interfaces, it communicates with other repositories and
        offers services geared towards its local subscribers.
        """
        # Topic prefix used for the global capability interface exchange.
        global_capability_topic_prefix = rospy.get_param(
            "~global_capability_topic_prefix", default="/gc"
        )

        self.__local_capability_interface_update_publisher = rospy.Publisher(
            "~capabilities/interfaces", Time, queue_size=1000
        )

        self.__local_capability_implementation_update_publisher = rospy.Publisher(
            "~capabilities/implementations", Time, queue_size=1000
        )

        # Global interfaces communication topics and services.
        self.__global_capability_interfaces_publisher = rospy.Publisher(
            f"{global_capability_topic_prefix}/interfaces",
            CapabilityInterface,
            queue_size=1000,
        )

        self.__global_capability_interfaces_requests_publisher = rospy.Publisher(
            f"{global_capability_topic_prefix}/interfaces/request",
            Time,
            latch=True,
            queue_size=1,
        )

        self.capability_repository = CapabilityRepository(
            publisher_interface_updates_locally=self.__local_capability_interface_update_publisher,
            publisher_implementation_updates_locally=(
                self.__local_capability_implementation_update_publisher
            ),
            publisher_interface_globally=self.__global_capability_interfaces_publisher,
            publisher_interface_request_globally=(
                self.__global_capability_interfaces_requests_publisher
            ),
        )

        # Local interfaces communication topics and services.
        self.__local_capabilities_load_service = rospy.Service(
            "~capabilities/load",
            LoadCapabilities,
            self.capability_repository.load_capabilities,
        )

        self.__local_capabilities_save_service = rospy.Service(
            "~capabilities/save",
            SaveCapabilities,
            self.capability_repository.save_capabilities,
        )

        self.__local_capability_interfaces_put_service = rospy.Service(
            "~capabilities/interfaces/put",
            PutCapabilityInterfaces,
            self.capability_repository.put_capability_interfaces,
        )

        self.__local_capability_interfaces_get_service = rospy.Service(
            "~capabilities/interfaces/get",
            GetCapabilityInterfaces,
            self.capability_repository.get_capability_interfaces,
        )

        # Local implementations services.
        self.__local_capability_implementations_get_service = rospy.Service(
            "~capabilities/implementations/get",
            GetCapabilityImplementations,
            self.capability_repository.get_capability_implementations,
        )

        self.__local_capability_implementations_put_service = rospy.Service(
            "~capabilities/implementations/put",
            PutCapabilityImplementation,
            self.capability_repository.put_capability_implementation,
        )

        self.__local_capability_implementations_delete_service = rospy.Service(
            "~capabilities/implementations/delete",
            DeleteCapabilityImplementation,
            self.capability_repository.delete_capability_implementation,
        )

        self.__global_capability_interfaces_subscriber = rospy.Subscriber(
            f"{global_capability_topic_prefix}/interfaces",
            CapabilityInterface,
            self.capability_repository.global_capability_interfaces_callback,
        )

        self.__global_capability_interfaces_requests_subscriber = rospy.Subscriber(
            f"{global_capability_topic_prefix}/interfaces/request",
            Time,
            self.capability_repository.global_capability_interfaces_request_callback,
        )

    def shutdown(self) -> None:
        """
        Shuts down the running node and disable all services and topics.

        :return: None
        """
        rospy.loginfo("Unregister publishers and subscribers!")

        self.__global_capability_interfaces_publisher.unregister()
        self.__global_capability_interfaces_subscriber.unregister()
        self.__global_capability_interfaces_requests_publisher.unregister()
        self.__global_capability_interfaces_requests_subscriber.unregister()

        rospy.loginfo("Shutting down local services and unregister publishers!")
        self.__local_capabilities_save_service.shutdown()
        self.__local_capabilities_load_service.shutdown()

        self.__local_capability_interfaces_get_service.shutdown()
        self.__local_capability_interfaces_put_service.shutdown()

        self.__local_capability_implementations_put_service.shutdown()
        self.__local_capability_implementations_get_service.shutdown()
        self.__local_capability_implementations_delete_service.shutdown()

        self.capability_repository.shutdown()


if __name__ == "__main__":
    rospy.init_node("capability_repository", argv=sys.argv)
    rospy.loginfo("Starting CapabilityRepository node")
    node = CapabilityRepositoryNode()
    rospy.spin()
    node.shutdown()
