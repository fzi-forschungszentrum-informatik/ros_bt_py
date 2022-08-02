"""
Module used to implement mission control components.
This includes the management of capabilities and their implementations as well as the forwarding to
other nodes by using an auction protocol.
"""
# pylint: disable=no-name-in-module,import-error
import os
import re
import shutil
import tempfile
from threading import RLock
from typing import Dict, Type, List

import actionlib
import genpy
import rospkg
import rospy
import yaml
from rospkg import ResourceNotFound
from std_msgs.msg import Time

from ros_bt_py.assignment_manager.assignment_manager import AssignmentManager
from ros_bt_py.helpers import HashableCapabilityInterface
from ros_bt_py_msgs.msg import (
    CapabilityInterface, CapabilityImplementation, FindBestCapabilityImplementationAction,
    FindBestCapabilityImplementationGoal, FindBestCapabilityImplementationResult,
)
from ros_bt_py_msgs.srv import (
    PutCapabilityInterfacesRequest, PutCapabilityInterfacesResponse,
    GetCapabilityInterfacesRequest, GetCapabilityInterfacesResponse,
    GetCapabilityImplementationsRequest, GetCapabilityImplementationsResponse, DeleteCapabilityImplementationResponse,
    DeleteCapabilityImplementationRequest, PutCapabilityImplementationRequest,
    PutCapabilityImplementationResponse, DeleteCapabilityImplementation,
    GetCapabilityInterfaces, PutCapabilityInterfaces,
    GetCapabilityImplementations, PutCapabilityImplementation, LoadCapabilities, LoadCapabilitiesRequest,
    LoadCapabilitiesResponse, SaveCapabilitiesRequest, SaveCapabilitiesResponse, SaveCapabilities,
)




class MissionControl:
    pass



class MissionControlNode:
    """
    Class to manage the capability implementations and interfaces on the local node.
    Additionally, it allows to exchange interface definitions with remote nodes.
    """

    # pylint: disable=too-many-instance-attributes

    def __init__(
            self,
            auction_manager: Type[AssignmentManager],
            local_capability_topic_prefix: str,
            global_capability_topic_prefix: str
    ):
        """
        Creates a new capability repository.
        The repository has to major interfaces, it communicates with other repositories and
        offers services geared towards its local subscribers.

        :param local_capability_topic_prefix: Prefix used for local communication.
        This topic only relays information relevant for the local robot, thus foreign implementations,
        etc. are not used.

        :param global_capability_topic_prefix: Prefix used for communication with other ros_bt_py nodes.
        This is used to sync available interfaces across all nodes.
        """

        self.local_capability_topic_prefix = local_capability_topic_prefix
        self.global_capability_topic_prefix = global_capability_topic_prefix


