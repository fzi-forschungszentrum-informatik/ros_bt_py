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
from typing import Dict

import genpy
import rospkg
import rospy
import yaml
from rospkg import ResourceNotFound
from std_msgs.msg import Time
from ros_bt_py.helpers import json_decode
from ros_bt_py_msgs.msg import CapabilityInterface, CapabilityImplementation
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


class HashableCapabilityInterface:
    """
    Wrapper class to allow for the hashing of capability interfaces.
    """

    def __init__(self, interface: CapabilityInterface):
        self.interface: CapabilityInterface = interface

    def __eq__(self, other: object) -> bool:

        def compare_node_data_lists(list1: list, list2: list) -> bool:
            l1_node_data = {(x.key, json_decode(x.serialized_type)) for x in list1}
            l2_node_data = {(x.key, json_decode(x.serialized_type)) for x in list2}

            return l1_node_data.isdisjoint(l2_node_data)

        if not isinstance(other, HashableCapabilityInterface):
            return False

        if not self.interface.name == other.interface.name:
            return False

        return (compare_node_data_lists(self.interface.inputs, other.interface.inputs) and
                compare_node_data_lists(self.interface.outputs, other.interface.outputs) and
                compare_node_data_lists(self.interface.options, other.interface.options))

    def __ne__(self, other: object) -> bool:
        return not self.__eq__(other)

    def __hash__(self) -> int:
        return hash(
            (
                self.interface.name,
                frozenset({(x.key, json_decode(x.serialized_type)) for x in self.interface.inputs}),
                frozenset({(x.key, json_decode(x.serialized_type)) for x in self.interface.outputs}),
                frozenset({(x.key, json_decode(x.serialized_type)) for x in self.interface.options})
            )
        )


class CapabilityRepository:
    """
    Class to manage the capability implementations and interfaces on the local node.
    Additionally, it allows to exchange interface definitions with remote nodes.
    """

    # pylint: disable=too-many-instance-attributes

    def __init__(
            self,
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

        # List containing all known capability interfaces.
        self.__capabilities_lock = RLock()
        self.capabilities: Dict[HashableCapabilityInterface, Dict[str, CapabilityImplementation]] = {}

        self.__rp = rospkg.RosPack()
        self.__capabilities_tmp_dir = tempfile.mkdtemp(prefix="capability_backup_")

        # Local interfaces communication topics and services.
        self.__local_capability_interface_publisher = rospy.Publisher(
            f'{local_capability_topic_prefix}/capabilities/interfaces',
            CapabilityInterface,
            queue_size=1000
        )

        self.__local_capabilities_load_service = rospy.Service(
            f'{local_capability_topic_prefix}/capabilities/load',
            LoadCapabilities,
            self.load_capabilities
        )

        self.__local_capabilities_save_service = rospy.Service(
            f'{local_capability_topic_prefix}/capabilities/save',
            SaveCapabilities,
            self.save_capabilities
        )

        self.__local_capability_interfaces_put_service = rospy.Service(
            f'{local_capability_topic_prefix}/capabilities/interfaces/put',
            PutCapabilityInterfaces,
            self.put_capability_interfaces
        )

        self.__local_capability_interfaces_get_service = rospy.Service(
            f'{local_capability_topic_prefix}/capabilities/interfaces/get',
            GetCapabilityInterfaces,
            self.get_capability_interfaces
        )

        # Local implementations services.

        self.__local_capability_implementation_publisher = rospy.Publisher(
            f'{local_capability_topic_prefix}/capabilities/implementations',
            CapabilityImplementation,
            queue_size=1000
        )

        self.__local_capability_implementations_get_service = rospy.Service(
            f'{local_capability_topic_prefix}/capabilities/implementations/get',
            GetCapabilityImplementations,
            self.get_capability_implementations
        )

        self.__local_capability_implementations_put_service = rospy.Service(
            f'{local_capability_topic_prefix}/capabilities/implementations/put',
            PutCapabilityImplementation,
            self.put_capability_implementation
        )

        self.__local_capability_implementations_delete_service = rospy.Service(
            f'{local_capability_topic_prefix}/capabilities/implementations/delete',
            DeleteCapabilityImplementation,
            self.delete_capability_implementation
        )

        # Global interfaces communication topics and services.
        self.__global_capability_interfaces_publisher = rospy.Publisher(
            f'{global_capability_topic_prefix}/interfaces',
            CapabilityInterface,
            queue_size=1000
        )

        self.__global_capability_interfaces_subscriber = rospy.Subscriber(
            f'{global_capability_topic_prefix}/interfaces',
            CapabilityInterface,
            self.__global_capability_interfaces_callback
        )

        self.__global_capability_interfaces_requests_subscriber = rospy.Subscriber(
            f'{global_capability_topic_prefix}/interfaces/request',
            Time,
            self.__global_capability_interfaces_request_callback
        )

        self.__global_capability_interfaces_requests_publisher = rospy.Publisher(
            f'{global_capability_topic_prefix}/interfaces/request',
            Time,
            queue_size=1000
        )

    # -- Inter-Nodes callback --
    def __global_capability_interfaces_callback(
            self,
            interface: CapabilityInterface
    ) -> None:
        """
        Subscriber callback for the capability announcement topic.

        :param interface: The capability interfaces that has been announced.
        :return: None
        """
        rospy.loginfo("Received announcement received!")
        hashable_interface = HashableCapabilityInterface(interface)
        if hashable_interface in self.capabilities:
            rospy.loginfo("Capability interface already exists, ignoring!")
            return
        with self.__capabilities_lock:
            self.capabilities[hashable_interface] = {}
        self.__publish_interface_local_topic(hashable_interface.interface)

    def __publish_local_interfaces_global_topic(
            self
    ) -> None:
        """
        Publishes all currently loaded local capabilities to the global capability topic.

        Fails if the publisher is not initialized.
        :return: None
        """
        for hashable_interface in self.capabilities:
            try:
                self.__global_capability_interfaces_publisher.publish(hashable_interface.interface)
            except rospy.ROSSerializationException as exc:
                rospy.logerr(f'Could not serialize msg, node not initialized: {exc}!')
                continue
            except rospy.ROSException as exc:
                rospy.logerr(f'Could not send msg, node not initialized: {exc}!')
                continue

    def __global_capability_interfaces_request_callback(
            self,
            msg: Time
    ) -> None:
        """
        Callback for publishing requests on the global capability request topic.
        Announces the local capability interfaces to the global topic.
        This allows all remote nodes to share the same interfaces.

        :param msg: The timestamp the request was posted at.
        :return: None
        """
        rospy.loginfo(f"Received announcement request at {msg.data}!")

        self.__publish_local_interfaces_global_topic()
        rospy.loginfo("Send all known interface definitions!")

    def shutdown(
            self
    ) -> None:
        """
        Shuts down the running node and disable all services and topics.
        :return: None
        """
        rospy.loginfo("Shutting down local services and unregister publishers!")
        self.__local_capabilities_save_service.shutdown()
        self.__local_capabilities_load_service.shutdown()

        self.__local_capability_implementation_publisher.unregister()
        self.__local_capability_interface_publisher.unregister()

        self.__local_capability_interfaces_get_service.shutdown()
        self.__local_capability_interfaces_put_service.shutdown()

        self.__local_capability_implementations_put_service.shutdown()
        self.__local_capability_implementations_get_service.shutdown()
        self.__local_capability_implementations_delete_service.shutdown()

        rospy.loginfo("Unregister global publishers and subscribers!")
        self.__global_capability_interfaces_publisher.unregister()
        self.__global_capability_interfaces_subscriber.unregister()
        self.__global_capability_interfaces_requests_publisher.unregister()
        self.__global_capability_interfaces_requests_subscriber.unregister()

        rospy.loginfo("Cleaning /tmp files!")
        try:
            shutil.rmtree(self.__capabilities_tmp_dir)
        except OSError as exc:
            rospy.logerr(
                f'Could not remove tmp files at: "{self.__capabilities_tmp_dir}"'
                f', "{exc}"! Please remove manually!'
            )

    # -- Interface operations --

    def __get_capabilities_folder_from_package(
            self,
            package: str
    ) -> str:
        """
        Validates the specified package name, ensures the "capabilities" folder exists and returns the path.

        The method will create the required folder should it not exist.
        :param package: String containing the package name the folder should be stored in.
        :return: The full path to the "capabilities_interface" folder.
        :raise ValueError: Specified package is not a valid package name or folder cannot be created.
        """

        # Check if the request contains a valid package.
        if len(package) < 1:
            raise ValueError('Empty package name provided!')

        try:
            package = self.__rp.get_path(package)
        except ResourceNotFound as exc:
            raise ValueError(f'Package could not be found: "{exc}"!') from exc

        # Check if the capability interface folder exists at the
        capability_interfaces_folder_path = os.path.join(package, "capabilities")
        if not os.path.isdir(capability_interfaces_folder_path):
            rospy.logwarn(f'Creating capabilities folder at: "{capability_interfaces_folder_path}"')
            os.mkdir(capability_interfaces_folder_path)
        if not os.path.isdir(capability_interfaces_folder_path):
            raise ValueError(f'Could not create capabilities folder: "{capability_interfaces_folder_path}"!')

        return capability_interfaces_folder_path

    def __publish_interface_local_topic(
            self,
            interface: CapabilityInterface
    ) -> None:
        """
        Announces the interface on the local interface topic.

        :raise rospy.ROSException: If the message could not be sent because the node
        is not available or the message is malformed.
        :param interface: The interface to publish.
        :return: None
        """
        try:
            self.__local_capability_interface_publisher.publish(interface)
        except rospy.ROSSerializationException as exc:
            rospy.logerr(f'Could not serialize msg, node not initialized: {exc}!')
            raise rospy.ROSException(f'Could not serialize msg, node not initialized: {exc}!')
        except rospy.ROSException as exc:
            rospy.logerr(f'Could not send msg, node not initialized: {exc}!')
            raise rospy.ROSException(f'Could not send msg, node not initialized: {exc}!')

    def __publish_implementation_local_topic(
            self,
            implementation: CapabilityImplementation
    ) -> None:
        """
        Announces the implementation on the local implementation topic.

        :raise rospy.ROSException: If the message could not be sent because the node
        is not available or the message is malformed.
        :param implementation: The implementation to publish.
        :return: None
        """
        try:
            self.__local_capability_implementation_publisher.publish(implementation)
        except rospy.ROSSerializationException as exc:
            rospy.logerr(f'Could not serialize msg, node not initialized: {exc}!')
            raise rospy.ROSException(f'Could not serialize msg, node not initialized: {exc}!')
        except rospy.ROSException as exc:
            rospy.logerr(f'Could not send msg, node not initialized: {exc}!')
            raise rospy.ROSException(f'Could not send msg, node not initialized: {exc}!')

    @staticmethod
    def __increment_name(name):
        """If `name` does not already end in a number, add "_2" to it.

        Otherwise, increase the number after the underscore.
        """
        match = re.search(r'_(\d+)$', name)
        prev_number = 1
        if match:
            prev_number = int(match.group(1))
            # remove the entire _$number part from the name
            name = name[:len(name) - len(match.group(0))]

        name += f'_{prev_number + 1}'
        return name

    def save_capabilities(
            self,
            request: SaveCapabilitiesRequest
    ) -> SaveCapabilitiesResponse:
        """
        Saves the provided capabilities at the package specified in the request.
        Capabilities are saved as yaml files.

        The service will abort at the first failed save.

        The folder used will be called "capabilities" in the specified package.

        :param request: The save request from the service call.
        :return: Returns successfully if all capabilities could be saved. A failed response will occur if the specified
        package cannot be found, the folder cannot be created or a capability cannot be saved.
        """
        # pylint: disable=too-many-locals
        response = SaveCapabilitiesResponse()
        try:
            capabilities_folder_path = self.__get_capabilities_folder_from_package(request.package)
        except ValueError as exc:
            response.success = False
            response.error_message = str(exc.args)
            return response

        # Secure existing files, so they can be manually restored.
        # They will be deleted by the operating system, thus this will not be a big problem.

        existing_files = list(os.scandir(capabilities_folder_path))

        if len(existing_files) > 0:
            rospy.logwarn(f'Existing capability interface files are moved to {self.__capabilities_tmp_dir}"')
            shutil.copytree(capabilities_folder_path, self.__capabilities_tmp_dir, dirs_exist_ok=True)

            for existing_file in existing_files:
                rospy.logwarn(f"Removing file {existing_file}")
                shutil.rmtree(existing_file.path)
            rospy.loginfo(f'Successfully moved {len(existing_files)} interface files.')

        with self.__capabilities_lock:
            capabilities = self.capabilities.items()

        used_folder_names = []
        for capability in capabilities:
            name = capability[0].interface.name
            while name in used_folder_names:
                name = self.__increment_name(name)
            used_folder_names.append(name)

            capability_folder_path = os.path.join(capabilities_folder_path, name)

            os.makedirs(capability_folder_path, exist_ok=True)
            if not os.path.isdir(capability_folder_path):
                response.error_message = f"Cannot create capability directory {capability_folder_path}"
                response.success = False
                rospy.logerr(response.error_message)
                return response
            capability_implementations_folder_path = os.path.join(capability_folder_path, "implementations")
            os.makedirs(capability_implementations_folder_path, exist_ok=True)
            if not os.path.isdir(capability_implementations_folder_path):
                response.error_message = f"Cannot create capability implementations directory" \
                                         f" {capability_implementations_folder_path}"
                response.success = False
                rospy.logerr(response.error_message)
                return response

            capability_interface_file_path \
                = os.path.join(capability_folder_path, 'interface.yaml')

            with open(capability_interface_file_path, 'w', encoding='UTF-8') as capability_interface_file:
                msg = genpy.message.strify_message(capability[0].interface)
                capability_interface_file.write(msg)

            for implementation_name, implementation in capability[1].items():
                with open(
                        os.path.join(
                            capability_implementations_folder_path,
                            f"{implementation_name}.yaml"
                        ),
                        'w', encoding='UTF-8'
                ) as capability_implementation_file:
                    msg = genpy.message.strify_message(implementation)
                    capability_implementation_file.write(msg)

        response.success = True
        return response

    def load_capabilities(self, request: LoadCapabilitiesRequest) -> LoadCapabilitiesResponse:
        """
        Loads all capabilities from a given package.
        This includes interfaces and if available any implementations.
        :param request: The request that contains the package.
        :return: A ros service response informing about the status of the request.
        """
        response = LoadCapabilitiesResponse()

        try:
            capabilities_folder_path = self.__get_capabilities_folder_from_package(request.package)
        except ValueError as exc:
            response.success = False
            response.error_message = str(exc.args)
            return response

        for capability_folder_path in [f.path for f in os.scandir(capabilities_folder_path)
                                       if f.is_dir() and os.path.isfile(os.path.join(f.path, 'interface.yaml'))]:
            interface_file_path = os.path.join(capability_folder_path, "interface.yaml")

            rospy.logwarn(f'Loading capability interface file: "{interface_file_path}".')

            with open(interface_file_path, 'r', encoding='UTF-8') as interface_file:
                capability = CapabilityInterface()
                try:
                    genpy.message.fill_message_args(
                        capability,
                        yaml.safe_load(interface_file),
                        keys={}
                    )
                except (genpy.MessageException, TypeError, KeyError):
                    rospy.logwarn(f'"{interface_file_path}" is malformed')
                    continue

                hashable_capability = HashableCapabilityInterface(capability)
                with self.__capabilities_lock:
                    self.capabilities[hashable_capability] = {}
                try:
                    self.__publish_interface_local_topic(capability)
                except rospy.ROSException as exc:
                    response.success = False
                    response.error_message = f"Error announcing change to local node topic: {exc}!"
                    return response

            implementation_subfolder_path = os.path.join(capability_folder_path, "implementations")
            if os.path.isdir(implementation_subfolder_path):

                for implementation_file_path in [f.path for f in os.scandir(implementation_subfolder_path)
                                                 if f.is_file() and (f.path.endswith(".yaml") or
                                                                     f.path.endswith(".yml"))]:
                    rospy.logdebug(f'Loading capability implementation file: "{implementation_file_path}".')
                    with open(implementation_file_path, 'r', encoding='UTF-8') as implementation_file:
                        implementation = CapabilityImplementation()
                        try:
                            genpy.message.fill_message_args(
                                implementation,
                                yaml.safe_load(implementation_file),
                                keys={}
                            )
                        except (genpy.MessageException, TypeError, KeyError):
                            rospy.logwarn(f'"{implementation_file_path}" is malformed')
                            continue
                        if self.capabilities[hashable_capability][implementation.implementation_name]:
                            rospy.logwarn(
                                f"Capability implementation with duplicate name detected:"
                                f" {implementation.implementation_name}"
                            )
                            continue
                        with self.__capabilities_lock:
                            self.capabilities[hashable_capability][implementation.implementation_name] = implementation
                        try:
                            self.__publish_implementation_local_topic(implementation)
                        except rospy.ROSException as exc:
                            response.success = False
                            response.error_message = f"Error announcing change to local node topic: {exc}!"
                            return response

        response.success = True
        return response

    def put_capability_interfaces(
            self,
            request: PutCapabilityInterfacesRequest
    ) -> PutCapabilityInterfacesResponse:
        """
        Adds the submitted capability interfaces to the repository.

        :param request: The submit request from the service call.
        :return:
        """
        response = PutCapabilityInterfacesResponse()
        for interface in request.capabilities:
            hashable_interface = HashableCapabilityInterface(interface)

            if hashable_interface in self.capabilities:
                rospy.logwarn(f"Interface {interface} is already loaded!")
                continue

            self.capabilities[hashable_interface] = {}

            try:
                self.__publish_interface_local_topic(interface)
            except rospy.ROSException as exc:
                response.success = False
                response.error_message = f"Error announcing change to local node topic: {exc}!"
                return response

        response.success = True
        return response

    def get_capability_interfaces(
            self,
            request: GetCapabilityInterfacesRequest
    ) -> GetCapabilityInterfacesResponse:
        """
        Returns the capabilities that are currently loaded.
        :param request: The service request.
        :return: All loaded capability interfaces.
        """
        # pylint: disable=unused-argument

        response = GetCapabilityInterfacesResponse()
        with self.__capabilities_lock:
            capability_interfaces = self.capabilities.keys()

        for hashable_interface in capability_interfaces:
            try:
                self.__publish_interface_local_topic(hashable_interface.interface)
            except rospy.ROSException as exc:
                response.success = False
                response.error_message = f"Error announcing change to local node topic: {exc}!"
                return response

        response.success = True
        return response

    # -- Implementation operations --
    def get_capability_implementations(
            self,
            request: GetCapabilityImplementationsRequest
    ) -> GetCapabilityImplementationsResponse:
        """
        Service handler that returns a given local implementation.
        :param request: The request to process.
        :return: A
        """
        response = GetCapabilityImplementationsResponse()

        hashable_interface = HashableCapabilityInterface(request.interface)
        try:
            with self.__capabilities_lock:
                capability_implementations = self.capabilities[hashable_interface]
        except KeyError:
            response.success = False
            response.error_message = f"Capability is not known: {request.interface}!"
            rospy.logwarn(response.error_message)
            return response

        response.implementations = capability_implementations
        response.success = True
        return response

    def delete_capability_implementation(
            self,
            request: DeleteCapabilityImplementationRequest
    ) -> DeleteCapabilityImplementationResponse:
        """
        Delete a current implementation from the repository.

        :param request: The request to use.
        :return: The service response.
        """
        response = DeleteCapabilityImplementationResponse()
        hashable_interface = HashableCapabilityInterface(request.interface)

        try:
            with self.__capabilities_lock:
                capability_implementations = self.capabilities[hashable_interface]
        except KeyError:
            response.success = False
            response.error_message = f'Capability {request.interface} not known!'
            rospy.logwarn(response.error_message)
            return response

        try:
            capability_implementations.pop(request.implementation_name)

        except KeyError:
            response.success = False
            response.error_message = f'Implementation {request.implementation_name} not known!'
            rospy.logwarn(response.error_message)
            return response

        # TODO: Implement deletion of persistent implementations.

        response.success = True
        return response

    def put_capability_implementation(
            self,
            request: PutCapabilityImplementationRequest
    ) -> PutCapabilityImplementationResponse:
        """
        Put a new implementation to the repository from the local node.

        :param request: The service request.
        :return: Success if the capability was successfully added to the repository.
        Failure if the capability interface is unknown or the uuids are not valid.
        """
        response = PutCapabilityImplementationResponse()
        hashable_interface = HashableCapabilityInterface(request.interface)

        try:
            self.capabilities[hashable_interface]
        except KeyError:
            response.success = False
            response.error_message = f'Capability {request.implementation.capability_uuid} not known!'
            rospy.logwarn(response.error_message)
            return response

        with self.__capabilities_lock:
            self.capabilities[hashable_interface][request.implementation.implementation_name] = request.implementation

        response.success = True
        return response
