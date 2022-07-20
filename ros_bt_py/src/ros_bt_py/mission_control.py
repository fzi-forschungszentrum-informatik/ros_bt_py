"""
Module used to implement mission control components.
This includes the management of capabilities and their implementations as well as the forwarding to
other nodes by using an auction protocol.
"""

import os
import shutil
import tempfile
import uuid
from threading import RLock
from typing import Dict

import genpy
import rospkg
import rospy
import yaml
from rospkg import ResourceNotFound
from std_msgs.msg import Time
from ros_bt_py_msgs.msg import CapabilityInterface, CapabilityImplementation
from ros_bt_py_msgs.srv import (
    SaveCapabilityInterfacesRequest, SaveCapabilityInterfacesResponse,
    LoadCapabilityInterfacesRequest, LoadCapabilityInterfacesResponse,
    PutCapabilityInterfacesRequest, PutCapabilityInterfacesResponse,
    GetCapabilityInterfacesRequest, GetCapabilityInterfacesResponse,
    GetCapabilityImplementationsRequest, GetCapabilityImplementationsResponse, DeleteCapabilityImplementationResponse,
    DeleteCapabilityImplementationRequest, PutCapabilityImplementationRequest,
    PutCapabilityImplementationResponse, DeleteCapabilityImplementation,
    LoadCapabilityInterfaces,
    SaveCapabilityInterfaces, GetCapabilityInterfaces, PutCapabilityInterfaces,
    GetCapabilityImplementations, PutCapabilityImplementation,
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
        self.__capability_interfaces_lock = RLock()
        self.capability_interfaces: Dict[str, CapabilityInterface] = dict()

        # List containing all locally known implementations.
        self.__capability_implementations_lock = RLock()
        self.capability_implementations: Dict[str, Dict[str, CapabilityImplementation]] = dict()

        self.__rp = rospkg.RosPack()
        self.__interfaces_tmp_dir = tempfile.mkdtemp(prefix="capability_interface_backup_")

        # Local interfaces communication topics and services.
        self.__local_capability_interface_publisher = rospy.Publisher(
            f'{local_capability_topic_prefix}/capabilities/interfaces',
            CapabilityInterface,
            queue_size=1000
        )

        self.__local_capability_interfaces_load_service = rospy.Service(
            f'{local_capability_topic_prefix}/capabilities/interfaces/load',
            LoadCapabilityInterfaces,
            self.load_capability_interfaces
        )

        self.__local_capability_interfaces_save_service = rospy.Service(
            f'{local_capability_topic_prefix}/capabilities/interfaces/save',
            SaveCapabilityInterfaces,
            self.save_capability_interfaces
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

    def __is_invalid_uuid(self, uuid_string: str) -> bool:
        """
        Checks if a given sting is not a valid UUID.

        :param uuid_string: The string to test against.
        :return: True if not a valid UUID, false if the input is a valid UUID.
        """
        try:
            uuid.UUID(uuid_string)
            return False
        except ValueError:
            return True

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
        if interface.uuid in self.capability_interfaces.keys():
            rospy.loginfo("Capability interface already exists, ignoring!")
            return
        with self.__capability_interfaces_lock:
            self.capability_interfaces[interface.uuid] = interface
        self.__publish_interface_local_topic(interface)

    def __publish_local_interfaces_global_topic(
            self
    ) -> None:
        """
        Publishes all currently loaded local capabilities to the global capability topic.

        Fails if the publisher is not initialized.
        :return: None
        """
        for interface in self.capability_interfaces.values():
            try:
                self.__global_capability_interfaces_publisher.publish(interface)
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
        self.__local_capability_interface_publisher.unregister()
        self.__local_capability_interfaces_get_service.shutdown()
        self.__local_capability_interfaces_put_service.shutdown()
        self.__local_capability_interfaces_save_service.shutdown()
        self.__local_capability_interfaces_load_service.shutdown()

        rospy.loginfo("Unregister global publishers and subscribers!")
        self.__global_capability_interfaces_publisher.unregister()
        self.__global_capability_interfaces_subscriber.unregister()

        rospy.loginfo("Cleaning /tmp files!")
        try:
            shutil.rmtree(self.__interfaces_tmp_dir)
        except OSError as exc:
            rospy.logerr(
                f'Could not remove tmp files at: "{self.__interfaces_tmp_dir}"'
                f', "{exc}"! Please remove manually!'
            )

    # -- Interface operations --

    def __get_capability_interfaces_folder_from_package(
            self,
            package: str
    ) -> str:
        """
        Validates the specified package name, ensures the "capabilities_interface" folder exists and returns the path.

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
            raise ValueError(f'Package could not be found: "{exc}"!')

        # Check if the capability interface folder exists at the
        capability_interfaces_folder_path = os.path.join(package, "capability_interfaces")
        if not os.path.isdir(capability_interfaces_folder_path):
            rospy.logwarn(f'Creating capability interface folder at: "{capability_interfaces_folder_path}"')
            os.mkdir(capability_interfaces_folder_path)
        if not os.path.isdir(capability_interfaces_folder_path):
            raise ValueError(f'Could not create capability interface folder: "{capability_interfaces_folder_path}"!')

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

    def save_capability_interfaces(
            self,
            request: SaveCapabilityInterfacesRequest
    ) -> SaveCapabilityInterfacesResponse:
        """
        Saves the provided capabilities at the package specified in the request.
        Capabilities are saved as yaml files.

        The service will abort at the first failed save.

        The folder used will be called "capability_interfaces" in the specified package.

        :param request: The save request from the service call.
        :return: Returns successfully if all capabilities could be saved. A failed response will occur if the specified
        package cannot be found, the folder cannot be created or a capability cannot be saved.
        """
        response = SaveCapabilityInterfacesResponse()
        try:
            capability_interfaces_folder_path = self.__get_capability_interfaces_folder_from_package(request.package)
        except ValueError as exc:
            response.success = False
            response.error_message = str(exc.args)
            return response

        # Secure existing files, so they can be manually restored.
        # They will be deleted by the operating system, thus this will not be a big problem.

        existing_files = [f for f in os.listdir(capability_interfaces_folder_path) if
                          os.path.isfile(os.path.join(capability_interfaces_folder_path, f))]

        if len(existing_files) > 0:
            rospy.logwarn(f'Existing capability interface files are moved to {self.__interfaces_tmp_dir}"')
            for existing_file in existing_files:
                os.rename(
                    os.path.join(capability_interfaces_folder_path, existing_file),
                    os.path.join(self.__interfaces_tmp_dir, existing_file),
                )
            rospy.loginfo(f'Successfully moved {len(existing_files)} interface files.')

        with self.__capability_interfaces_lock:
            capability_interfaces = self.capability_interfaces.items()

        for capability_uuid, capability in capability_interfaces:
            capability_interface_file_path \
                = os.path.join(capability_interfaces_folder_path, f'{capability_uuid.upper()}.yaml')

            rospy.logdebug(f'Saving capability interface: "{capability}" at "{capability_interface_file_path}".')

            with open(capability_interface_file_path, 'w') as capability_interface_file:
                msg = genpy.message.strify_message(capability)
                capability_interface_file.write(msg)

        response.success = True
        return response

    def load_capability_interfaces(
            self,
            request: LoadCapabilityInterfacesRequest
    ) -> LoadCapabilityInterfacesResponse:
        """
        Loads all capabilities from the provided package.
        They should be saved as yaml files.

        :param request: The load request from the service call.
        :return: Returns the service response.
        """
        response = LoadCapabilityInterfacesResponse()
        loaded_capability_interfaces = []

        try:
            capability_interfaces_folder_path = self.__get_capability_interfaces_folder_from_package(request.package)
        except ValueError as exc:
            response.success = False
            response.error_message = str(exc.args)
            return response

        capability_interface_files = [f for f in os.listdir(capability_interfaces_folder_path) if
                                      os.path.isfile(os.path.join(capability_interfaces_folder_path, f))]

        for capability_interface_file_name in capability_interface_files:
            capability_interface_file_path \
                = os.path.join(capability_interfaces_folder_path, capability_interface_file_name)

            rospy.logdebug(f'Loading capability interface file: "{capability_interface_file_path}".')

            with open(capability_interface_file_path, 'r') as capability_interface_file:
                data = yaml.safe_load(capability_interface_file)
                rospy.logdebug(f"Loaded yaml: {data}")
                capability = CapabilityInterface()
                try:
                    genpy.message.fill_message_args(
                        capability, data, keys={}
                    )
                except (genpy.MessageException, TypeError, KeyError):
                    rospy.logwarn(f'"{capability_interface_file_path}" is malformed')
                    continue

                if self.__is_invalid_uuid(capability.uuid):
                    rospy.logwarn(f'Interface: "{capability}" does not contain a valid uuid.')
                    continue

                loaded_capability_interfaces.append(capability)

                if capability.uuid in self.capability_interfaces.keys():
                    rospy.logdebug(f"Updating interface {capability.uuid}!")
                else:
                    rospy.logdebug(f"Adding interface {capability.uuid}!")
                    with self.__capability_implementations_lock:
                        self.capability_implementations[capability.uuid] = dict()

                with self.__capability_interfaces_lock:
                    self.capability_interfaces[capability.uuid] = capability

        self.__publish_local_interfaces_global_topic()

        for interface in loaded_capability_interfaces:
            try:
                self.__publish_interface_local_topic(interface)
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

            if self.__is_invalid_uuid(interface.uuid):
                rospy.logwarn(f"Interface {interface} does not contain a valid uuid.")
                response.success = False
                response.error_message = f"Interface {interface} does not contain a valid uuid."
                return response

            if interface.uuid in self.capability_interfaces.keys():
                rospy.logdebug(f"Updating interface {interface.uuid}!")
            else:
                rospy.logdebug(f"Adding interface {interface.uuid}!")
                with self.__capability_implementations_lock:
                    self.capability_implementations[interface.uuid] = dict()

            with self.__capability_interfaces_lock:
                self.capability_interfaces[interface.uuid] = interface

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
        with self.__capability_interfaces_lock:
            capability_interfaces = self.capability_interfaces.values()

        for interface in capability_interfaces:
            try:
                self.__publish_interface_local_topic(interface)
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
        if self.__is_invalid_uuid(request.capability_uuid):
            response.success = False
            response.error_message = f"Invalid uuid in request: {request.capability_uuid}!"
            rospy.logwarn(response.error_message)
            return response

        try:
            with self.__capability_implementations_lock:
                capability_implementations = self.capability_implementations[request.capability_uuid]
        except KeyError:
            response.success = False
            response.error_message = f"Uuid is not known: {request.capability_uuid}!"
            rospy.logwarn(response.error_message)
            return response

        response.implementations = capability_implementations.values()
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

        if self.__is_invalid_uuid(request.capability_uuid):
            response.success = False
            response.error_message = f'Specified capability uuid {request.capability_uuid} invalid!'
            rospy.logwarn(response.error_message)
            return response

        if self.__is_invalid_uuid(request.implementation_uuid):
            response.success = False
            response.error_message = \
                f'Specified implementation uuid {request.implementation_uuid} invalid!'
            rospy.logwarn(response.error_message)
            return response

        try:
            implementations = self.capability_implementations[request.capability_uuid]
        except KeyError:
            response.success = False
            response.error_message = f'Capability {request.capability_uuid} not known!'
            rospy.logwarn(response.error_message)
            return response

        try:
            implementations.pop(request.implementation_uuid)
        except KeyError:
            response.success = False
            response.error_message = f'Implementation {request.implementation_uuid} not known!'
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

        if self.__is_invalid_uuid(request.implementation.capability_uuid):
            response.success = False
            response.error_message = f'Specified capability uuid {request.implementation.capability_uuid} invalid!'
            rospy.logwarn(response.error_message)
            return response

        if self.__is_invalid_uuid(request.implementation.implementation_uuid):
            response.success = False
            response.error_message \
                = f'Specified implementation uuid {request.implementation.implementation_uuid} invalid!'
            rospy.logwarn(response.error_message)
            return response

        try:
            self.capability_implementations[request.implementation.capability_uuid]
        except KeyError:
            response.success = False
            response.error_message = f'Capability {request.implementation.capability_uuid} not known!'
            rospy.logwarn(response.error_message)
            return response

        with self.__capability_implementations_lock:
            self.capability_implementations[request.implementation.capability_uuid][
                request.implementation.implementation_uuid] = request.implementation

        response.success = True
        return response
