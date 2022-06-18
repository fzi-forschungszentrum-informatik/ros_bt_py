import os
import tempfile
import uuid
from typing import Dict

import genpy
import rospy
import yaml

from ros_bt_py_msgs.msg import CapabilityInterface

from ros_bt_py_msgs.srv import SaveCapabilityInterfacesRequest, SaveCapabilityInterfacesResponse, \
    LoadCapabilityInterfacesRequest, LoadCapabilityInterfacesResponse, \
    SubmitCapabilityInterfacesRequest, SubmitCapabilityInterfacesResponse, \
    GetCapabilityInterfacesRequest, GetCapabilityInterfacesResponse, DeleteCapabilityInterfacesRequest, \
    DeleteCapabilityInterfacesResponse

import rospkg
import catkin.workspace
from rospkg import ResourceNotFound


class CapabilityRepository(object):

    def __init__(self):
        # List containing all known capability interfaces.
        self.capability_interfaces: Dict[str, CapabilityInterface] = dict()

        self._rp = rospkg.RosPack()
        self._tmp_dir = tempfile.mkdtemp(prefix="CapabilityInterfaceBackup")

    def shutdown(self):
        os.remove(self._tmp_dir)

    def __get_capability_interfaces_folder_from_package(self, package: str) -> str:
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
            package = self._rp.get_path(package)
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

    def save_capability_interfaces(self, request: SaveCapabilityInterfacesRequest) -> SaveCapabilityInterfacesResponse:
        """
        Saves the provided capabilities at the package specified in the request.
        Capabilities are saved as yaml files.

        The service will abort at the first failed save.

        The folder used will be called "capability_interfaces" in the specified package.

        :param request: The save request from the service call.
        :return: Returns successfully if all capabilities could be saved. A failed response will occur if the specified package cannot be found, the folder cannot be created or a capability cannot be saved.
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
            rospy.logwarn(f'Existing capability interface files are moved to {self._tmp_dir}"')
            for existing_file in existing_files:
                os.rename(
                    os.path.join(capability_interfaces_folder_path, existing_file),
                    os.path.join(self._tmp_dir, existing_file),
                )
            rospy.loginfo(f'Successfully moved {len(existing_files)} interface files.')

        for capability_uuid, capability in self.capability_interfaces.items():
            capability_interface_file_path \
                = os.path.join(capability_interfaces_folder_path, f'{capability_uuid.upper()}.yaml')

            rospy.logdebug(f'Saving capability interface: "{capability}" at "{capability_interface_file_path}".')

            with open(capability_interface_file_path, 'w') as capability_interface_file:
                msg = genpy.message.strify_message(capability)
                capability_interface_file.write(msg)

        response.success = True
        return response

    def load_capability_interfaces(self, request: LoadCapabilityInterfacesRequest) -> LoadCapabilityInterfacesResponse:
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
                        capability, data, keys={})
                except (genpy.MessageException, TypeError, KeyError) as e:
                    rospy.logwarn(f'"{capability_interface_file_path}" is malformed')
                    continue

                if len(capability.uuid) != 36:
                    rospy.logwarn(f'Interface: "{capability}" does not contain a valid uuid.')
                    continue

                loaded_capability_interfaces.append(capability)

                if capability.uuid in self.capability_interfaces:
                    rospy.logdebug(f"Updating interface {capability.uuid}!")
                else:
                    rospy.logdebug(f"Adding interface {capability.uuid}!")

                self.capability_interfaces[capability.uuid] = capability

        response.success = True
        response.capabilities = loaded_capability_interfaces
        return response

    def submit_capability_interfaces(self, request: SubmitCapabilityInterfacesRequest) -> SubmitCapabilityInterfacesResponse:
        """
        Adds the submitted capability interfaces to the repository.

        :param request: The submit request from the service call.
        :return:
        """
        response = SubmitCapabilityInterfacesResponse()
        for interface in request.capabilities:

            if len(interface.uuid) != 36:
                rospy.logwarn(f"Interface {interface} does not contain a valid uuid.")
                continue

            if interface.uuid in self.capability_interfaces:
                rospy.logdebug(f"Updating interface {interface.uuid}!")
            else:
                rospy.logdebug(f"Adding interface {interface.uuid}!")
            self.capability_interfaces[interface.uuid] = interface

        response.success = True
        return response

    def get_capability_interfaces(self, request: GetCapabilityInterfacesRequest) -> GetCapabilityInterfacesResponse:
        """
        Returns the capabilities that are currently loaded.
        :param request: The service request.
        :return: All loaded capability interfaces.
        """

        response = GetCapabilityInterfacesResponse()
        response.capabilities = list(self.capability_interfaces.values())
        return response

    def delete_capability_interfaces(self, request: DeleteCapabilityInterfacesRequest) -> DeleteCapabilityInterfacesResponse:
        """
        Removes the capabilities with the specified uuids from the local repository.
        :param request: The service request.
        :return: Successful response if all capabilities were removed, false on the first failure.
        """

        response = DeleteCapabilityInterfacesResponse()
        for capability_uuid in request.uuid:
            try:
                self.capability_interfaces.pop(capability_uuid)
            except KeyError as exc:
                response.success = False
                response.error_message = f"Capability interface with uuid {capability_uuid} does not exist!"
                return response

        response.success = True
        return response
