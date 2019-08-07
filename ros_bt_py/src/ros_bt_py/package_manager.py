import jsonpickle
import os

import roslib
import rospkg
import rospy

import catkin
from catkin.find_in_workspaces import find_in_workspaces # we need catkin in exec_depend!
from ros_bt_py_msgs.msg import Message, Messages, Package, Packages
from ros_bt_py_msgs.srv import GetMessageFields, GetMessageFieldsResponse, SaveTreeResponse
from ros_bt_py_msgs.srv import GetPackageStructureResponse

# FIXME !!!! !!!!! !!!!! move the "ListOfPackages" message type to ros_bt_py !!!!
from ros_ta_msgs.msg import ListOfPackages


class PackageManager(object):
    """Provides functionality to interact with ROS messages and catkin packages
    """
    def __init__(self,
                 publish_message_list_callback=None,
                 publish_packages_list_callback=None):
        # using rospkg is nice because it provides path resolution and honors CATKIN_IGNORE file
        # so we do not have to do this manually
        self.rp = rospkg.RosPack()

        self.message_list_pub = publish_message_list_callback
        if self.message_list_pub is None:
            rospy.loginfo('No callback for publishing message list data provided.')
        else:
            self.publish_message_list()
        self.packages_list_pub = publish_packages_list_callback
        if self.packages_list_pub is None:
            rospy.loginfo('No callback for publishing packages list data provided.')
        else:
            self.publish_packages_list()
    
    def _get_package_paths(self, pkgname, rospack):
        _catkin_workspace_to_source_spaces = {}
        _catkin_source_path_to_packages = {}
        paths = []
        path = rospack.get_path(pkgname)
        paths.append(path)
        results = find_in_workspaces(search_dirs=['share'],
                                     project=pkgname,
                                     first_match_only=True,
                                     workspace_to_source_spaces=_catkin_workspace_to_source_spaces,
                                     source_path_to_packages=_catkin_source_path_to_packages)
        if results and results[0] != path:
            paths.append(results[0])
        return paths

    def publish_message_list(self):
        """Publishes a list of all ROS messages/services available on the system.
        Uses a similar strategy to rosmsg/rossrv to detect message/service files.
        """
        rospack = rospkg.RosPack()

        messages = []
        packages = rospack.list()

        actions = []
        for package in packages:
            for package_path in self._get_package_paths(package, rospack):
                path = package_path + "/msg"
                resources = []
                if os.path.isdir(path):
                    resources = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
                result = [x[:-len(".msg")] for x in resources]
                result.sort()
                for msg_type in result:
                    if msg_type[-6:] == "Action":
                        actions.append(msg_type)
                    append_msg = True
                    for action in actions:
                        if msg_type == action+"Feedback" or msg_type == action+"Goal" or msg_type == action+"Result":
                            append_msg = False
                    if append_msg:
                        messages.append(Message(msg=package+"/"+msg_type, service=False))
                path = package_path + "/srv"
                resources = []
                if os.path.isdir(path):
                    resources = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
                result = [x[:-len(".srv")] for x in resources]
                result.sort()
                for srv_type in result:
                    messages.append(Message(msg=package+"/"+srv_type, service=True))

        msg = Messages()
        msg.messages = messages
        self.message_list_pub.publish(msg)

    def get_message_fields(self, request):
        """Returns the jsonpickled fields of the provided message type.
        """
        response = GetMessageFieldsResponse()
        try:
            message_class = None
            if request.service:
                message_class = roslib.message.get_service_class(request.message_type)
            else:
                message_class = roslib.message.get_message_class(request.message_type)
            for field in str(message_class()).split("\n"):
                response.field_names.append(field.split(":")[0].strip())
            response.fields = jsonpickle.encode(message_class())
            response.success = True
        except Exception as e:
            response.success = False
            response.error_message = "Could not get message fields for {}: {}".format(request.message_type, e)
        return response
    
    def publish_packages_list(self):
        # get source_paths, which allows us to restrict the package list only to packages within the source space
        # this is needed because we should only ever add files to packages in the source space
        source_paths = []
        for ws in catkin.workspace.get_workspaces():
            source_paths += catkin.workspace.get_source_paths(ws)

        packages = self.rp.list()

        self.package_paths = []

        list_of_packages = Packages()
        list_of_packages.ros_root = rospkg.get_ros_root()

        for package in packages:
            # add all paths to the package path to be able to load installed capabilities
            package_path = self.rp.get_path(package)
            self.package_paths.append(package_path)

            # filter the published packages to only include packages in the source space (because we can edit those)
            for source_path in source_paths:
                if package_path.startswith(source_path):
                    package_msg = Package()
                    package_msg.package = package
                    package_msg.path = package_path
                    list_of_packages.packages.append(package_msg)
                    break

        self.packages_list_pub.publish(list_of_packages)
    
    def get_directory_structure(self, rootdir):
        """
        Creates a nested dictionary that represents the folder structure of rootdir
        """
        dir = {}
        rootdir = rootdir.rstrip(os.sep)
        start = rootdir.rfind(os.sep) + 1
        for path, dirs, files in os.walk(rootdir):
            folders = path[start:].split(os.sep)
            subdir = dict.fromkeys(files)
            parent = reduce(dict.get, folders[:-1], dir)
            parent[folders[-1]] = subdir
        return dir

    def get_package_structure(self, request):
        """Returns a listing of all files and subdirectories of a ROS package as a jsonpickled string
        """
        response = GetPackageStructureResponse()
        try:
            rospy.logwarn(request.package)
            package_path = self.rp.get_path(request.package)
            if not os.path.isdir(package_path):
                response.success = False
                response.error_message = 'Package path "{}" does not exist'.format(package_path)
            else:
                package_structure = self.get_directory_structure(package_path)

                response.success = True
                response.package_structure = jsonpickle.encode(package_structure)
                
                # for path, subdirs, files in os.walk(package_path):
                #     for name in files:
                #         absolute_path = os.path.join(path, name)
                #         rospy.logerr("file: {}".format(absolute_path))
                #     rospy.logwarn("subdirs {}".format(subdirs))

        except rospkg.common.ResourceNotFound:
            response.success = False
            response.error_message = 'Package "{}" does not exist'.format(request.package)

        return response


# # Filename can contain a relative path starting from the package root
# string filename
# Package package
# Tree tree
# bool allow_overwrite
# bool allow_rename
# ---
# bool success
# string error_message


    def save_tree(self, request):
        """Saves a tree message in the given package

        :param ros_bt_py_msgs.srv.SaveTree request:

        If `request.filename` contains forward slashes, treat it as a relative path.
        If `request.allow_overwrite` is True, the file will be overwritten, otherwise service call will fail.
        If `request.allow_rename` is True files will never be overwritten, the new file will always be renamed.

        :returns: :class:`ros_bt_py_msgs.src.SaveTreeResponse` or `None`

        Always returns the path under which the tree was save in response.file_path in the package:// style
        """
        response = SaveTreeResponse()

        try:
            package_path = self.rp.get_path(request.package)
            if not os.path.isdir(package_path):
                response.success = False
                response.error_message = 'Package path "{}" does not exist'.format(package_path)
            else:
                for path, subdirs, files in os.walk(package_path):
                    for name in files:
                        absolute_path = os.path.join(path, name)
                        rospy.logerr("file: {}".format(absolute_path))
                    rospy.logwarn("subdirs {}".format(subdirs))
                # capabilities_folder = os.path.join(path, "capabilities")
                # if not os.path.isdir(capabilities_folder):
                #     rospy.logerr("creating capabilities folder")
                #     os.mkdir(capabilities_folder)
                # if not os.path.isdir(capabilities_folder):
                #     response.success = False
                #     response.error_message = "Could not create capabilities folder"
                # else:
                #     capability_folder = os.path.join(capabilities_folder, request.capability.name)
                #     if os.path.isdir(capability_folder):
                #         counter = 0
                #         new_folder = capability_folder
                #         while os.path.isdir(new_folder):
                #             new_folder = capability_folder + "_" + str(counter)
                #             counter += 1
                #         capability_folder = new_folder
                #     rospy.logerr("capability_folder %s", capability_folder)
                #     capability_folder_name = capability_folder.replace(capabilities_folder, '')
                #     rospy.logerr("capability folder name %s", capability_folder_name)
                #     os.mkdir(capability_folder)
                #     if not os.path.isdir(capability_folder):
                #         response.success = False
                #         response.error_message = "Could not create folder for capability"
                #     else:
                #         rospy.logerr("writing capability.yaml and coordinator.yaml")
                #         capability_file_path = os.path.join(capability_folder, "capability.yaml")
                #         capability_coordinator_path = os.path.join(capability_folder, "coordinator.yaml")
                #         with open(capability_file_path, 'w') as capability_file:
                #             msg = genpy.message.strify_message(request.capability)
                #             capability_file.write(msg)
                #         with open(capability_coordinator_path, 'w') as coordinator_file:
                #             if request.coordinator.name == '':
                #                 request.coordinator.name = request.capability.name
                #             msg = genpy.message.strify_message(request.coordinator)
                #             coordinator_file.write(msg)
                #         # add the newly created capability to the capability manager
                #         if request.capability.path != "" and ":" not in request.capability.path:
                #             package_path = "package://" + request.package + "/capabilities/" + \
                #                            capability_folder_name + "/" + request.capability.path
                #             rospy.logwarn("relative path! %s", package_path)

                #             request.capability.path = package_path
                #         self.parse_capability_object(request.capability)
                #         self.verify_capabilities()

        except rospkg.common.ResourceNotFound:
            response.success = False
            response.error_message = 'Package "{}" does not exist'.format(request.package)

        return response
