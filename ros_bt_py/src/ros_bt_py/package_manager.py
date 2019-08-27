import jsonpickle
import os

import roslib
import rospkg
import rospy
import genpy

import catkin
from catkin.find_in_workspaces import find_in_workspaces
from ros_bt_py_msgs.msg import Message, Messages, Package, Packages
from ros_bt_py_msgs.srv import GetMessageFields, GetMessageFieldsResponse, SaveTreeResponse
from ros_bt_py_msgs.srv import GetPackageStructureResponse

from ros_bt_py.node import increment_name


class PackageManager(object):
    """Provides functionality to interact with ROS messages and catkin packages
    """
    def __init__(self,
                 publish_message_list_callback=None,
                 publish_packages_list_callback=None):
        # using rospkg is nice because it provides path resolution and honors CATKIN_IGNORE file
        # so we do not have to do this manually
        self.rospack = rospkg.RosPack()
        self.item_id = 0

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
                    resources = [f for f in os.listdir(path)
                                 if os.path.isfile(os.path.join(path, f))]
                result = [x[:-len(".msg")] for x in resources]
                result.sort()
                for msg_type in result:
                    if msg_type[-6:] == "Action":
                        actions.append(msg_type)
                    append_msg = True
                    for action in actions:
                        if (msg_type == action + "Feedback" or msg_type == action + "Goal" or
                                msg_type == action + "Result"):
                            append_msg = False
                    if append_msg:
                        messages.append(Message(msg=package + "/" + msg_type, service=False))
                path = package_path + "/srv"
                resources = []
                if os.path.isdir(path):
                    resources = [f for f in os.listdir(path)
                                 if os.path.isfile(os.path.join(path, f))]
                result = [x[:-len(".srv")] for x in resources]
                result.sort()
                for srv_type in result:
                    messages.append(Message(msg=package + "/" + srv_type, service=True))

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
            response.error_message = "Could not get message fields for {}: {}".format(
                request.message_type, e)
        return response

    def publish_packages_list(self):
        # get source_paths, which allows us to restrict the package list to packages
        # within the source space
        # this is needed because we should only ever add files to packages in the source space
        source_paths = []
        for ws in catkin.workspace.get_workspaces():
            source_paths += catkin.workspace.get_source_paths(ws)

        packages = self.rospack.list()

        self.package_paths = []

        list_of_packages = Packages()
        list_of_packages.ros_root = rospkg.get_ros_root()

        for package in packages:
            # add all paths to the package path to be able to load installed capabilities
            package_path = self.rospack.get_path(package)
            self.package_paths.append(package_path)

            # filter the published packages to only include packages in the source space
            for source_path in source_paths:
                if package_path.startswith(source_path):
                    package_msg = Package()
                    package_msg.package = package
                    package_msg.path = package_path
                    list_of_packages.packages.append(package_msg)
                    break

        self.packages_list_pub.publish(list_of_packages)

    def _root(self, name=None, children=[]):
        return {
            "type": "root",
            "name": name,
            "children": children,
        }

    def _workspace(self, name=None, children=[], absolute_path=None):
        return {
            "type": "workspace",
            "name": name,
            "children": children,
            "absolute_path": absolute_path,
        }

    def _package(self, name=None, children=[]):
        return {
            "type": "package",
            "name": name,
            "children": children,
        }

    def get_workspace_tree(self):
        """Returns a tree that contains all catkin workspaces with a source space and their packages
        """

        # get a list of all packages
        packages = self.rospack.list()

        list_of_packages = []

        for package in packages:
            # add all paths to the package path to be able to load installed capabilities
            package_path = self.rospack.get_path(package)
            package_msg = {}
            package_msg["name"] = package
            package_msg["path"] = package_path
            list_of_packages.append(package_msg)

        workspace_tree = self._root()
        for ws in catkin.workspace.get_workspaces():
            source_paths = catkin.workspace.get_source_paths(ws)
            rospy.logwarn(source_paths)
            if len(source_paths) == 0:
                continue
            elif len(source_paths) > 1:
                rospy.logwarn("multiple source paths, only adding the first")

            # only consider the first source_path
            source_path = source_paths[0]

            workspace = self._workspace(absolute_path=source_path)

            # filter the packages to only include packages in the source space
            for source_path in source_paths:
                for package in list_of_packages:
                    if package["path"].startswith(source_path):
                        workspace["children"].append(self._package(name=package["name"]))

            workspace_tree["children"].append(workspace)

        return workspace_tree

    def get_id(self):
        self.item_id += 1
        return self.item_id

    def reset_id(self):
        self.item_id = 0

    def path_to_dict(self, path, show_hidden=False, parent=0):
        """Turns a path into a dictionary
        """
        d = {'name': os.path.basename(path)}
        d['item_id'] = self.get_id()
        d['parent'] = parent
        if os.path.isdir(path):
            d['type'] = "directory"
            d['children'] = [self.path_to_dict(os.path.join(path, f),
                                               show_hidden=show_hidden, parent=d['item_id'])
                             for f in os.listdir(path) if (show_hidden or not f.startswith("."))]
        else:
            d['type'] = "file"
        return d

    def get_package_structure(self, request):
        """Returns a listing of files and subdirectories of a ROS package as a jsonpickled string

        Hides hidden files by default, unless show_hidden is set to true.
        """
        response = GetPackageStructureResponse()
        try:
            package_path = self.rospack.get_path(request.package)
            if not os.path.isdir(package_path):
                response.success = False
                response.error_message = 'Package path "{}" does not exist'.format(package_path)
            else:

                self.reset_id()
                package_structure = self.path_to_dict(
                    path=package_path, show_hidden=request.show_hidden)

                response.success = True
                response.package_structure = jsonpickle.encode(package_structure)

        except rospkg.common.ResourceNotFound:
            response.success = False
            response.error_message = 'Package "{}" does not exist'.format(request.package)

        return response

    def make_filepath_unique(self, filepath):
        name, extension = os.path.splitext(filepath)
        while os.path.exists(name + extension):
            name = increment_name(name)
        return name + extension

    def save_tree(self, request):
        """Saves a tree message in the given package

        :param ros_bt_py_msgs.srv.SaveTree request:

        If `request.filename` contains forward slashes, treat it as a relative path.
        If `request.allow_overwrite` is True, the file is overwritten, otherwise service call fails
        If `request.allow_rename` is True files will no be overwritten,
            the new file will always be renamed.

        :returns: :class:`ros_bt_py_msgs.src.SaveTreeResponse` or `None`

        Always returns the path under which the tree was saved
        in response.file_path in the package:// style
        """
        response = SaveTreeResponse()

        try:
            package_path = self.rospack.get_path(request.package)
            if not os.path.isdir(package_path):
                response.success = False
                response.error_message = 'Package path "{}" does not exist'.format(package_path)
            else:
                save_path = os.path.join(package_path, request.filename)

                if os.path.commonprefix(os.path.realpath(save_path), package_path) != package_path:
                    response.success = False
                    response.error_message = 'Path outside package path'
                    return response
                save_path = save_path.rstrip(os.sep)  # split trailing /
                path, filename = os.path.split(save_path)

                try:
                    os.makedirs(path)
                except OSError:
                    if not os.path.isdir(path):
                        response.success = False
                        response.error_message = "Could not create path"
                        return response

                if os.path.isdir(save_path):
                    response.success = False
                    response.error_message = "File path already exists as directory"
                    return response

                if os.path.isfile(save_path):
                    if request.allow_rename:
                        save_path = self.make_filepath_unique(save_path)
                        if os.path.isfile(save_path):
                            response.success = False
                            response.error_message = "Rename failed"
                            return response
                    else:
                        if not request.allow_overwrite:
                            response.success = False
                            response.error_message = "Overwrite not allowed"
                            return response

                with open(save_path, 'w') as save_file:
                    msg = genpy.message.strify_message(request.tree)
                    save_file.write(msg)

                response.success = True
                return response

        except rospkg.common.ResourceNotFound:
            response.success = False
            response.error_message = 'Package "{}" does not exist'.format(request.package)

        return response
