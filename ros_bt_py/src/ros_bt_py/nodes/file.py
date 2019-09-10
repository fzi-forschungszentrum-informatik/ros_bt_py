import yaml
import rospkg

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(NodeConfig(
    options={'file_path': str},
    inputs={},
    outputs={'load_success': bool,
             'load_error_msg': str,
             'content': list,
             'line_count': int},
    max_children=0,
    optional_options=['something']))
class File(Leaf):
    """Loads a yaml file from the location pointed to by `file_path`.
    This uses package:// and file:// style URIs.

    """
    def __init__(self, options=None, debug_manager=None, name=None):
        super(File, self).__init__(options, debug_manager, name)
        self.data = None

    def _do_setup(self):
        self.data = self.load_file(self.options['file_path'])

    def _do_tick(self):
        if self.data:
            self.outputs['content'] = self.data
            self.outputs['line_count'] = len(self.data)

            return NodeMsg.SUCCEEDED
        return NodeMsg.FAILED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def load_file(self, path):
        rospack = rospkg.RosPack()
        file_path = ''
        if path.startswith('file://'):
            file_path = path[len('file://'):]
        elif path.startswith('package://'):
            package_name = path[len('package://'):].split('/', 1)[0]
            package_path = rospack.get_path(package_name)
            file_path = package_path + path[len('package://') + len(package_name):]
        else:
            self.outputs['load_success'] = False
            self.outputs['load_error_msg'] = ('File path "%s" is malformed. It needs to start with '
                                              'either "file://" or "package://"') % path
            return None
        try:
            data_file = open(file_path, 'r')
        except IOError as ex:
            self.outputs['load_success'] = False
            self.outputs['load_error_msg'] = ('Error opening file %s: %s' % (file_path, str(ex)))
            self.logwarn(self.outputs['load_error_msg'])
            return None
        with data_file:
            data = yaml.load(data_file)
            if data and isinstance(data, list):
                if not all(isinstance(line, basestring) for line in data):
                    self.outputs['load_success'] = False
                    self.outputs['load_error_msg'] = (
                        'YAML file "%s" is malformed, is it really a list?' % file_path)
                else:
                    self.outputs['load_success'] = True
                    return data
            else:
                self.outputs['load_success'] = False
                self.outputs['load_error_msg'] = ('No data in YAML file %s!' % file_path)
        return None
