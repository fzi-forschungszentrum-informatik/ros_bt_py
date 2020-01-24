from ros_bt_py.migration import Migration, migration
from ros_bt_py.node_config import OptionRef


class Action(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass
