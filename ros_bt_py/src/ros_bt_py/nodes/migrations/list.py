from ros_bt_py.migration import Migration, migration
from ros_bt_py.ros_helpers import LoggerLevel

class IterateList(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='added version')
    def adding_version(self):
        pass


class IterateList(Migration):
    @migration(from_version='0.9.0', to_version='0.9.1', changelog='Running once without ticking child')
    def adding_version(self):
        pass


class IsInList(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='added version')
    def adding_version(self):
        pass


class InsertInList(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='added version')
    def adding_version(self):
        pass


class GetListElementOption(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='added version')
    def adding_version(self):
        pass


class ListLength(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='added version')
    def adding_version(self):
        pass
