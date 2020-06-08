from ros_bt_py.migration import Migration, migration
from ros_bt_py.ros_helpers import LoggerLevel


class IterateList(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='added version')
    def adding_version(self):
        pass

    @migration(from_version='0.9.0', to_version='0.9.1',
               changelog='Running once without ticking child')
    def adding_version_091(self):
        pass

    @migration(from_version='0.9.1', to_version='0.9.2', changelog='Failing on failed child')
    def adding_version_092(self):
        pass

    @migration(from_version='0.9.2', to_version='1.0.0', changelog='Succeed on empty list')
    def adding_version_100(self):
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
