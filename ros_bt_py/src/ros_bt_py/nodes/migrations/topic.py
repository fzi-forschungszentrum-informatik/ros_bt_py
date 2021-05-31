from ros_bt_py.migration import Migration, migration


class TopicSubscriber(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass

    @migration(from_version='0.9.0', to_version='1.0.0', changelog='Preventing Shutdown exceptions')
    def fix_exception(self):
        pass


class TopicMemorySubscriber(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass

    @migration(from_version='0.9.0', to_version='1.0.0', changelog='Preventing Shutdown exceptions')
    def fix_exception(self):
        pass


class TopicPublisher(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass

    @migration(from_version='0.9.0', to_version='1.0.0', changelog='Preventing Shutdown exceptions')
    def fix_exception(self):
        pass
