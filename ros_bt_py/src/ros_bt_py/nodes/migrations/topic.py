from ros_bt_py.migration import Migration, migration


class TopicSubscriber(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass


class TopicOnlineSubscriber(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass


class TopicPublisher(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass
