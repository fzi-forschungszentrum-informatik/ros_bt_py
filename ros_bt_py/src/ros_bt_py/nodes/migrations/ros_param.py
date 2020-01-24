from ros_bt_py.migration import Migration, migration


class RosParam(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass

    @migration(from_version='0.9.0', to_version='1.0.0', changelog='changing to RosParamOption')
    def changing_node_class(self):
        self.change_node_class('RosParamOption')


class RosParamOption(Migration):
    @migration(from_version='', to_version='1.0.0', changelog='adding version number')
    def adding_version(self):
        pass


class RosParamInput(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass
