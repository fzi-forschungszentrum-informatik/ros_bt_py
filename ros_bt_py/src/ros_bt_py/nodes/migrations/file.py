from ros_bt_py.migration import Migration, migration


class File(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass

    @migration(from_version='0.9.0', to_version='1.0.0', changelog='changed name to YamlListOption')
    def changing_node_class(self):
        self.change_node_class('YamlListOption')


class FileInput(Migration):
    @migration(from_version='', to_version='1.0.0', changelog='changed name to YamlListInput')
    def changing_node_class(self):
        self.change_node_class('YamlListInput')


class YamlInput(Migration):
    @migration(from_version='', to_version='1.0.0', changelog='changed name to YamlDictInput')
    def changing_node_class(self):
        self.change_node_class('YamlDictInput')


class YamlListInput(Migration):
    @migration(from_version='', to_version='1.0.0', changelog='added version number')
    def adding_version(self):
        pass


class YamlListOption(Migration):
    @migration(from_version='', to_version='1.0.0', changelog='added version number')
    def adding_version(self):
        pass


class YamlDictInput(Migration):
    @migration(from_version='', to_version='1.0.0', changelog='added version number')
    def adding_version(self):
        pass
