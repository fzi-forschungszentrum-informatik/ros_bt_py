from ros_bt_py.migration import Migration, migration


class IOInputOption(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass


class IOInput(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass


class IOOutputOption(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass


class IOOutput(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass
