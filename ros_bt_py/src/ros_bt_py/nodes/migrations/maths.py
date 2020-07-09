from ros_bt_py.migration import Migration, migration


class Convert(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass


class Operation(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass


class UnaryOperation(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass
