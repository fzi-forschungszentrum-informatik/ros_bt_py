from ros_bt_py.migration import Migration, migration


class LookupTF(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass


class LookupTFConst(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass
