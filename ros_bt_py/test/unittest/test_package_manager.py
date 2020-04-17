import unittest

from ros_bt_py.package_manager import PackageManager


class TestPackageManager(unittest.TestCase):
    def testLoadPackageManager(self):
        manager = PackageManager()
