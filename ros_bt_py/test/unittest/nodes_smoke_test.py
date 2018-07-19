import importlib
import pkgutil

import unittest


class TestNodeImport(unittest.TestCase):
    """Simple smoke test for all node classes

    Your custom tree node packages should *at least* have something like
    this! Nodes should not actually *do* anything until setup() is
    called, but importing them here ensures they at least used the
    define_bt_node decorator correctly.
    """

    def testNodeImport(self):
        """Find and import all submodules of ros_bt_py.nodes

        This doesn't check much, just that we made no egregious errors
        using define_bt_node.
        """
        import ros_bt_py.nodes
        for _, name, _ in pkgutil.walk_packages(
                ros_bt_py.nodes.__path__):
            importlib.import_module(ros_bt_py.nodes.__name__ + '.' + name)
