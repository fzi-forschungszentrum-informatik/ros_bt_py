import unittest


class TestNodeImport(unittest.TestCase):
    """Simple smoke test for all node classes

    Your custom tree node packages should *at least* have something like
    this! Nodes should not actually *do* anything until setup() is
    called, but importing them here ensures they at least used the
    define_bt_node decorator correctly.
    """

    def testNodeImport(self):
        from ros_bt_py.nodes.getters import GetListItem
        from ros_bt_py.nodes.action import Action
        from ros_bt_py.nodes.fallback import Fallback
        from ros_bt_py.nodes.mock_nodes import MockLeaf
        from ros_bt_py.nodes.passthrough_node import PassthroughNode
        from ros_bt_py.nodes.sequence import Sequence
        from ros_bt_py.nodes.topic import TopicSubscriber, TopicPublisher
