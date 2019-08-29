import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.ros_nodes.message_from_dict import MessageFromDict, MessageFromConstDict


class TestMessageFromDict(unittest.TestCase):
    def setUp(self):
        self.mfd = MessageFromDict({'message_type': NodeMsg})
        self.mfd.setup()

    def testPartialPopulate(self):
        self.mfd.inputs['dict'] = {'name': 'foo'}

        self.assertEqual(self.mfd.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(self.mfd.outputs['message'].name, 'foo')

        self.assertEqual(self.mfd.untick(), NodeMsg.IDLE)
        self.assertEqual(self.mfd.reset(), NodeMsg.IDLE)
        self.assertEqual(self.mfd.shutdown(), NodeMsg.SHUTDOWN)

    def testPopulateInvalidKey(self):
        self.mfd.inputs['dict'] = {'frob': 123}

        self.assertEqual(self.mfd.tick(), NodeMsg.FAILED)


class TestMessageFromConstDict(unittest.TestCase):
    def testPartialPopulate(self):
        mfd = MessageFromConstDict({'message_type': NodeMsg,
                                    'dict': {'name': 'foo'}})
        mfd.setup()

        self.assertEqual(mfd.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(mfd.outputs['message'].name, 'foo')

        self.assertEqual(mfd.untick(), NodeMsg.IDLE)
        self.assertEqual(mfd.reset(), NodeMsg.IDLE)
        self.assertEqual(mfd.shutdown(), NodeMsg.SHUTDOWN)

    def testPopulateInvalidKey(self):
        mfd = MessageFromConstDict({'message_type': NodeMsg,
                                    'dict': {'frob': 123}})
        mfd.setup()

        self.assertEqual(mfd.tick(), NodeMsg.FAILED)
