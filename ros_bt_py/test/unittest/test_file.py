import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.nodes.file import File, FileInput


class TestFile(unittest.TestCase):
    def setUp(self):
        # self.constant = Constant({'constant_type': int,
        #                           'constant_value': 42})
        # self.constant.setup()
        pass

    def testFileLoad(self):
        path = 'package://ros_bt_py/test/testdata/data/file_greetings.yaml'
        file_node = File(options={
            'file_path': path,
        })
        self.assertEqual(file_node.state, NodeMsg.UNINITIALIZED)
        file_node.setup()
        self.assertEqual(file_node.state, NodeMsg.IDLE)

        self.assertEqual(file_node.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(file_node.untick(), NodeMsg.IDLE)
        self.assertEqual(file_node.reset(), NodeMsg.IDLE)
        self.assertEqual(file_node.shutdown(), NodeMsg.SHUTDOWN)

    def testFileLoadMalformedPath(self):
        file_node = File(options={
            'file_path': 'malformed://ros_bt_py/etc/data/greetings.yaml',
        })
        self.assertEqual(file_node.state, NodeMsg.UNINITIALIZED)
        file_node.setup()
        self.assertEqual(file_node.state, NodeMsg.IDLE)
        self.assertEqual(file_node.data, None)
        self.assertEqual(file_node.tick(), NodeMsg.FAILED)

    def testFileLoadNotAvailable(self):
        file_node = File(options={
            'file_path': 'file://',
        })
        self.assertEqual(file_node.state, NodeMsg.UNINITIALIZED)
        file_node.setup()
        self.assertEqual(file_node.state, NodeMsg.IDLE)
        self.assertEqual(file_node.data, None)
        self.assertEqual(file_node.tick(), NodeMsg.FAILED)

    def testFileLoadMalformedContent(self):
        path = 'package://ros_bt_py/test/testdata/data/file_malformed.yaml'
        file_node = File(options={
            'file_path': path,
        })
        self.assertEqual(file_node.state, NodeMsg.UNINITIALIZED)
        file_node.setup()
        self.assertEqual(file_node.state, NodeMsg.IDLE)

        self.assertEqual(file_node.data, None)
        self.assertEqual(file_node.tick(), NodeMsg.FAILED)

    def testFileLoadMalformedList(self):
        path = 'package://ros_bt_py/test/testdata/data/file_malformed_list.yaml'
        file_node = File(options={
            'file_path': path,
        })
        self.assertEqual(file_node.state, NodeMsg.UNINITIALIZED)
        file_node.setup()
        self.assertEqual(file_node.state, NodeMsg.IDLE)

        self.assertEqual(file_node.data, None)
        self.assertEqual(file_node.tick(), NodeMsg.FAILED)

    # def testConstant(self):
    #     self.constant.tick()
    #     self.assertEqual(self.constant.state, NodeMsg.SUCCEEDED)
    #     self.assertEqual(self.constant.outputs['constant'], 42)

    #     self.constant.tick()
    #     self.assertEqual(self.constant.state, NodeMsg.SUCCEEDED)
    #     self.assertEqual(self.constant.outputs['constant'], 42)

    #     self.constant.reset()
    #     self.assertEqual(self.constant.state, NodeMsg.IDLE)

    #     self.constant.untick()
    #     self.assertEqual(self.constant.state, NodeMsg.IDLE)

    #     self.constant.shutdown()
    #     self.assertEqual(self.constant.state, NodeMsg.SHUTDOWN)


class TestFileInput(unittest.TestCase):
    def testFileLoad(self):
        path = 'package://ros_bt_py/test/testdata/data/file_greetings.yaml'
        file_node = FileInput()
        file_node.setup()

        file_node.inputs['file_path'] = path
        self.assertTrue(file_node.inputs.is_updated('file_path'))
        self.assertEqual(NodeMsg.SUCCEEDED, file_node.tick())

        self.assertEqual(file_node.outputs['load_success'], True)
        self.assertEqual(file_node.outputs['line_count'], 4)
        self.assertEqual(file_node.outputs['content'][2], 'Hola')

        self.assertEqual(file_node.untick(), NodeMsg.IDLE)
        self.assertEqual(file_node.reset(), NodeMsg.IDLE)
        self.assertEqual(file_node.shutdown(), NodeMsg.SHUTDOWN)

    def testFileLoadNotAvailable(self):
        path = 'file://'
        file_node = FileInput()
        file_node.setup()
        file_node.inputs['file_path'] = path
        self.assertEqual(NodeMsg.FAILED, file_node.tick())
        self.assertEqual(file_node.outputs['load_success'], False)

    def testFileLoadMalformedPath(self):
        path = 'malformed://ros_bt_py/etc/data/greetings.yaml'
        file_node = FileInput()
        file_node.setup()
        file_node.inputs['file_path'] = path
        self.assertEqual(NodeMsg.FAILED, file_node.tick())
        self.assertEqual(file_node.outputs['load_success'], False)
