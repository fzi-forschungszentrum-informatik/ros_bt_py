import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg, Tree
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.ros_nodes.message_converters import MessageToFields, FieldsToMessage
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.node_data import NodeData


class TestMessageToFields(unittest.TestCase):
    def testPassthrough(self):
        converter = MessageToFields({'input_type': int})
        converter.setup()

        converter.inputs['in'] = 42

        converter.tick()
        self.assertEqual(converter.state, NodeMsg.SUCCEEDED)
        self.assertEqual(converter.outputs['out'], 42)

        converter.reset()
        self.assertEqual(converter.state, NodeMsg.IDLE)

        converter.untick()
        self.assertEqual(converter.state, NodeMsg.IDLE)

        converter.shutdown()
        self.assertEqual(converter.state, NodeMsg.SHUTDOWN)

    def testTreeMessage(self):
        converter = MessageToFields({'input_type': Tree})
        converter.setup()

        converter.inputs['in'] = Tree(name='foo', root_name='bar', tick_frequency_hz=42.0)

        converter.tick()
        self.assertEqual(converter.state, NodeMsg.SUCCEEDED)
        self.assertEqual(converter.outputs['name'], 'foo')
        self.assertEqual(converter.outputs['root_name'], 'bar')
        self.assertEqual(converter.outputs['tick_frequency_hz'], 42.0)

        converter.reset()
        self.assertEqual(converter.state, NodeMsg.IDLE)

        converter.untick()
        self.assertEqual(converter.state, NodeMsg.IDLE)

        converter.shutdown()
        self.assertEqual(converter.state, NodeMsg.SHUTDOWN)


class TestFieldsToMessage(unittest.TestCase):
    def testPassthrough(self):
        converter = FieldsToMessage({'output_type': int})
        converter.setup()

        converter.inputs['in'] = 42

        converter.tick()
        self.assertEqual(converter.state, NodeMsg.SUCCEEDED)
        self.assertEqual(converter.outputs['out'], 42)

        converter.reset()
        self.assertEqual(converter.state, NodeMsg.IDLE)

        converter.untick()
        self.assertEqual(converter.state, NodeMsg.IDLE)

        converter.shutdown()
        self.assertEqual(converter.state, NodeMsg.SHUTDOWN)

    def testTreeMessage(self):
        converter = FieldsToMessage({'output_type': Tree})
        converter.setup()

        converter.inputs['name'] = 'foo'
        converter.inputs['path'] = ''
        converter.inputs['root_name'] = 'bar'
        converter.inputs['nodes'] = []
        converter.inputs['data_wirings'] = []
        converter.inputs['tick_frequency_hz'] = 42.0
        converter.inputs['state'] = ''
        converter.inputs['public_node_data'] = []

        converter.tick()
        self.assertEqual(converter.state, NodeMsg.SUCCEEDED)
        self.assertEqual(
            converter.outputs['out'],
            Tree(name='foo', root_name='bar', tick_frequency_hz=42.0))

        converter.reset()
        self.assertEqual(converter.state, NodeMsg.IDLE)

        converter.untick()
        self.assertEqual(converter.state, NodeMsg.IDLE)

        converter.shutdown()
        self.assertEqual(converter.state, NodeMsg.SHUTDOWN)

    def testBrokenTreeMessage(self):
        converter = FieldsToMessage({'output_type': Tree})

        converter.setup()

        converter.inputs['name'] = 'foo'
        converter.inputs['path'] = ''
        converter.inputs['root_name'] = 'bar'
        converter.inputs['nodes'] = []
        converter.inputs['data_wirings'] = []
        converter.inputs['tick_frequency_hz'] = 42.0
        # break one of the inputs
        converter.inputs._map['tick_frequency_hzs'] = NodeData(data_type=int, initial_value=42)
        converter.inputs['state'] = ''
        converter.inputs['public_node_data'] = []

        converter.tick()
        self.assertEqual(converter.state, NodeMsg.FAILED)
