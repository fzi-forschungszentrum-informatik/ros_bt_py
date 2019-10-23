import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg, Tree
from std_msgs.msg import Duration, Header, Time

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

    def testHeaderMessage(self):
        converter = MessageToFields({'input_type': Header})

        converter.setup()

        converter.inputs['in'] = Header(frame_id='foo', seq=42)

        converter.tick()
        self.assertEqual(converter.state, NodeMsg.SUCCEEDED)
        self.assertEqual(converter.outputs['frame_id'], 'foo')
        self.assertEqual(converter.outputs['seq'], 42)

    def testDurationMessage(self):
        converter = MessageToFields({'input_type': Duration})

        converter.setup()

        duration = Duration()
        duration.data.secs = 23
        duration.data.nsecs = 42
        converter.inputs['in'] = duration

        converter.tick()
        self.assertEqual(converter.state, NodeMsg.SUCCEEDED)
        self.assertEqual(converter.outputs['data'].data.secs, 23)
        self.assertEqual(converter.outputs['data'].data.nsecs, 42)


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

    def testHeaderMessage(self):
        converter = FieldsToMessage({'output_type': Header})

        converter.setup()

        converter.inputs['stamp'] = Time()
        converter.inputs['frame_id'] = 'foo'
        converter.inputs['seq'] = 42

        converter.tick()
        self.assertEqual(converter.state, NodeMsg.SUCCEEDED)
        self.assertEqual(converter.outputs['out'].stamp.secs, 0)
        self.assertEqual(converter.outputs['out'].stamp.nsecs, 0)
        self.assertEqual(converter.outputs['out'].frame_id, 'foo')
        self.assertEqual(converter.outputs['out'].seq, 42)

    def testDurationMessage(self):
        converter = FieldsToMessage({'output_type': Duration})

        converter.setup()

        converter.inputs['data'] = Duration()

        converter.tick()
        self.assertEqual(converter.state, NodeMsg.SUCCEEDED)
        self.assertEqual(converter.outputs['out'].data.secs, 0)
        self.assertEqual(converter.outputs['out'].data.nsecs, 0)
