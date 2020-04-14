import unittest

from ros_bt_py_msgs.msg import Node, UtilityBounds
from ros_bt_py_msgs.msg import NodeDataWiring
from ros_bt_py_msgs.msg import NodeDataLocation

from ros_bt_py.nodes.list import ListLength, IsInList, IterateList,\
    InsertInList, GetListElementOption
from ros_bt_py.nodes.mock_nodes import MockLeaf
from ros_bt_py.nodes.compare import CompareConstant
from ros_bt_py.nodes.decorators import IgnoreFailure

try:
    import unittest.mock as mock
except ImportError:
    import mock


class TestListLength(unittest.TestCase):
    def testListLength(self):
        list_length = ListLength()
        list_length.setup()
        list_length.inputs['list'] = ['a', 'b', 'c']
        self.assertEqual(list_length.tick(), Node.SUCCEEDED)
        self.assertEqual(list_length.outputs['length'], 3)

        self.assertEqual(list_length.untick(), Node.IDLE)
        self.assertEqual(list_length.reset(), Node.IDLE)
        self.assertEqual(list_length.shutdown(), Node.SHUTDOWN)


class TestIsInList(unittest.TestCase):
    def testIsInList(self):
        in_list = IsInList({
            'compare_type': str,
            'list': ['a', 'b', 'c']
        })

        in_list.setup()
        in_list.inputs['in'] = 'not in list'
        self.assertEqual(in_list.tick(), Node.FAILED)
        in_list.inputs['in'] = 'a'
        self.assertEqual(in_list.tick(), Node.SUCCEEDED)

        self.assertEqual(in_list.untick(), Node.IDLE)
        self.assertEqual(in_list.reset(), Node.IDLE)
        self.assertFalse(in_list._received_in)
        self.assertEqual(in_list.shutdown(), Node.SHUTDOWN)


class TestGetListElementOption(unittest.TestCase):
    def testGetOption(self):
        get_elem = GetListElementOption({
            'element_type': str,
            'index': 2
        })
        get_elem.setup()
        get_elem.inputs['list'] = ['nop', 'no', 'yep']

        self.assertEqual(get_elem.tick(), Node.SUCCEEDED)
        self.assertEqual(get_elem.outputs['element'], 'yep')

    def testOutOfRange(self):
        get_elem = GetListElementOption({
            'element_type': str,
            'index': 5
        })
        get_elem.setup()
        get_elem.inputs['list'] = ['nop', 'no', 'yep']

        self.assertRaises(IndexError, get_elem.tick)

    def testWrongType(self):
        get_elem = GetListElementOption({
            'element_type': int,
            'index': 2
        })
        get_elem.setup()
        get_elem.inputs['list'] = ['no', 'int', 'toto']
        self.assertRaises(TypeError, get_elem.tick)


class TestInsertInList(unittest.TestCase):
    def testInsert(self):
        insert = InsertInList({
            'element_type': str,
            'index': 2
        })
        insert.setup()
        insert.inputs['list'] = ['first', 'before', 'after', 'last']
        insert.inputs['element'] = 'here'

        self.assertEqual(insert.tick(), Node.SUCCEEDED)
        self.assertEqual(insert.outputs['list'][2], 'here')
        self.assertEqual(insert.outputs['list'][1], 'before')
        self.assertEqual(insert.outputs['list'][3], 'after')

    def testOutOfRangePositive(self):
        insert = InsertInList({
            'element_type': str,
            'index': 10
        })
        insert.setup()

        # index > len(list) insert at last position
        insert.inputs['list'] = ['first', 'before', 'after', 'last']
        insert.inputs['element'] = 'here'
        self.assertEqual(insert.tick(), Node.SUCCEEDED)
        self.assertEqual(insert.outputs['list'][-1], 'here')

    def testOutOfRangeNegative(self):
        insert = InsertInList({
            'element_type': str,
            'index': -10
        })
        insert.setup()

        # index > len(list) insert at last position
        insert.inputs['list'] = ['first', 'before', 'after', 'last']
        insert.inputs['element'] = 'here'
        self.assertEqual(insert.tick(), Node.SUCCEEDED)
        self.assertEqual(insert.outputs['list'][0], 'here')


class TestIterateList(unittest.TestCase):
    def setUp(self):
        self.iterate = IterateList({
            'item_type': str
        })

        self.compare = CompareConstant({
            'compare_type': str,
            'expected': 'toto'
        })
        self.tick_count = mock.Mock(wraps=self.compare._do_tick)
        self.compare._do_tick = self.tick_count

        self.ignore_failure = IgnoreFailure()

    def connect_compare(self):
        self.compare.wire_data(NodeDataWiring(
            source=NodeDataLocation(
                node_name=self.iterate.name,
                data_kind=NodeDataLocation.OUTPUT_DATA,
                data_key='list_item'
            ),
            target=NodeDataLocation(
                node_name=self.compare.name,
                data_kind=NodeDataLocation.INPUT_DATA,
                data_key='in'
            )))

    def tick_until_compare_tick(self):
        initial_count = self.tick_count.call_count
        max_iteration = 20
        ii = 0

        last_tick = Node.RUNNING
        while self.tick_count.call_count != initial_count + 1 and ii < max_iteration:
            # tick the iterator until it decides to tick its compare (or until we give up)
            ii += 1
            self.assertEqual(last_tick, Node.RUNNING)
            last_tick = self.iterate.tick()
        self.assertLess(ii, max_iteration)
        return last_tick

    def testIterateWithChildSuccessInput(self):
        self.iterate.add_child(self.ignore_failure)
        self.ignore_failure.add_child(self.compare)
        self.connect_compare()

        self.iterate.inputs['list'] = ['some', 'ignored', 'string']
        self.iterate.setup()

        token = self.tick_until_compare_tick()
        self.assertEqual(token, Node.RUNNING)
        self.assertEqual(self.compare.inputs['in'], 'some')

        token = self.tick_until_compare_tick()
        self.assertEqual(token, Node.RUNNING)
        self.assertEqual(self.compare.inputs['in'], 'ignored')

        token = self.tick_until_compare_tick()
        self.assertEqual(token, Node.SUCCEEDED)
        self.assertEqual(self.compare.inputs['in'], 'string')

    def testIterateWithChildFailInput(self):
        self.iterate.add_child(self.compare)
        self.connect_compare()

        self.iterate.inputs['list'] = ['toto', 'fail']
        self.iterate.setup()

        token = self.tick_until_compare_tick()
        self.assertEqual(token, Node.RUNNING)
        self.assertEqual(self.compare.inputs['in'], 'toto')

        token = self.tick_until_compare_tick()
        self.assertEqual(token, Node.FAILED)
        self.assertEqual(self.compare.inputs['in'], 'fail')

    def testIterateWithChildRunningInput(self):
        self.iterate.add_child(self.compare)
        self.connect_compare()
        self.tick_count.side_effect = [
            Node.RUNNING, Node.SUCCEEDED, Node.RUNNING, Node.SUCCEEDED]

        self.iterate.inputs['list'] = ['ignored', 'bymock']
        self.iterate.setup()

        token = self.tick_until_compare_tick()
        self.assertEqual(token, Node.RUNNING)
        self.assertEqual(self.compare.inputs['in'], 'ignored')

        # child returned running - input did not change
        token = self.tick_until_compare_tick()
        self.assertEqual(token, Node.RUNNING)
        self.assertEqual(self.compare.inputs['in'], 'ignored')

        token = self.tick_until_compare_tick()
        self.assertEqual(token, Node.RUNNING)
        self.assertEqual(self.compare.inputs['in'], 'bymock')

        # child returned running - input did not change
        token = self.tick_until_compare_tick()
        self.assertEqual(token, Node.SUCCEEDED)
        self.assertEqual(self.compare.inputs['in'], 'bymock')

    def testIterateWithoutChild(self):
        self.iterate.inputs['list'] = ['a', 'b']

        self.iterate.setup()

        self.assertEqual(self.iterate.tick(), Node.RUNNING)
        self.assertEqual(self.iterate.outputs['list_item'], 'a')
        self.assertEqual(self.iterate.tick(), Node.SUCCEEDED)
        self.assertEqual(self.iterate.outputs['list_item'], 'b')

        self.assertEqual(self.iterate.untick(), Node.IDLE)
        self.assertEqual(self.iterate.reset(), Node.IDLE)
        self.assertEqual(self.iterate.shutdown(), Node.SHUTDOWN)
