import unittest

from ros_bt_py_msgs.msg import Node, UtilityBounds

from ros_bt_py.nodes.list import ListLength, IsInList, IterateList
from ros_bt_py.nodes.mock_nodes import MockLeaf


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


class TestIterateList(unittest.TestCase):
    def testIterateWithChild(self):
        iterate_list = IterateList({
            'item_type': str
        })

        run_success_fail = MockLeaf(
            options={'output_type': int,
                     'state_values': [Node.RUNNING,
                                      Node.SUCCEEDED,
                                      Node.FAILED],
                     'output_values': [0, 1, 2]})
        iterate_list.add_child(run_success_fail)

        iterate_list.inputs['list'] = ['a', 'b']

        iterate_list.setup()

        self.assertEqual(iterate_list.tick(), Node.RUNNING)
        self.assertEqual(iterate_list.outputs['list_item'], 'a')
        self.assertEqual(run_success_fail.tick_count, 1)

        self.assertEqual(iterate_list.tick(), Node.RUNNING)
        self.assertEqual(iterate_list.outputs['list_item'], 'a')
        self.assertEqual(run_success_fail.tick_count, 2)

        self.assertEqual(iterate_list.tick(), Node.RUNNING)
        self.assertEqual(iterate_list.outputs['list_item'], 'b')
        self.assertEqual(run_success_fail.tick_count, 3)

        # iteration is over
        self.assertEqual(iterate_list.tick(), Node.SUCCEEDED)
        self.assertEqual(iterate_list.outputs['list_item'], 'b')
        self.assertEqual(run_success_fail.tick_count, 3)

        self.assertEqual(iterate_list.untick(), Node.PAUSED)
        self.assertEqual(iterate_list.children[0].state, Node.PAUSED)
        self.assertEqual(iterate_list.reset(), Node.IDLE)
        self.assertEqual(iterate_list.children[0].state, Node.IDLE)
        self.assertEqual(iterate_list.shutdown(), Node.SHUTDOWN)
        self.assertEqual(iterate_list.children[0].state, Node.SHUTDOWN)

    def testIterateWithoutChild(self):
        iterate_list = IterateList({
            'item_type': str
        })

        iterate_list.inputs['list'] = ['a', 'b']

        iterate_list.setup()

        self.assertEqual(iterate_list.tick(), Node.RUNNING)
        self.assertEqual(iterate_list.outputs['list_item'], 'a')
        self.assertEqual(iterate_list.tick(), Node.RUNNING)
        self.assertEqual(iterate_list.outputs['list_item'], 'b')
        self.assertEqual(iterate_list.tick(), Node.SUCCEEDED)

        self.assertEqual(iterate_list.untick(), Node.IDLE)
        self.assertEqual(iterate_list.reset(), Node.IDLE)
        self.assertEqual(iterate_list.shutdown(), Node.SHUTDOWN)
