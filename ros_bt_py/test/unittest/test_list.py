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


class TestIterateList(unittest.TestCase):
    def testIterate(self):
        iterate_list = IterateList({
            'item_type': str
        })

        list_provider = MockLeaf(
            options={'output_type': int,
                     'state_values': [Node.RUNNING,
                                      Node.SUCCEEDED,
                                      Node.FAILED],
                     'output_values': [0, 1, 2]})
        iterate_list.add_child(list_provider)

        iterate_list.inputs['list'] = ['a', 'b']

        iterate_list.setup()

        # input just changed - child are not ticked but input is set
        self.assertEqual(iterate_list.tick(), Node.RUNNING)
        self.assertEqual(iterate_list.outputs['list_item'], 'a')
        self.assertEqual(list_provider.state, Node.IDLE)

        # first iteration: childs are ticked
        self.assertEqual(iterate_list.tick(), Node.RUNNING)
        self.assertEqual(iterate_list.outputs['list_item'], 'a')
        self.assertEqual(list_provider.state, Node.RUNNING)

        # child returned running so first iteration is still not over
        self.assertEqual(iterate_list.tick(), Node.RUNNING)
        self.assertEqual(iterate_list.outputs['list_item'], 'a')
        self.assertEqual(list_provider.state, Node.SUCCEEDED)

        # First iteration is over, output changed, childs aren't ticked
        self.assertEqual(iterate_list.tick(), Node.RUNNING)
        self.assertEqual(iterate_list.outputs['list_item'], 'b')

        # second and iteration
        self.assertEqual(iterate_list.tick(), Node.RUNNING)
        self.assertEqual(iterate_list.outputs['list_item'], 'b')
        self.assertEqual(list_provider.state, Node.FAILED)

        # iteration is over
        self.assertEqual(iterate_list.tick(), Node.SUCCEEDED)
