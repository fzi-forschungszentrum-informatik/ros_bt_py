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

        max_iteration = 20
        def tick_until_child_tick():
            initial_count = run_success_fail.tick_count
            max_iteration = 20
            ii = 0

            iterate_tick = []
            while run_success_fail.tick_count != initial_count+1 and ii < max_iteration:
                # tick the iterator until it decides to tick its child (or until we give up)
                ii += 1
                token = iterate_list.tick()
                iterate_tick.append(token)
            self.assertLess(ii, max_iteration)
            return iterate_tick

        iterator_tokens = tick_until_child_tick()
        [ self.assertEqual(token, Node.RUNNING) for token in iterator_tokens ]
        self.assertEqual(iterate_list.outputs['list_item'], 'a')

        # child returned running so output did not change
        iterator_tokens = tick_until_child_tick()
        [ self.assertEqual(token, Node.RUNNING) for token in iterator_tokens ]
        self.assertEqual(iterate_list.outputs['list_item'], 'a')

        # child returned success so moved to next item in list
        iterator_tokens = tick_until_child_tick()
        self.assertEqual(iterate_list.outputs['list_item'], 'b')
        [ self.assertEqual(token, Node.RUNNING) for token in iterator_tokens ]

        # last tick is success when iteration is over
        self.assertEqual(iterate_list.tick(), Node.SUCCEEDED)

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
