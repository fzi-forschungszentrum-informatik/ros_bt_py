import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.nodes.setters import AppendListItem, SetAttr, SetDictItem


class TestAppendListItem(unittest.TestCase):
    def testAppendToEmptyList(self):
        append = AppendListItem({'list_type': int})
        append.setup()

        append.inputs['list'] = []
        append.inputs['value'] = 1

        self.assertEqual(append.tick(), NodeMsg.SUCCEEDED)
        self.assertTrue(append.outputs.is_updated('new_list'))
        self.assertListEqual(append.outputs['new_list'], [1])

        self.assertEqual(append.tick(), NodeMsg.SUCCEEDED)
        self.assertFalse(append.outputs.is_updated('new_list'))
        self.assertListEqual(append.outputs['new_list'], [1])

        append.inputs['value'] = 2

        self.assertEqual(append.tick(), NodeMsg.SUCCEEDED)
        self.assertTrue(append.outputs.is_updated('new_list'))
        self.assertListEqual(append.outputs['new_list'], [2])

        self.assertEqual(append.untick(), NodeMsg.IDLE)
        self.assertEqual(append.reset(), NodeMsg.IDLE)
        self.assertEqual(append.shutdown(), NodeMsg.SHUTDOWN)

    def testAppendToList(self):
        append = AppendListItem({'list_type': int})
        append.setup()

        append.inputs['list'] = [1, 2, 3]
        append.inputs['value'] = 4

        self.assertEqual(append.tick(), NodeMsg.SUCCEEDED)
        self.assertTrue(append.outputs.is_updated('new_list'))
        self.assertListEqual(append.outputs['new_list'], [1, 2, 3, 4])

        self.assertEqual(append.tick(), NodeMsg.SUCCEEDED)
        self.assertFalse(append.outputs.is_updated('new_list'))
        self.assertListEqual(append.outputs['new_list'], [1, 2, 3, 4])

        append.inputs['value'] = 42

        self.assertEqual(append.tick(), NodeMsg.SUCCEEDED)
        self.assertTrue(append.outputs.is_updated('new_list'))
        self.assertListEqual(append.outputs['new_list'], [1, 2, 3, 42])

    def testAppendWrongType(self):
        append = AppendListItem({'list_type': int})
        append.setup()

        with self.assertRaises(TypeError):
            append.inputs['list'] = [1, 2, 3]
            append.inputs['value'] = 'hello'

            append.tick()


class TestSetAttr(unittest.TestCase):
    def testSetCorrectAttr(self):
        set_attr = SetAttr({'object_type': NodeMsg,
                            'attr_name': 'name',
                            'attr_type': str})
        set_attr.setup()

        msg = NodeMsg()
        set_attr.inputs['object'] = msg
        set_attr.inputs['attr_value'] = 'foo'

        self.assertEqual(set_attr.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(set_attr.outputs['new_object'].name, 'foo')

        self.assertEqual(set_attr.untick(), NodeMsg.IDLE)
        self.assertEqual(set_attr.reset(), NodeMsg.IDLE)
        self.assertEqual(set_attr.shutdown(), NodeMsg.SHUTDOWN)

    def testSetNestedObjectAttr(self):
        class Inner(object):
            def __init__(self, name='inner_object'):
                self.name = name

        class Outer(object):
            def __init__(self):
                self.inner = Inner()
        set_attr = SetAttr({'object_type': Outer,
                            'attr_name': 'inner.name',
                            'attr_type': str})
        set_attr.setup()

        set_attr.inputs['object'] = Outer()
        set_attr.inputs['attr_value'] = 'updated_inner_object'

        self.assertEqual(set_attr.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(set_attr.outputs['new_object'].inner.name, 'updated_inner_object')

        self.assertEqual(set_attr.untick(), NodeMsg.IDLE)
        self.assertEqual(set_attr.reset(), NodeMsg.IDLE)
        self.assertEqual(set_attr.shutdown(), NodeMsg.SHUTDOWN)


class TestSetDictItem(unittest.TestCase):
    def testSetdictItem(self):
        set_dict_item = SetDictItem({'attr_name': 'name',
                                     'attr_type': str})

        set_dict_item.setup()

        set_dict_item.inputs['object'] = {'name': 'foo'}
        set_dict_item.inputs['attr_value'] = 'bar'

        self.assertEqual(set_dict_item.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(set_dict_item.outputs['new_object']['name'], 'bar')

        self.assertEqual(set_dict_item.untick(), NodeMsg.IDLE)
        self.assertEqual(set_dict_item.reset(), NodeMsg.IDLE)
        self.assertEqual(set_dict_item.shutdown(), NodeMsg.SHUTDOWN)
