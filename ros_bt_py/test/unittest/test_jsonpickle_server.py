import jsonpickle
import unittest

from ros_bt_py_msgs.msg import Node
from ros_bt_py_msgs.srv import GetJsonpickleInstanceRequest as Request
from ros_bt_py.jsonpickle_server import get_jsonpickle_instance as service


class TestJsonpickleServer(unittest.TestCase):
    def getInvalidType(self):
        req = Request(serialized_type='not a real type')
        res = service(req)

        self.assertFalse(res.success)

    def getBasicType(self):
        types = [int, float, str, list, dict, set]

        for basic_type in types:
            req = Request(serialized_type=jsonpickle.encode(basic_type))
            res = service(req)

            self.assertTrue(res.success)
            self.assertEqual(jsonpickle.decode(res.serialized_instance), basic_type)

    def getRosMessageType(self):
        req = Request(serialized_type=jsonpickle.encode(Node))
        res = service(req)

        self.assertTrue(res.success)
        self.assertEqual(jsonpickle.decode(res.serialized_instance), Node)
