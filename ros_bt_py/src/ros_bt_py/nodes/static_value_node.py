from ros_bt_py.node import Node


class StaticValueNode(Node):
    def __init__(self):
        super(StaticValueNode, self).__init__()
        self._register_outputs({'out', 
