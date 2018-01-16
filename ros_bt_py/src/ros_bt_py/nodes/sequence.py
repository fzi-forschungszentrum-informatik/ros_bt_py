from ros_bt_py.node import Node


class Sequence(Node):
    def __init__(self):
        super(Sequence, self).__init__()
        self.children = {}

        self.state = Node.States.IDLE
