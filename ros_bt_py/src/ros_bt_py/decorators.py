"""Decorators for ros_bt_py, the sorta type-safe BT library!
"""

from ros_bt_py import Node

# class decorator needs parameters:
# *inputs
# *outputs
# *options
# *max_children
# Generates an __init__ function that accepts values for all options
# error if any of these are missing
# set a special value that indicates the decorator has been applied
# in Node: check for that value (find a way that doesn't detect it if it's been added to the base class)
def DefineBTNode(node_class, node_config):
    for base in node_class.__bases__:
        if  hasattr(base, 'node_config'):
            node_config.extend(base.config)
    node_class.node_config = node_config
    return node_class
