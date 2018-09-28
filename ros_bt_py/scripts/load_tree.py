#! /usr/bin/env python2.7

import argparse
from os import path

import rospy

from ros_bt_py_msgs.srv import LoadTree, LoadTreeRequest


def main():
    parser = argparse.ArgumentParser(description='Load a BT from a yaml file')

    parser.add_argument(
        'pkg',
        nargs='?',
        help='package name')
    parser.add_argument(
        'tree_file',
        help=('path of the YAMl file - if pkg is given, relative to package root, '
              'a regular path otherwise'))
    parser.add_argument(
        '--namespace',
        help='namespace of the tree services. defaults to "/tree_node"',
        default='/tree_node')
    parser.add_argument(
        '--timeout',
        type=float,
        default=2.0)

    # Filter out ROS arguments and pass everything but the filename to
    # argparse
    args = parser.parse_args(rospy.myargv()[1:])

    service_name = args.namespace.strip().rstrip('/') + '/load_tree'
    rospy.init_node('load_bt', anonymous=True)

    load_tree_proxy = rospy.ServiceProxy(service_name, LoadTree)
    load_tree_proxy.wait_for_service(args.timeout)

    req = LoadTreeRequest()
    if args.pkg:
        req.tree.path = 'package://' + args.pkg.strip() + '/' + args.tree_file.strip()
    else:
        req.tree.path = 'file://' + path.abspath(args.tree_file.strip())
    res = load_tree_proxy(req)

    if res.success:
        rospy.loginfo('Successfully loaded tree from path ' + req.tree.path)
        return 0
    else:
        rospy.logerr(res.error_message)
        return 1


if __name__ == '__main__':
    main()
