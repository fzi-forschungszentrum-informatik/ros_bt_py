#!/usr/bin/env python
# Copyright 2018-2023 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import argparse
from os import path

import rospy

from ros_bt_py_msgs.srv import LoadTree, LoadTreeRequest


def main():
    parser = argparse.ArgumentParser(description="Load a BT from a yaml file")

    parser.add_argument("pkg", nargs="?", help="package name")
    parser.add_argument(
        "tree_file",
        help=(
            "path of the YAMl file - if pkg is given, relative to package root, "
            "a regular path otherwise"
        ),
    )
    parser.add_argument(
        "--namespace",
        help='namespace of the tree services. defaults to "/tree_node"',
        default="/tree_node",
    )
    parser.add_argument("--timeout", type=float, default=2.0)

    # Filter out ROS arguments and pass everything but the filename to
    # argparse
    args = parser.parse_args(rospy.myargv()[1:])

    service_name = args.namespace.strip().rstrip("/") + "/load_tree"
    rospy.init_node("load_bt", anonymous=True)

    load_tree_proxy = rospy.ServiceProxy(service_name, LoadTree)
    load_tree_proxy.wait_for_service(args.timeout)

    req = LoadTreeRequest()
    if args.pkg:
        req.tree.path = "package://" + args.pkg.strip() + "/" + args.tree_file.strip()
    else:
        req.tree.path = "file://" + path.abspath(args.tree_file.strip())
    res = load_tree_proxy(req)

    if res.success:
        rospy.loginfo("Successfully loaded tree from path " + req.tree.path)
        return 0
    else:
        rospy.logerr(res.error_message)
        return 1


if __name__ == "__main__":
    main()
