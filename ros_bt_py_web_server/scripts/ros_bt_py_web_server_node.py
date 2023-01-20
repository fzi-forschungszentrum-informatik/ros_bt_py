#!/usr/bin/env python
#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022 FZI Forschungszentrum Informatik
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
#    * Neither the name of the {copyright_holder} nor the names of its
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
#  -------- END LICENSE BLOCK --------

import rospy

from ros_bt_py_web_server.web_server import WebServer


class WebServerNode(object):
    def __init__(self):
        rospy.loginfo("initializing ros_bt_py web server node...")
        port = rospy.get_param("~port", default=8085)
        packages = rospy.get_param("~packages", default=[])
        cache_static_files = rospy.get_param("~cache_static_files", default=True)
        serve_single_package_from_root = rospy.get_param(
            "~serve_single_package_from_root", default=False
        )

        if not isinstance(packages, list):
            raise TypeError(
                "packages must be a list, but is a %s" % type(packages.__name__)
            )

        self.web_server = WebServer(
            port=port,
            packages=packages,
            cache_static_files=cache_static_files,
            serve_single_package_from_root=serve_single_package_from_root,
        )

        rospy.loginfo(
            'initialized ros_bt_py web server node, listening on port: "%d"'
            % self.web_server.port
        )

    def start(self):
        self.web_server.start()

    def shutdown(self):
        self.web_server.shutdown()


if __name__ == "__main__":
    rospy.init_node("ros_bt_py_web_server_node")

    node = WebServerNode()

    rospy.on_shutdown(node.shutdown)

    node.start()
