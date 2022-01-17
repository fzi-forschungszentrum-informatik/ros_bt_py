#!/usr/bin/env python

import rospy

from web_server.web_server import WebServer


class WebServerNode(object):
    def __init__(self):
        rospy.loginfo('initializing web server node...')
        port = rospy.get_param('~port', default=8085)
        packages = rospy.get_param('~packages', default=[])
        cache_static_files = rospy.get_param('~cache_static_files', default=True)
        serve_single_package_from_root = rospy.get_param(
            '~serve_single_package_from_root', default=False)

        if not isinstance(packages, list):
            raise TypeError('packages must be a list, but is a %s' %
                            type(packages.__name__))

        self.web_server = WebServer(
            port=port,
            packages=packages,
            cache_static_files=cache_static_files,
            serve_single_package_from_root=serve_single_package_from_root)

        rospy.loginfo(
            'initialized web server node, listening on port: "%d"' % self.web_server.port)

    def start(self):
        self.web_server.start()

    def shutdown(self):
        self.web_server.shutdown()


if __name__ == '__main__':
    rospy.init_node('web_server_node')

    node = WebServerNode()

    rospy.on_shutdown(node.shutdown)

    node.start()
