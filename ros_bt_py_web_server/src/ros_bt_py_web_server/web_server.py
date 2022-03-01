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
import rospkg
from socket import error
import errno

from tornado.web import Application, RedirectHandler, RequestHandler, StaticFileHandler
from tornado.ioloop import IOLoop


class Package(object):
    def __init__(self, name, absolute_path, subdirectory='', default_filename='index.html',
                 rewrite_requests_to_default_filename=False):
        self.name = name
        self.absolute_path = absolute_path
        self.subdirectory = subdirectory
        self.default_filename = default_filename
        self.rewrite_requests_to_default_filename = rewrite_requests_to_default_filename

    def __repr__(self):
        return \
            'Package(name=%s, absolute_path=%s, subdirectory=%s, default_filename=%s, ' \
            'rewrite_requests_to_default_filename=%s)' % (
                self.name,
                self.absolute_path,
                self.subdirectory,
                self.default_filename,
                self.rewrite_requests_to_default_filename)


class CustomStaticFileHandler(StaticFileHandler):
    def initialize(self, path, package, subdirectory='',
                   default_filename=None, cache_static_files=True,
                   rewrite_requests_to_default_filename=False):
        self.root = path
        self.default_filename = default_filename
        self.cache_static_files = cache_static_files
        self.package = package
        self.subdirectory = subdirectory
        self.rewrite_requests_to_default_filename = rewrite_requests_to_default_filename

    def write_error(self, status_code, *args, **kwargs):
        if status_code == 404:
            if self.rewrite_requests_to_default_filename:
                self.render(self.root + '/' + self.default_filename)
            else:
                file_path = self.default_filename
                if self.subdirectory:
                    file_path = self.subdirectory + '/' + file_path
                self.render("templates/404.html", file_path=file_path, package=self.package)
        else:
            super(CustomStaticFileHandler, self).write_error(status_code, *args, **kwargs)

    def set_extra_headers(self, path):
        if not self.cache_static_files:
            # disable cache for multiple browsers
            self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')


class PackageListRequestHandler(RequestHandler):
    def initialize(self, packages):
        self.packages = packages

    def get(self):
        self.render("templates/package_list.html", packages=self.packages)


class WebServer(object):
    '''Provides a simple tornado based web server
    '''
    def __init__(self,
                 port=8085,
                 packages=[],
                 cache_static_files=True,
                 serve_single_package_from_root=False):
        self.port = port
        self.rospack = rospkg.RosPack()
        self.packages = []
        self.cache_static_files = cache_static_files
        self.serve_single_package_from_root = serve_single_package_from_root
        self.ioloop = None

        if packages:
            # only provide the given packages
            for package in packages:
                if 'directory' not in package:
                    subdirectory = ''
                else:
                    subdirectory = package['directory']
                if 'default_filename' not in package:
                    default_filename = 'index.html'
                else:
                    default_filename = package['default_filename']
                if 'rewrite_requests_to_default_filename' not in package:
                    rewrite_requests_to_default_filename = False
                else:
                    rewrite_requests_to_default_filename = \
                        package['rewrite_requests_to_default_filename']
                if 'package' not in package:
                    rospy.logwarn('[web_server]: No "package" key in provided package config:'
                                  '"%s"' % package)
                else:
                    package_path = self.rospack.get_path(package['package'])
                    self.packages.append(Package(
                        name=package['package'],
                        absolute_path=package_path,
                        subdirectory=subdirectory,
                        default_filename=default_filename,
                        rewrite_requests_to_default_filename=rewrite_requests_to_default_filename))
        else:
            # provide all available packages
            packages = self.rospack.list()

            for package in packages:
                package_path = self.rospack.get_path(package)

                self.packages.append(Package(name=package, absolute_path=package_path))

        handlers = []
        if len(self.packages) == 1 and self.serve_single_package_from_root:
            package = self.packages[0]
            handler = (r'/(.*)|/',
                       CustomStaticFileHandler,
                       {'path': package.absolute_path + '/' + package.subdirectory,
                        'package': package.name,
                        'subdirectory': package.subdirectory,
                        'default_filename': package.default_filename,
                        'cache_static_files': self.cache_static_files,
                        'rewrite_requests_to_default_filename':
                            package.rewrite_requests_to_default_filename})
            handlers.append(handler)
        else:
            handlers = [('/', PackageListRequestHandler, {'packages': self.packages})]

            for package in self.packages:
                handler = ('/' + package.name,
                           RedirectHandler,
                           {'url': '/' + package.name + '/'})
                handlers.append(handler)
                handler = (r'/' + package.name + r'/(.*)|/' + package.name,
                           CustomStaticFileHandler,
                           {'path': package.absolute_path + '/' + package.subdirectory,
                            'package': package.name,
                            'subdirectory': package.subdirectory,
                            'default_filename': package.default_filename,
                            'cache_static_files': self.cache_static_files,
                            'rewrite_requests_to_default_filename':
                                package.rewrite_requests_to_default_filename})
                handlers.append(handler)

        self.application = Application(handlers)

    def start(self):
        try:
            self.application.listen(self.port)

            self.ioloop = IOLoop.instance()
            self.ioloop.start()
        except error as socket_error:
            if socket_error.errno is errno.EADDRINUSE:
                rospy.logerr('[web_server]: Port: "%d": %s' % (
                    self.port, str(socket_error)))
            elif socket_error.errno is errno.EACCES:
                rospy.logerr('[web_server]: Port: "%d": %s' % (
                    self.port, str(socket_error)))
            else:
                raise socket_error

    def shutdown(self):
        if self.ioloop:
            self.ioloop.stop()
