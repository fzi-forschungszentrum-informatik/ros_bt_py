import unittest
from socket import error
import errno

import threading
from time import sleep

from tornado.testing import AsyncTestCase, gen_test, bind_unused_port
from tornado.httpclient import AsyncHTTPClient
from tornado.httpserver import HTTPServer

from web_server.web_server import WebServer, Package, CustomStaticFileHandler

try:
    import unittest.mock as mock
except ImportError:
    import mock


class TestWebServer(unittest.TestCase):
    def setUp(self):
        pass

    def testWebServerNoPackagesProvided(self):
        web_server = WebServer()

    def testWebServerPackagesProvided(self):
        web_server = WebServer(packages=[
            {
                "package": "web_server",
                "directory": "html"
            },
            {
                "package": "web_server",
            },
            {
                "package": "web_server",
                "directory": "html",
                "default_filename": "editor.html"
            },
            {
                "directory": "html"
            },
            {
            },
        ])

        expected_package = Package('web_server', '')

        repr_str = str(expected_package)

        self.assertEqual(len(web_server.packages), 3)
        self.assertEqual(expected_package.name, web_server.packages[0].name)

    @mock.patch('web_server.web_server.IOLoop')
    def testWebServerPackagesProvidedStartExceptions(self, mock_ioloop):
        web_server = WebServer(packages=[
            {
                "package": "web_server",
                "directory": "html"
            }
        ])

        web_server.application.listen = mock.MagicMock()
        web_server.application.listen.side_effect = error(errno.EADDRINUSE, '')

        web_server.start()
        self.assertIsNone(web_server.ioloop)

        web_server.application.listen = mock.MagicMock()
        web_server.application.listen.side_effect = error(errno.EACCES, '')

        web_server.start()
        self.assertIsNone(web_server.ioloop)

        web_server.application.listen = mock.MagicMock()
        web_server.application.listen.side_effect = error(errno.EIO, '')

        self.assertRaises(error, web_server.start)

        web_server.application.listen = mock.MagicMock()

        mock_ioloop.instance = mock.MagicMock()

        web_server.start()
        web_server.shutdown()

    def testCustomStaticFileHandler(self):
        handler = CustomStaticFileHandler(
            application=mock.MagicMock(), request=mock.MagicMock(), path='', package='')

        handler._transforms = []

        response = handler.write_error(status_code=500)


class TestRequests(AsyncTestCase):
    def setUp(self):
        super(TestRequests, self).setUp()

        self.web_server = WebServer(packages=[
            {
                "package": "web_server",
                "directory": "test",
            }
        ], cache_static_files=False)

        server = HTTPServer(self.web_server.application)
        socket, self.port = bind_unused_port()
        server.add_socket(socket)

    def tearDown(self):
        super(TestRequests, self).tearDown()

    @gen_test
    def test_http_fetch(self):
        client = AsyncHTTPClient()
        response = yield client.fetch(
            "http://127.0.0.1:" + str(self.port) + "/web_server/testdata/",
            raise_error=False)

        self.assertEqual(response.code, 200)

        self.assertIn(b"hello there", response.body)

        response = yield client.fetch(
            "http://127.0.0.1:" + str(self.port) + "/web_server/testdata/missing",
            raise_error=False)

        self.assertEqual(response.code, 404)

        response = yield client.fetch(
            "http://127.0.0.1:" + str(self.port), raise_error=False)

        self.assertEqual(response.code, 200)


class TestSinglePackageRequests(AsyncTestCase):
    def setUp(self):
        super(TestSinglePackageRequests, self).setUp()

        self.web_server = WebServer(packages=[
            {
                "package": "web_server",
                "directory": "test",
            }
        ], cache_static_files=False, serve_single_package_from_root=True)

        server = HTTPServer(self.web_server.application)
        socket, self.port = bind_unused_port()
        server.add_socket(socket)

    def tearDown(self):
        super(TestSinglePackageRequests, self).tearDown()

    @gen_test
    def test_http_fetch(self):
        client = AsyncHTTPClient()
        response = yield client.fetch(
            "http://127.0.0.1:" + str(self.port) + "/testdata/",
            raise_error=False)

        self.assertEqual(response.code, 200)

        self.assertIn(b"hello there", response.body)

        response = yield client.fetch(
            "http://127.0.0.1:" + str(self.port) + "/testdata/missing",
            raise_error=False)

        self.assertEqual(response.code, 404)

        response = yield client.fetch(
            "http://127.0.0.1:" + str(self.port) + "/web_server",
            raise_error=False)

        self.assertEqual(response.code, 404)


class TestRewriteRequests(AsyncTestCase):
    def setUp(self):
        super(TestRewriteRequests, self).setUp()

        self.web_server = WebServer(packages=[
            {
                "package": "web_server",
                "directory": "test",
                "default_filename": "testdata/index.html",
                "rewrite_requests_to_default_filename": True
            }
        ], cache_static_files=False)

        server = HTTPServer(self.web_server.application)
        socket, self.port = bind_unused_port()
        server.add_socket(socket)

    def tearDown(self):
        super(TestRewriteRequests, self).tearDown()

    @gen_test
    def test_http_fetch(self):
        client = AsyncHTTPClient()
        response = yield client.fetch(
            "http://127.0.0.1:" + str(self.port) + "/web_server/testdata/index.html",
            raise_error=False)

        self.assertEqual(response.code, 200)

        self.assertIn(b"hello there", response.body)

        response = yield client.fetch(
            "http://127.0.0.1:" + str(self.port) + "/web_server/testdata/missing",
            raise_error=False)

        self.assertEqual(response.code, 404)

        self.assertIn(b"hello there", response.body)

        response = yield client.fetch(
            "http://127.0.0.1:" + str(self.port) + "/web_server/testdata/second.html",
            raise_error=False)

        self.assertEqual(response.code, 200)

        self.assertIn(b"hi", response.body)


class TestSinglePackageRewriteRequests(AsyncTestCase):
    def setUp(self):
        super(TestSinglePackageRewriteRequests, self).setUp()

        self.web_server = WebServer(packages=[
            {
                "package": "web_server",
                "directory": "test",
                "default_filename": "testdata/index.html",
                "rewrite_requests_to_default_filename": True
            }
        ], cache_static_files=False, serve_single_package_from_root=True)

        server = HTTPServer(self.web_server.application)
        socket, self.port = bind_unused_port()
        server.add_socket(socket)

    def tearDown(self):
        super(TestSinglePackageRewriteRequests, self).tearDown()

    @gen_test
    def test_http_fetch(self):
        client = AsyncHTTPClient()
        response = yield client.fetch(
            "http://127.0.0.1:" + str(self.port) + "/testdata/index.html",
            raise_error=False)

        self.assertEqual(response.code, 200)

        self.assertIn(b"hello there", response.body)

        response = yield client.fetch(
            "http://127.0.0.1:" + str(self.port) + "/testdata/missing",
            raise_error=False)

        self.assertEqual(response.code, 404)

        self.assertIn(b"hello there", response.body)

        response = yield client.fetch(
            "http://127.0.0.1:" + str(self.port) + "/testdata/second.html",
            raise_error=False)

        self.assertEqual(response.code, 200)

        self.assertIn(b"hi", response.body)
