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
import unittest

from ros_bt_py_msgs.msg import Node, UtilityBounds

from ros_bt_py.nodes.web import DownloadImage
from ros_bt_py.nodes.mock_nodes import MockLeaf

try:
    import unittest.mock as mock
except ImportError:
    import mock


class TestDownloadImage(unittest.TestCase):
    def setUp(self):
        self.image_raw = mock.mock_open()
        self.get_success = mock.MagicMock(status_code=200, raw=self.image_raw())
        self.get_failure = mock.MagicMock(status_code=404)

    @mock.patch("ros_bt_py.nodes.web.requests.get")
    def testDownloadAndSaveImage(self, mock_get):
        mock_get.return_value = self.get_success
        dl = DownloadImage({"cache_folder": "file:///toto"})
        dl.setup()
        dl.inputs["image_url"] = "www.foobar.com/picture.png"
        with mock.patch("ros_bt_py.nodes.web.open", mock.mock_open()) as mocked_file:
            self.assertEqual(dl.tick(), Node.SUCCEEDED)
            assert mocked_file.call_count == 1
            # test caching
            with mock.patch("os.path.exists") as mocked_exists:
                mocked_exists.return_value = True
                self.assertEqual(dl.tick(), Node.SUCCEEDED)
                assert mocked_exists.call_count == 1

        self.assertEqual(dl.outputs["download_error_msg"], "")
        self.assertTrue(dl.outputs["download_success"])
        self.assertTrue(dl.outputs["filepath"].startswith("/toto"))
        self.assertTrue(dl.outputs["filepath"].endswith(".png"))

        self.assertEqual(dl.untick(), Node.IDLE)
        self.assertEqual(dl.reset(), Node.IDLE)
        self.assertEqual(dl.shutdown(), Node.SHUTDOWN)

    @mock.patch("ros_bt_py.nodes.web.requests.get")
    def testDownloadFail(self, mock_get):
        mock_get.return_value = self.get_failure
        dl = DownloadImage({"cache_folder": "file:///toto"})
        dl.setup()
        dl.inputs["image_url"] = "www.foobar.com/picture.png"
        with mock.patch("ros_bt_py.nodes.web.open", mock.mock_open()) as mocked_file:
            self.assertEqual(dl.tick(), Node.FAILED)
            assert mocked_file.call_count == 0

        self.assertNotEqual(dl.outputs["download_error_msg"], "")
        self.assertFalse(dl.outputs["download_success"])
        self.assertEqual(dl.outputs["filepath"], "")

        self.assertEqual(dl.untick(), Node.IDLE)
        self.assertEqual(dl.reset(), Node.IDLE)
        self.assertEqual(dl.shutdown(), Node.SHUTDOWN)

    def testMalformedCacheFolder(self):
        dl = DownloadImage({"cache_folder": "/notareal.file"})
        dl.setup()
        self.assertFalse(dl.outputs["download_success"])
        dl.inputs["image_url"] = "www.foobar.com/picture.png"
        self.assertEqual(dl.tick(), Node.FAILED)

    def testPackageCacheFolder(self):
        dl = DownloadImage({"cache_folder": "package://ros_bt_py/cache"})
        dl.setup()
