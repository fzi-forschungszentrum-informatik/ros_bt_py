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

    @mock.patch('ros_bt_py.nodes.web.requests.get')
    def testDownloadAndSaveImage(self, mock_get):
        mock_get.return_value = self.get_success
        dl = DownloadImage({'cache_folder': 'file:///toto'})
        dl.setup()
        dl.inputs['image_url'] = 'www.foobar.com/picture.png'
        with mock.patch('ros_bt_py.nodes.web.open', mock.mock_open()) as mocked_file:
            self.assertEqual(dl.tick(), Node.SUCCEEDED)
            mocked_file.assert_called_once()

        self.assertEqual(dl.outputs['download_error_msg'], '')
        self.assertTrue(dl.outputs['download_success'])
        self.assertTrue(dl.outputs['filepath'].startswith('/toto'))
        self.assertTrue(dl.outputs['filepath'].endswith('.png'))

        self.assertEqual(dl.untick(), Node.IDLE)
        self.assertEqual(dl.reset(), Node.IDLE)
        self.assertEqual(dl.shutdown(), Node.SHUTDOWN)

    @mock.patch('ros_bt_py.nodes.web.requests.get')
    def testDownloadFail(self, mock_get):
        mock_get.return_value = self.get_failure
        dl = DownloadImage({'cache_folder': 'file:///toto'})
        dl.setup()
        dl.inputs['image_url'] = 'www.foobar.com/picture.png'
        with mock.patch('ros_bt_py.nodes.web.open', mock.mock_open()) as mocked_file:
            self.assertEqual(dl.tick(), Node.FAILED)
            mocked_file.assert_not_called()

        self.assertNotEqual(dl.outputs['download_error_msg'], '')
        self.assertFalse(dl.outputs['download_success'])
        self.assertEqual(dl.outputs['filepath'], '')

        self.assertEqual(dl.untick(), Node.IDLE)
        self.assertEqual(dl.reset(), Node.IDLE)
        self.assertEqual(dl.shutdown(), Node.SHUTDOWN)
