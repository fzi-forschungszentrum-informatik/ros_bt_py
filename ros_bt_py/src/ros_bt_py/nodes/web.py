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
import rospkg
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

import requests
import shutil
import os
import hashlib


@define_bt_node(NodeConfig(
    options={'cache_folder': str},
    inputs={'image_url': str},
    outputs={
        'filepath': str,
        'download_success': bool,
        'download_error_msg': str
    },
    max_children=0))
class DownloadImage(Leaf):
    """Download an image from a URL and store it on the filesystem.
    """
    def _do_setup(self):
        self.output_filepath = ''
        try:
            self.output_path = self.expend_path(self.options['cache_folder'])
        except ValueError as exc:
            self.output_path = None
            self.outputs['download_success'] = False
            self.outputs['download_error_msg'] = (
                'File path "%s" is malformed. It needs to start with '
                'either "file://" or "package://"') % self.inputs['image_url']
            self.logerr(self.outputs['download_error_msg'])

    def _do_tick(self):
        if self.output_path is None:
            return NodeMsg.FAILED

        if self.inputs.is_updated('image_url'):
            # update the filepath
            _, file_extension = os.path.splitext(self.inputs['image_url'])
            output_filename = hashlib.md5(
                self.inputs['image_url'].encode()).hexdigest() + file_extension

            self.output_filepath = os.path.join(self.output_path, output_filename)

        self.outputs['filepath'] = self.output_filepath

        if self.output_filepath != '' and os.path.exists(self.output_filepath):
            # image already cached
            self.outputs['download_success'] = True
            self.outputs['download_error_msg'] = ''
            return NodeMsg.SUCCEEDED
        else:
            # download image
            r = requests.get(url=self.inputs['image_url'], stream=True)
            if r.status_code != 200:
                self.outputs['download_success'] = False
                self.outputs['download_error_msg'] = (
                    'Could not download image, http error code: %s' % (r.status_code))
                self.outputs['filepath'] = ''
                return NodeMsg.FAILED

        with open(self.output_filepath, 'wb') as f:
            r.raw.decode_content = True
            shutil.copyfileobj(r.raw, f)

        self.outputs['download_success'] = True
        self.outputs['download_error_msg'] = ''
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def expend_path(self, path):
        rospack = rospkg.RosPack()
        file_path = ''
        if path.startswith('file://'):
            file_path = path[len('file://'):]
        elif path.startswith('package://'):
            package_name = path[len('package://'):].split('/', 1)[0]
            package_path = rospack.get_path(package_name)
            file_path = package_path + path[len('package://') + len(package_name):]
        else:
            raise ValueError("Malformed path")
        return file_path
