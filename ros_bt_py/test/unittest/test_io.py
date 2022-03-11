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

from ros_bt_py_msgs.msg import Node as NodeMsg, Tree

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.nodes.io import IOInputOption, IOInput, IOOutputOption, IOOutput


class TestIOInputOption(unittest.TestCase):
    def setUp(self):
        self.default_tree = Tree(name='default_tree')
        self.input_tree = Tree(name='input_tree')

    def testWithoutInput(self):
        io = IOInputOption({'io_type': int,
                            'default': 42})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithIntInput(self):
        io = IOInputOption({'io_type': int,
                            'default': 42})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['in'] = 23

        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithMessageInput(self):
        io = IOInputOption({'io_type': Tree,
                            'default': self.default_tree})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.default_tree)

        io.inputs['in'] = self.input_tree
        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)


class TestIOInput(unittest.TestCase):
    def setUp(self):
        self.default_tree = Tree(name='default_tree')
        self.input_tree = Tree(name='input_tree')

    def testWithoutInputs(self):
        io = IOInput({'io_type': int})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)
        self.assertRaises(ValueError, io.tick)

    def testWithoutInputButWithDefault(self):
        io = IOInput({'io_type': int})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['default'] = 42

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithIntInput(self):
        io = IOInput({'io_type': int})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['default'] = 42
        io.inputs['in'] = 23

        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithMessageInput(self):
        io = IOInput({'io_type': Tree})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['default'] = self.default_tree

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.default_tree)

        io.inputs['in'] = self.input_tree
        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)


class TestIOOutputOption(unittest.TestCase):
    def setUp(self):
        self.default_tree = Tree(name='default_tree')
        self.input_tree = Tree(name='input_tree')

    def testWithoutInput(self):
        io = IOOutputOption({'io_type': int,
                             'default': 42})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithIntInput(self):
        io = IOOutputOption({'io_type': int,
                             'default': 42})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['in'] = 23

        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithMessageInput(self):
        io = IOOutputOption({'io_type': Tree,
                             'default': self.default_tree})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.default_tree)

        io.inputs['in'] = self.input_tree
        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)


class TestIOOutput(unittest.TestCase):
    def setUp(self):
        self.default_tree = Tree(name='default_tree')
        self.input_tree = Tree(name='input_tree')

    def testWithoutInputs(self):
        io = IOOutput({'io_type': int})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)
        self.assertRaises(ValueError, io.tick)

    def testWithoutInputButWithDefault(self):
        io = IOOutput({'io_type': int})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['default'] = 42

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithIntInput(self):
        io = IOOutput({'io_type': int})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['default'] = 42
        io.inputs['in'] = 23

        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithMessageInput(self):
        io = IOOutput({'io_type': Tree})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['default'] = self.default_tree

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.default_tree)

        io.inputs['in'] = self.input_tree
        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)
