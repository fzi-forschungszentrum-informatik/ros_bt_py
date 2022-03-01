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
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    version='',
    options={},
    inputs={},
    outputs={},
    max_children=0))
class NodeWithoutVersion(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0))
class NodeWithVersionButWithoutMigration(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0))
class NodeWithoutMigrationFunction(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0))
class NodeWithoutMigrationToFirstVersion(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='',
    options={},
    inputs={},
    outputs={},
    max_children=0))
class NodeWithoutVersionAndWithFirstMigration(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0))
class NodeWithBrokenMigrationPath(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0))
class NodeWithIdenticalVersions(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={'valid_option': int,
             'type_option': type},
    inputs={},
    outputs={'valid_output': int},
    max_children=0))
class NodeWithWorkingMigrations(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={'change_type': str},
    inputs={'change_type': str},
    outputs={'change_type': str},
    max_children=0))
class NodeWithWorkingMigrationsChangeType(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0))
class NodeWithWorkingMigrationsThatChangesTree(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0))
class NodeWithGetOptionException(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={'valid_option': int},
    inputs={},
    outputs={},
    max_children=0))
class NodeWithAddOptionException(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0))
class NodeWithAddInputException(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={},
    inputs={},
    outputs={'valid_output': int},
    max_children=0))
class NodeWithAddOutputException(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0))
class NodeWithOptionrefException(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
