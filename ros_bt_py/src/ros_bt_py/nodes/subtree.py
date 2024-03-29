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


"""BT node to encapsulate a part of a tree in a reusable subtree."""
from typing import Optional, Dict

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds, Tree, NodeDataLocation
from ros_bt_py_msgs.srv import LoadTreeRequest

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.tree_manager import TreeManager, get_success, get_error_message
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={"subtree_path": str, "use_io_nodes": bool},
        inputs={},
        outputs={"load_success": bool, "load_error_msg": str},
        max_children=0,
        optional_options=["use_io_nodes"],
    )
)
class Subtree(Leaf):
    """Loads a subtree from the location pointed to by `subtree_uri`.

    This is the only node that modifies its `node_config` member - it
    will populate its inputs and outputs when its constructor is
    called, based on the public inputs and outputs of the subtree.

    Please note that it is **NOT** possible to have public *option*
    values. Since they can affect the types of inputs/outputs, they
    could only feasibly be set in the Subtree node's own options, but
    at that point we don't know their names or types yet.
    """

    def __init__(  # noqa: C901
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: str = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        """Create the tree manager, load the subtree."""
        super(Subtree, self).__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        self.root = None
        self.prefix = f"{self.name}."
        # since the subtree gets a prefix, we can just have it use the
        # parent debug manager
        self.manager = TreeManager(name=name, debug_manager=debug_manager)

        response = self.manager.load_tree(
            request=LoadTreeRequest(tree=Tree(path=self.options["subtree_path"])),
            prefix=self.prefix,
        )

        if not get_success(response):
            self.outputs["load_success"] = False
            self.outputs["load_error_msg"] = get_error_message(response)
            return

        self.outputs["load_success"] = True

        # If we loaded the tree successfully, change node_config to
        # include the public inputs and outputs
        subtree_inputs = {}
        subtree_outputs = {}

        # If io nodes are used restrict the subtrees inputs and outputs to io nodes
        io_inputs = []
        io_outputs = []
        subtree_msg = self.manager.to_msg()
        if options.get("use_io_nodes"):
            for node in subtree_msg.nodes:
                if node.module == "ros_bt_py.nodes.io":
                    if (
                        node.node_class == "IOInput"
                        or node.node_class == "IOInputOption"
                    ):
                        io_inputs.append(node.name)
                    elif (
                        node.node_class == "IOOutput"
                        or node.node_class == "IOOutputOption"
                    ):
                        io_outputs.append(node.name)
            modified_public_node_data = []
            for node_data in subtree_msg.public_node_data:
                if node_data.data_kind == "inputs" and node_data.node_name in io_inputs:
                    modified_public_node_data.append(node_data)
                elif (
                    node_data.data_kind == "outputs"
                    and node_data.node_name in io_outputs
                ):
                    modified_public_node_data.append(node_data)
            subtree_msg.public_node_data = modified_public_node_data

        for node_data in subtree_msg.public_node_data:
            # Remove the prefix from the node name to make for nicer
            # input/output names (and also not break wirings)
            node_name = node_data.node_name
            if node_name.startswith(self.prefix):
                node_name = node_name[len(self.prefix) :]

            if node_data.data_kind == NodeDataLocation.INPUT_DATA:
                subtree_inputs[
                    f"{node_name}.{node_data.data_key}"
                ] = self.manager.nodes[node_data.node_name].inputs.get_type(
                    node_data.data_key
                )
            elif node_data.data_kind == NodeDataLocation.OUTPUT_DATA:
                subtree_outputs[
                    f"{node_name}.{node_data.data_key}"
                ] = self.manager.nodes[node_data.node_name].outputs.get_type(
                    node_data.data_key
                )

        # merge subtree input and option dicts, so we can receive
        # option updates between ticks
        self.node_config.extend(
            NodeConfig(
                options={},
                inputs=subtree_inputs,
                outputs=subtree_outputs,
                max_children=0,
            )
        )

        # Register the input and output values from the subtree
        self._register_node_data(source_map=subtree_inputs, target_map=self.inputs)
        self._register_node_data(source_map=subtree_outputs, target_map=self.outputs)

        # Handle forwarding inputs and outputs using the subscribe mechanics:
        for node_data in self.manager.to_msg().public_node_data:
            # get the node name without prefix to match our renamed
            # inputs and outputs
            node_name = node_data.node_name
            if node_name.startswith(self.prefix):
                node_name = node_name[len(self.prefix) :]

            if node_data.data_kind == NodeDataLocation.INPUT_DATA:
                if options.get("use_io_nodes") and node_data.node_name not in io_inputs:
                    self.logwarn(
                        f"removed an unconnected input ({node_name}) from the subtree"
                    )
                else:
                    self.inputs.subscribe(
                        key=f"{node_name}.{node_data.data_key}",
                        callback=self.manager.nodes[
                            node_data.node_name
                        ].inputs.get_callback(node_data.data_key),
                    )
            elif node_data.data_kind == NodeDataLocation.OUTPUT_DATA:
                if (
                    options.get("use_io_nodes")
                    and node_data.node_name not in io_outputs
                ):
                    pass
                else:
                    self.manager.nodes[node_data.node_name].outputs.subscribe(
                        key=node_data.data_key,
                        callback=self.outputs.get_callback(
                            f"{node_name}.{node_data.data_key}"
                        ),
                    )

    def _do_setup(self):
        self.root = self.manager.find_root()
        if self.root is None:
            raise BehaviorTreeException(
                "Cannot find root in subtree, does the subtree "
                f"{self.options['subtree_path']} exist?"
            )
        self.root.setup()
        if self.debug_manager and self.debug_manager.get_publish_subtrees():
            self.manager.name = self.name
            self.manager.tree_msg.name = self.name
            self.debug_manager.add_subtree_info(self.name, self.manager.to_msg())

    def _do_tick(self):
        new_state = self.root.tick()
        if self.debug_manager and self.debug_manager.get_publish_subtrees():
            self.manager.name = self.name
            self.manager.tree_msg.name = self.name
            self.debug_manager.add_subtree_info(self.name, self.manager.to_msg())
        return new_state

    def _do_untick(self):
        new_state = self.root.untick()
        if self.debug_manager and self.debug_manager.get_publish_subtrees():
            self.manager.name = self.name
            self.manager.tree_msg.name = self.name
            self.debug_manager.add_subtree_info(self.name, self.manager.to_msg())
        return new_state

    def _do_reset(self):
        if not self.root:
            return NodeMsg.IDLE
        new_state = self.root.reset()
        if self.debug_manager and self.debug_manager.get_publish_subtrees():
            self.manager.name = self.name
            self.manager.tree_msg.name = self.name
            self.debug_manager.add_subtree_info(self.name, self.manager.to_msg())
        return new_state

    def _do_shutdown(self):
        if not self.root:
            return NodeMsg.SHUTDOWN
        self.root.shutdown()
        if self.debug_manager and self.debug_manager.get_publish_subtrees():
            self.manager.name = self.name
            self.manager.tree_msg.name = self.name
            self.debug_manager.add_subtree_info(self.name, self.manager.to_msg())

    def _do_calculate_utility(self):
        self.root = self.manager.find_root()
        if self.root is not None:
            return self.root.calculate_utility()
        else:
            return UtilityBounds(
                has_lower_bound_success=False,
                has_upper_bound_success=False,
                has_lower_bound_failure=False,
                has_upper_bound_failure=False,
            )
