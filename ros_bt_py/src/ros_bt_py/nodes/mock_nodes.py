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


from typing import Optional, Dict

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.exceptions import NodeConfigError
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={"output_type": type, "state_values": list, "output_values": list},
        inputs={},
        outputs={
            "out": OptionRef("output_type"),
            "current_index": int,
            "tick_count": int,
            "untick_count": int,
            "reset_count": int,
            "shutdown_count": int,
        },
        max_children=0,
    )
)
class MockLeaf(Leaf):
    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: str = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(Leaf, self).__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        self.setup_called = False
        self.tick_count = 0
        self.untick_count = 0
        self.reset_count = 0
        self.shutdown_count = 0

    def _do_setup(self):
        self.setup_called = True
        self.outputs["current_index"] = 0
        self.outputs["tick_count"] = self.tick_count
        self.outputs["untick_count"] = self.untick_count
        self.outputs["reset_count"] = self.reset_count
        self.outputs["shutdown_count"] = self.shutdown_count

        if len(self.options["state_values"]) != len(self.options["output_values"]):
            raise NodeConfigError(
                "state_values and output_values must have the same length!"
            )
        for value in self.options["output_values"]:
            try:
                self.outputs["out"] = value
            except TypeError:
                raise NodeConfigError(
                    'Provided output value "%s (%s)" is not compatible with '
                    "output type %s"
                    % (
                        str(value),
                        type(value).__name__,
                        self.options["output_type"].__name__,
                    )
                )
            self.outputs["out"] = None

    def _do_tick(self):
        self.outputs["out"] = self.options["output_values"][
            self.outputs["current_index"]
        ]
        new_state = self.options["state_values"][self.outputs["current_index"]]
        # Increment index (and roll over if necessary
        self.outputs["current_index"] = (self.outputs["current_index"] + 1) % len(
            self.options["state_values"]
        )

        self.tick_count += 1
        self.outputs["tick_count"] = self.tick_count
        return new_state

    def _do_untick(self):
        # We leave current_index untouched, so paused is the most semantically
        # correct state
        self.untick_count += 1
        self.outputs["untick_count"] = self.untick_count
        return NodeMsg.PAUSED

    def _do_reset(self):
        self.outputs["current_index"] = 0
        self.reset_count += 1
        self.outputs["reset_count"] = self.reset_count
        return NodeMsg.IDLE

    def _do_shutdown(self):
        self.outputs["current_index"] = 0
        self.shutdown_count += 1
        self.outputs["shutdown_count"] = self.shutdown_count

    def _do_calculate_utility(self):
        return UtilityBounds(can_execute=True)


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={
            "can_execute": bool,
            "utility_lower_bound_success": float,
            "utility_upper_bound_success": float,
            "utility_lower_bound_failure": float,
            "utility_upper_bound_failure": float,
        },
        inputs={},
        outputs={"calculate_utility_count": int},
        max_children=0,
    )
)
class MockUtilityLeaf(Leaf):
    def _do_calculate_utility(self):
        if self.outputs["calculate_utility_count"]:
            self.outputs["calculate_utility_count"] += 1
        else:
            self.outputs["calculate_utility_count"] = 1
        return UtilityBounds(
            can_execute=self.options["can_execute"],
            has_lower_bound_success=True,
            lower_bound_success=self.options["utility_lower_bound_success"],
            has_upper_bound_success=True,
            upper_bound_success=self.options["utility_upper_bound_success"],
            has_lower_bound_failure=True,
            lower_bound_failure=self.options["utility_lower_bound_failure"],
            has_upper_bound_failure=True,
            upper_bound_failure=self.options["utility_upper_bound_failure"],
        )

    def _do_shutdown(self):
        self.shutdown_count += 1

    def _do_setup(self):
        self.setup_called = True
        self.tick_count = 0
        self.untick_count = 0
        self.reset_count = 0
        self.shutdown_count = 0

    def _do_tick(self):
        self.tick_count += 1
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        self.untick_count += 1
        return NodeMsg.IDLE

    def _do_reset(self):
        self.reset_count += 1
        return NodeMsg.IDLE
