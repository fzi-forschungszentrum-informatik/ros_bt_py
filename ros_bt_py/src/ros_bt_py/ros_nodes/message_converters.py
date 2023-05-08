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


import inspect
from typing import Optional, Dict

import genpy

from ros_bt_py_msgs.msg import Node as NodeMsg
from std_msgs.msg import Duration, Time

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={"input_type": type},
        inputs={"in": OptionRef("input_type")},
        outputs={},
        max_children=0,
    )
)
class MessageToFields(Leaf):
    """Takes a ROS message as input and splits it into multiple outputs based on the message fields

    The outputs will be named after the fields in the input message

    If the input is not as ROS message, it will be be passed through

    """

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: str = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(MessageToFields, self).__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        node_outputs = {}

        self.passthrough = True

        if (
            inspect.isclass(self.options["input_type"])
            and genpy.message.Message in self.options["input_type"].__mro__
        ):
            msg = self.options["input_type"]()
            for field in msg.__slots__:
                if isinstance(getattr(msg, field), genpy.rostime.Time):
                    # change the type of a genpy.rostime.Time field to std_msgs/Time
                    node_outputs[field] = Time
                elif isinstance(getattr(msg, field), genpy.rostime.Duration):
                    # change the type of a genpy.rostime.Duration field to std_msgs/Duration
                    node_outputs[field] = Duration
                else:
                    node_outputs[field] = type(getattr(msg, field))
            self.passthrough = False
        else:
            node_outputs["out"] = self.options["input_type"]

        self.node_config.extend(
            NodeConfig(options={}, inputs={}, outputs=node_outputs, max_children=0)
        )

        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.passthrough:
            self.outputs["out"] = self.inputs["in"]
        else:
            for field in self.outputs:
                value = getattr(self.inputs["in"], field)
                if isinstance(value, genpy.rostime.Time):
                    time_value = Time()
                    time_value.data.secs = value.secs
                    time_value.data.nsecs = value.nsecs
                    self.outputs[field] = time_value
                elif isinstance(value, genpy.rostime.Duration):
                    duration_value = Duration()
                    duration_value.data.secs = value.secs
                    duration_value.data.nsecs = value.nsecs
                    self.outputs[field] = duration_value
                else:
                    if type(value) == tuple and self.outputs.get_type(field) == list:
                        self.outputs[field] = list(value)
                    else:
                        self.outputs[field] = value
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={"output_type": type},
        inputs={},
        outputs={"out": OptionRef("output_type")},
        max_children=0,
    )
)
class FieldsToMessage(Leaf):
    """Takes multiple fields as input and outputs a ROS message

    The inputs will be named after the fields in the output message

    If the output is not as ROS message, the input will be be passed through

    """

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: str = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(FieldsToMessage, self).__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        node_inputs = {}

        self.passthrough = True

        if (
            inspect.isclass(self.options["output_type"])
            and genpy.message.Message in self.options["output_type"].__mro__
        ):
            msg = self.options["output_type"]()
            for field in msg.__slots__:
                if isinstance(getattr(msg, field), genpy.rostime.Time):
                    # change the type of a genpy.rostime.Time field to std_msgs/Time
                    node_inputs[field] = Time
                elif isinstance(getattr(msg, field), genpy.rostime.Duration):
                    # change the type of a genpy.rostime.Duration field to std_msgs/Duration
                    node_inputs[field] = Duration
                else:
                    node_inputs[field] = type(getattr(msg, field))
            self.passthrough = False
        else:
            node_inputs["in"] = self.options["output_type"]

        self.node_config.extend(
            NodeConfig(options={}, inputs=node_inputs, outputs={}, max_children=0)
        )

        self._register_node_data(source_map=node_inputs, target_map=self.inputs)

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.passthrough:
            self.outputs["out"] = self.inputs["in"]
        else:
            msg = self.options["output_type"]()

            for field in msg.__slots__:
                if isinstance(getattr(msg, field), genpy.rostime.Time):
                    time_value = genpy.rostime.Time(
                        secs=self.inputs[field].data.secs,
                        nsecs=self.inputs[field].data.nsecs,
                    )
                    setattr(msg, field, time_value)
                elif isinstance(getattr(msg, field), genpy.rostime.Duration):
                    duration_value = genpy.rostime.Duration(
                        secs=self.inputs[field].data.secs,
                        nsecs=self.inputs[field].data.nsecs,
                    )
                    setattr(msg, field, duration_value)
                else:
                    setattr(msg, field, self.inputs[field])

            self.outputs["out"] = msg
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE
