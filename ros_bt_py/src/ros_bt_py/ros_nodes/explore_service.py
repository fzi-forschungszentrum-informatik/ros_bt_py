#  -------- BEGIN LICENSE BLOCK --------
#  Copyright 2022 FZI Forschungszentrum Informatik
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#     * Neither the name of the {copyright_holder} nor the names of its
#        contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#  -------- END LICENSE BLOCK --------
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.node import define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.service import ServiceInput


@define_bt_node(
    NodeConfig(inputs={}, options={}, outputs={}, max_children=0, version="1.0.0")
)
class HuskyExploreService(ServiceInput):
    def _do_calculate_utility(self) -> UtilityBounds:
        # This number is the duration in (sec * 10) / detection radius
        utility_value = 53
        return UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            lower_bound_success=utility_value,
            has_upper_bound_success=True,
            upper_bound_success=utility_value,
            has_lower_bound_failure=True,
            lower_bound_failure=0,
            has_upper_bound_failure=True,
            upper_bound_failure=utility_value,
        )


@define_bt_node(
    NodeConfig(inputs={}, options={}, outputs={}, max_children=0, version="1.0.0")
)
class SpotExploreService(ServiceInput):
    def _do_calculate_utility(self) -> UtilityBounds:
        # This number is the duration in (sec * 10) / detection radius
        utility_value = 13
        return UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            lower_bound_success=utility_value,
            has_upper_bound_success=True,
            upper_bound_success=utility_value,
            has_lower_bound_failure=True,
            lower_bound_failure=0,
            has_upper_bound_failure=True,
            upper_bound_failure=utility_value,
        )


@define_bt_node(
    NodeConfig(inputs={}, options={}, outputs={}, max_children=0, version="1.0.0")
)
class BebopExploreService(ServiceInput):
    def _do_calculate_utility(self) -> UtilityBounds:
        # This number is the duration in (sec * 10) / detection radius
        utility_value = 4
        return UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            lower_bound_success=utility_value,
            has_upper_bound_success=True,
            upper_bound_success=utility_value,
            has_lower_bound_failure=True,
            lower_bound_failure=0,
            has_upper_bound_failure=True,
            upper_bound_failure=utility_value,
        )
