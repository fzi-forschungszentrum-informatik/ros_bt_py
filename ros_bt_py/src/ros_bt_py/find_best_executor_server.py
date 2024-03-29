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


import rospy
from rosservice import rosservice_find

from actionlib.simple_action_server import SimpleActionServer

from ros_bt_py_msgs.msg import FindBestExecutorAction, FindBestExecutorResult
from ros_bt_py_msgs.srv import EvaluateUtility, EvaluateUtilityRequest, LoadTreeRequest
from ros_bt_py.tree_manager import TreeManager


class FindBestExecutorServer(object):
    def __init__(self):
        self.tree_manager = TreeManager()
        self._as = SimpleActionServer(
            "find_best_executor",
            FindBestExecutorAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )

        self._as.start()

    def execute_cb(self, goal):
        eval_services = rosservice_find("ros_bt_py_msgs/EvaluateUtility")
        rospy.loginfo("Found these eval services: %s", eval_services)

        bounds = []
        res = self.tree_manager.load_tree(LoadTreeRequest(tree=goal.tree))
        if res.success:
            bounds.append(
                ("__local", self.tree_manager.find_root().calculate_utility())
            )

        for srv_name in eval_services:
            if self._as.is_preempt_requested():
                # TODO(nberg): maybe send result with best so far?
                self._as.set_preempted()
                return

            service_client = rospy.ServiceProxy(srv_name, EvaluateUtility)
            service_client.wait_for_service(1.0)
            # Cut off the service name to get just the namespace
            #
            # The second paramater to rsplit limits it to one split,
            # which neatly separates the service name from the
            # namespace. The namespace is element 0 of the resulting array
            srv_namespace = srv_name.rsplit("/", 1)[0]
            bounds.append(
                (
                    srv_namespace,
                    service_client(EvaluateUtilityRequest(tree=goal.tree)).utility,
                )
            )

        rospy.loginfo("Utility bounds by service namespace: %s", bounds)
        if not bounds:
            self._as.set_aborted(result=None, text="")

        bounds = [x for x in bounds if x[1].can_execute]
        # First sort bounds so that the lowest average bound
        # (i.e. lowest cost) bounds object is first
        bounds.sort(key=lambda x: avg_bound(x[1]))

        # Now sort so that the ones with the most set bounds are
        # first. Since sort() is stable, this retains the "lowest
        # average bound first" within sets of elements with the same
        # number of set bounds
        bounds.sort(reverse=True, key=lambda x: num_set_bounds(x[1]))

        result = FindBestExecutorResult()
        # If no executor can execute the tree, send a response with an
        # empty namespace *and* local_is_best = False
        if not bounds:
            self._as.set_succeeded(result)
            return

        # Otherwise, extract the best executor
        best_name, best_bounds = bounds[0]

        rospy.loginfo(
            'Executor "%s" has the best bounds:\n%s', best_name, str(best_bounds)
        )

        if best_name == "__local":
            result.local_is_best = True
        else:
            result.local_is_best = False
            result.best_executor_namespace = best_name

        self._as.set_succeeded(result)


def num_set_bounds(bounds):
    """Count how many of the 4 bounds are set"""
    count = 0
    if bounds.has_upper_bound_success:
        count += 1
    if bounds.has_upper_bound_failure:
        count += 1
    if bounds.has_lower_bound_success:
        count += 1
    if bounds.has_lower_bound_failure:
        count += 1
    return count


def avg_bound(bounds):
    """A simple average of those bounds that are set"""
    count = 0
    bound_sum = 0
    if bounds.has_upper_bound_success:
        count += 1
        bound_sum += bounds.upper_bound_success
    if bounds.has_upper_bound_failure:
        count += 1
        bound_sum += bounds.upper_bound_failure
    if bounds.has_lower_bound_success:
        count += 1
        bound_sum += bounds.lower_bound_success
    if bounds.has_lower_bound_failure:
        count += 1
        bound_sum += bounds.lower_bound_failure

    if count > 0:
        return bound_sum / count
    else:
        return 0
