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
            'find_best_executor',
            FindBestExecutorAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self._as.start()

    def execute_cb(self, goal):
        eval_services = rosservice_find('ros_bt_py/EvaluateUtility')

        bounds = []
        res = self.tree_manager.load_tree(LoadTreeRequest(tree=goal.tree))
        if res.success:
            bounds.append(('__local', self.tree_manager.find_root().calculate_utility()))
        for srv_name in eval_services:
            if self._as.is_preempt_requested():
                # TODO(nberg): maybe send result with best so far?
                self._as.set_preempted()
                return

            service_client = rospy.ServiceProxy(srv_name,
                                                EvaluateUtility)
            service_client.wait_for_service(1.0)
            bounds.append((srv_name,
                           service_client(
                               EvaluateUtilityRequest(tree=goal.tree)).utility))

        if not bounds:
            self._as.set_aborted(result=None, text="")

        # First sort by the number of set bounds, then by the average
        # utility bound (over the ones that are set).
        best_name, best_bounds = sorted(
            bounds,
            key=lambda x: (num_set_bounds(x), avg_bound(x)))[0]

        rospy.logdebug('Executor "%s" has the best bounds:\n%s',
                       best_name, str(best_bounds))
        result = FindBestExecutorResult()
        if best_name == '__local':
            result.local_is_best = True
        else:
            result.local_is_best = False
            result.best_executor_namespace = best_name

        self.set_succeeded(result)


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

    return bound_sum / count
