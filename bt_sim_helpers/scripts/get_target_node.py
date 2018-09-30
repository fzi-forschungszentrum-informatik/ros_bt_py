import rospy

from geometry_msgs.msg import Pose, Point, Quaternion
from bt_sim_helpers.srv import GetTarget, GetTargetResponse


class GetTargetServer(object):
    def __init__(self):
        norm_quat = Quaternion(0, 0, 0, 1)
        self.targets = [Pose(Point(2, 2, 0), norm_quat),
                        Pose(Point(6, 4, 0), norm_quat)]
        self.get_target_service = rospy.Service('get_target',
                                                GetTarget,
                                                self.get_target)
        self.counter = 0

    def get_target(self, req):
        # self.counter += 1
        # if self.counter > 100:
        #     rospy.sleep(3.0)
        #     self.counter = 0
        res = GetTargetResponse()
        try:
            res.target_pose = self.targets[req.target_id]
            return res
        except IndexError:
            return None


def main():
    rospy.init_node('sim_helper')
    helper = GetTargetServer()
    rospy.spin()


if __name__ == '__main__':
    main()
