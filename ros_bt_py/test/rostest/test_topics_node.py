#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32


class TopicMirror(object):
    def __init__(self):
        self.publisher = rospy.Publisher('numbers_out', Int32, latch=False, queue_size=1)
        self.subscriber = rospy.Subscriber('numbers_in', Int32, self.numbers_cb)

    def numbers_cb(self, msg):
        rospy.loginfo('Reflecting message %s', str(msg))
        self.publisher.publish(msg)


if __name__ == '__main__':
    rospy.init_node('test_topics')
    mirror = TopicMirror()
    ready_pub = rospy.Publisher('ready', Int32, latch=True, queue_size=1)
    ready_pub.publish(data=0)

    rospy.spin()
