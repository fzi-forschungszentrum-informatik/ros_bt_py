#! /usr/bin/env python2.7

import rospy
import tf2_ros
from tf.transformations import vector_norm

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion, PoseStamped, Transform, TransformStamped
from bt_sim_helpers.msg import Door, Doors
from bt_sim_helpers.srv import OpenDoor, OpenDoorRequest, OpenDoorResponse


class DoorHelper(object):
    def __init__(self):
        self.max_interact_distance = rospy.get_param('~max_interact_distance')
        self.frame_prefix = rospy.get_param('~door_frame_prefix')
        self.world_frame = rospy.get_param('~world_frame')
        door_positions = rospy.get_param('~positions', [])
        door_list = [Door(door_id=index,
                          is_open=False,
                          pose=PoseStamped(
                              header=Header(frame_id=self.world_frame),
                              pose=Pose(position=Point(pos[0], pos[1], 0),
                                        orientation=Quaternion(0, 0, 0, 1))))
                     for index, pos in enumerate(door_positions) if len(pos) == 2]
        self.doors = Doors(door_list)

        self.doors_pub = rospy.Publisher('doors', Doors, latch=True, queue_size=1)
        self.doors_pub.publish(self.doors)

        self.open_service = rospy.Service('open_door', OpenDoor, self.open_door_handler)

        self.buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buf)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Auto-starts
        self.tf_timer = rospy.Timer(
            period=rospy.Duration(0.05),
            callback=self.publish_door_tfs)

    def door_frame(self, door_id):
        return '{}{}'.format(self.frame_prefix, door_id)

    def publish_door_tfs(self, _=None):
        for door in self.doors.doors:
            self.publish_door_tf(door)

    def publish_door_tf(self, door):
        self.tf_broadcaster.sendTransform(
            TransformStamped(header=Header(stamp=rospy.Time.now(),
                                           frame_id=door.pose.header.frame_id),
                             child_frame_id=self.door_frame(door.door_id),
                             transform=Transform(
                                 translation=Vector3(door.pose.pose.position.x,
                                                     door.pose.pose.position.y,
                                                     door.pose.pose.position.z),
                                 rotation=door.pose.pose.orientation)))

    def open_door_handler(self, req):
        if req.door_id >= len(self.doors.doors):
            return OpenDoorResponse(
                success=False,
                error_message='Door ID {} is out of bounds'.format(req.door_id))
        try:
            trans = self.buf.lookup_transform(
                req.opener_frame,
                self.door_frame(req.door_id),
                rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            return OpenDoorResponse(success=False,
                                    error_message=str(ex))

        dist = vector_norm([trans.transform.translation.x,
                            trans.transform.translation.y,
                            trans.transform.translation.z])

        if dist > self.max_interact_distance:
            return OpenDoorResponse(
                success=False,
                error_message=(
                    'opener "{}" is too far away ({:.2f}/{:.2f}) from door {}'
                    .format(
                        req.opener_frame,
                        dist,
                        self.max_interact_distance,
                        req.door_id)))

        if req.action == OpenDoorRequest.OPEN:
            self.doors.doors[req.door_id].is_open = True
        elif req.action == OpenDoorRequest.CLOSE:
            self.doors.doors[req.door_id].is_open = False
        else:
            return OpenDoorResponse(success=False,
                                    error_message='Invalid action {}'.format(req.action))

        # All done, publish new transform for door
        self.doors_pub.publish(self.doors)
        self.publish_door_tf(self.doors.doors[req.door_id])
        return OpenDoorResponse(success=True)


if __name__ == '__main__':
    rospy.init_node('sim_pickups_node')

    helper = DoorHelper()
    rospy.spin()
    helper.tf_timer.shutdown()
