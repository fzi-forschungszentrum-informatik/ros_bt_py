#! /usr/bin/env python2.7

import rospy
import tf2_ros
from tf.transformations import vector_norm

from std_msgs.msg import Header
from geometry_msgs.msg import (Pose, Point, Vector3, Quaternion, PoseStamped, Transform,
                               TransformStamped)
from visualization_msgs.msg import Marker, MarkerArray

from bt_sim_helpers.msg import SimObjects, SimObject
from bt_sim_helpers.srv import InteractObject, InteractObjectRequest, InteractObjectResponse


class ObjectHelper(object):
    def __init__(self):
        self.max_interact_distance = rospy.get_param('~max_interact_distance')
        self.frame_prefix = rospy.get_param('~object_frame_prefix')
        self.world_frame = rospy.get_param('~world_frame')
        object_positions = rospy.get_param('~positions', [])
        object_list = [SimObject(object_id=index,
                                 state=SimObject.ON_GROUND,
                                 pose=PoseStamped(
                                     header=Header(frame_id=self.world_frame),
                                     pose=Pose(position=Point(pos[0], pos[1], 0),
                                               orientation=Quaternion(0, 0, 0, 1))))
                       for index, pos in enumerate(object_positions) if len(pos) == 2]
        self.objects = SimObjects(object_list)

        self.objects_pub = rospy.Publisher('objects', SimObjects, latch=True, queue_size=1)
        self.objects_pub.publish(self.objects)

        self.marker_pub = rospy.Publisher('object_markers', MarkerArray, latch=True, queue_size=1)
        self.marker_pub.publish(MarkerArray(self.create_object_markers()))

        self.interact_service = rospy.Service('interact', InteractObject, self.interact_handler)

        self.buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buf)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Auto-starts
        self.tf_timer = rospy.Timer(
            period=rospy.Duration(0.05),
            callback=self.publish_object_tfs)

    def object_frame(self, obj_id):
        return '{}{}'.format(self.frame_prefix, obj_id)

    def create_object_markers(self):
        markers = []
        interact_diameter = self.max_interact_distance * 2

        for obj in self.objects.sim_objects:
            box = Marker()
            box.frame_locked = True
            box.header.frame_id = self.object_frame(obj.object_id)
            box.header.stamp = rospy.Time.now()
            box.ns = 'sim_objects'

            box.type = Marker.CYLINDER
            box.action = Marker.ADD
            # Make quaternion valid
            box.pose.orientation.w = 1.0

            box.scale.x = interact_diameter * 0.25
            box.scale.y = interact_diameter * 0.25
            box.scale.z = interact_diameter * 0.5

            box.color.r = 1.0
            box.color.g = 1.0
            box.color.b = 0.0
            box.color.a = 0.8

            box.pose.position.z = box.scale.z * 0.5

            if obj.state == SimObject.PICKED_UP:
                box.pose.position.z += interact_diameter * 0.5

            if obj.state == SimObject.DELIVERED:
                box.color.r = 0.0

            box.id = obj.object_id * 2

            area = Marker(
                header=box.header,
                ns=box.ns,
                id=box.id + 1,
                type=Marker.CYLINDER,
                action=Marker.ADD)

            area.scale.x = interact_diameter
            area.scale.y = interact_diameter
            area.scale.z = 0.01
            area.pose.orientation.w = 1.0
            area.pose.position.z = area.scale.z * -0.5

            area.color.r = 0.8
            area.color.g = 0.85
            area.color.b = 0.81
            area.color.a = 1.0

            markers.append(box)
            markers.append(area)

        return markers

    def publish_object_tfs(self, _=None):
        for obj in self.objects.sim_objects:
            self.publish_object_tf(obj)
        self.marker_pub.publish(MarkerArray(self.create_object_markers()))

    def publish_object_tf(self, obj):
        self.tf_broadcaster.sendTransform(
            TransformStamped(header=Header(stamp=rospy.Time.now(),
                                           frame_id=obj.pose.header.frame_id),
                             child_frame_id=self.object_frame(obj.object_id),
                             transform=Transform(
                                 translation=Vector3(obj.pose.pose.position.x,
                                                     obj.pose.pose.position.y,
                                                     obj.pose.pose.position.z),
                                 rotation=obj.pose.pose.orientation)))

    def interact_handler(self, req):
        if req.object_id >= len(self.objects.sim_objects):
            return InteractObjectResponse(
                success=False,
                error_message='Object ID {} is out of bounds'.format(req.object_id))
        try:
            trans = self.buf.lookup_transform(
                req.interacter_frame,
                self.object_frame(req.object_id),
                rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            return InteractObjectResponse(success=False,
                                          error_message=str(ex))

        dist = vector_norm([trans.transform.translation.x,
                            trans.transform.translation.y,
                            trans.transform.translation.z])

        if dist > self.max_interact_distance:
            return InteractObjectResponse(
                success=False,
                error_message=(
                    'interacter "{}" is too far away ({:.2f}/{:.2f}) from object {}'
                    .format(
                        req.interacter_frame,
                        dist,
                        self.max_interact_distance,
                        req.object_id)))

        if req.action == InteractObjectRequest.PICK_UP:
            if self.objects.sim_objects[req.object_id].state != SimObject.ON_GROUND:
                return InteractObjectResponse(
                    success=False,
                    error_message=(
                        'Cannot pick up object {}, since it is not on the ground'.format(
                            req.object_id)))
            # If we're picking it up, change its header frame ID to
            # that of the interacter, and adjust position to (0, 0, 0)
            self.objects.sim_objects[req.object_id].state = SimObject.PICKED_UP
            self.objects.sim_objects[req.object_id].pose.header.frame_id = req.interacter_frame
            self.objects.sim_objects[req.object_id].pose.pose.position = Point(0, 0, 0)
        elif req.action == InteractObjectRequest.DROP:
            if self.objects.sim_objects[req.object_id].state != SimObject.PICKED_UP:
                return InteractObjectResponse(
                    success=False,
                    error_message=(
                        'Cannot drop object {}, since it is not being carried'.format(
                            req.object_id)))

            current_frame = self.objects.sim_objects[req.object_id].pose.header.frame_id
            if current_frame != req.interacter_frame:
                return InteractObjectResponse(
                    success=False,
                    error_message=(
                        ('Interacter "{}" cannot drop object {}, since '
                         'it is being carried by "{}"').format(
                            req.interacter_frame,
                            req.object_id,
                            current_frame
                        )))
            # If we're dropping it, change its header frame ID back to
            # world, and look up its current pose and save that
            try:
                trans = self.buf.lookup_transform(
                    self.world_frame,
                    self.object_frame(req.object_id),
                    rospy.Time())
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as ex:
                return InteractObjectResponse(success=False,
                                              error_message=str(ex))
            self.objects.sim_objects[req.object_id].state = SimObject.ON_GROUND
            self.objects.sim_objects[req.object_id].pose.header.frame_id = self.world_frame
            self.objects.sim_objects[req.object_id].pose.pose.position = Point(
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z)

        # All done, publish new transform for object and object list
        self.publish_object_tf(self.objects.sim_objects[req.object_id])
        self.objects_pub.publish(self.objects)
        return InteractObjectResponse(success=True)


if __name__ == '__main__':
    rospy.init_node('sim_pickups_node')

    helper = ObjectHelper()
    rospy.spin()
    helper.tf_timer.shutdown()
