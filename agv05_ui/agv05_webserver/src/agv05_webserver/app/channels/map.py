from __future__ import absolute_import
from __future__ import unicode_literals

import geometry_msgs.msg
import rospy
import std_msgs.msg
import ujson as json

from .channel import Channel


class MapChannel(Channel):
    id = 'map'

    def on_subscribe(self, connection):
        if self.get_num_subscribers() <= 1:
            self._start()

    def on_unsubscribe(self, connection):
        if self.get_num_subscribers() <= 1:
            self._stop()

    def _start(self):
        self.map_layout = None
        self.robot_motion = None
        self.robot_pose = None
        self.robot_svg = None

        self.__map_layout_sub = rospy.Subscriber('/map_layout', std_msgs.msg.String, self.handle_map_layout, queue_size=1)
        self.__robot_motion_sub = rospy.Subscriber('/robot_motion', std_msgs.msg.String, self.handle_robot_motion, queue_size=1)
        self.__robot_pose_sub = rospy.Subscriber('/robot_pose', geometry_msgs.msg.Pose2D, self.handle_robot_pose, queue_size=1)
        self.__robot_svg_sub = rospy.Subscriber('/robot_svg', std_msgs.msg.String, self.handle_robot_svg, queue_size=1)

    def _stop(self):
        self.__map_layout_sub.unregister()
        self.__robot_motion_sub.unregister()
        self.__robot_pose_sub.unregister()
        self.__robot_svg_sub.unregister()

    def on_message(self, data, connection):
        try:
            if data['id'] == 'retrieve_all':
                self._retrieve_all(connection)
        except KeyError as ex:
            pass

    def handle_map_layout(self, msg):
        if self.map_layout == msg.data:
            return
        self.map_layout = msg.data
        self._send_map_layout()

    def handle_robot_motion(self, msg):
        try:
            self.robot_motion = json.loads(msg.data)
        except Exception:
            pass
        else:
            self._send_robot_motion()

    def handle_robot_pose(self, msg):
        # convert msg to dict so that it is json-serializable
        self.robot_pose = {k: getattr(msg, k) for k in msg.__slots__}
        self._send_robot_pose()

    def handle_robot_svg(self, msg):
        try:
            self.robot_svg = json.loads(msg.data)
        except Exception:
            pass
        else:
            self._send_robot_svg()

    def _send_map_layout(self, connection=None):
        if not self.map_layout:
            return

        data = {
            'id': 'map_layout',
            'map_layout': self.map_layout,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_robot_motion(self, connection=None):
        if not self.robot_motion:
            return

        data = {
            'id': 'robot_motion',
            'robot_motion': self.robot_motion,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_robot_pose(self, connection=None):
        if not self.robot_pose:
            return

        data = {
            'id': 'robot_pose',
            'robot_pose': self.robot_pose,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_robot_svg(self, connection=None):
        if not self.robot_svg:
            return

        data = {
            'id': 'robot_svg',
            'robot_svg': self.robot_svg,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _retrieve_all(self, connection):
        self._send_map_layout(connection)
        self._send_robot_motion(connection)
        self._send_robot_pose(connection)
        self._send_robot_svg(connection)
