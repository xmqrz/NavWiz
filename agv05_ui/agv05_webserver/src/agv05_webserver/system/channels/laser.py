from __future__ import absolute_import
from __future__ import unicode_literals

from functools import partial
from rospy.numpy_msg import numpy_msg
from wraptor.context import throttle as throttler
import base64
import distutils.util
import logging
import math
import os
import rospy
import sensor_msgs.msg
import std_msgs.msg
import tf
import ujson as json

from .channel import Channel

logger = logging.getLogger(__name__)


def get_topics_list(parameter_name):
    param = rospy.get_param(parameter_name, '')
    if isinstance(param, str):
        return [t.strip() for t in param.split(' ') if t and t.strip()]
    if not isinstance(param, dict):
        return []

    topics = []
    _flatten_dict_topics(param, topics)
    return topics


def _flatten_dict_topics(tree, topics, prefix=''):
    for key, value in tree.items():
        if isinstance(value, str):
            topics.append(prefix + key)
        elif isinstance(value, dict):
            _flatten_dict_topics(value, topics, prefix + key + '/')


class LaserChannel(Channel):
    id = 'laser'

    def __init__(self, *args, **kwargs):
        super(LaserChannel, self).__init__(*args, **kwargs)
        self.tf_frames = ['laser']
        self.__laser_subs = {}

        try:
            self.show_lidar_intensities = distutils.util.strtobool(os.environ['SHOW_LIDAR_INTENSITIES'])
        except Exception:
            self.show_lidar_intensities = False

    def on_subscribe(self, connection):
        # read laser scan topics from param
        topic_1 = get_topics_list('/primary_obstacle_scan_topics')
        topic_2 = get_topics_list('/secondary_obstacle_scan_topics')
        topic_3 = get_topics_list('/tertiary_obstacle_scan_topics')
        topic_4 = get_topics_list('/quaternary_obstacle_scan_topics')
        topic_5 = get_topics_list('/quinary_obstacle_scan_topics')
        topics = set(topic_1 + topic_2 + topic_3 + topic_4 + topic_5)

        if self.get_num_subscribers() <= 1:
            self._start()
        self._update(topics)

    def on_unsubscribe(self, connection):
        if self.get_num_subscribers() <= 1:
            self._stop()
            self._update(set())

    def _start(self):
        self.robot_running = False
        self.robot_svg = None
        self.laser_pose = {
            'laser': {'x': 0, 'y': 0, 'theta': 0, 'flip': False},
        }
        self.__robot_running_sub = rospy.Subscriber(self.agv05_executor + '/robot_running', std_msgs.msg.UInt8, self.handle_robot_running, queue_size=1)
        self.__robot_svg_sub = rospy.Subscriber('/robot_svg', std_msgs.msg.String, self.handle_robot_svg, queue_size=1)
        self.__pose_timer = rospy.Timer(rospy.Duration(0.5), self.handle_laser_pose)

    def _stop(self):
        self.__robot_running_sub.unregister()
        self.__robot_svg_sub.unregister()
        self.__pose_timer.shutdown()

    def _update(self, topics):
        current_topics = set(self.__laser_subs.keys())
        topics_to_remove = current_topics - topics
        topics_to_add = topics - current_topics

        for topic in topics_to_remove:
            self.__laser_subs[topic].unregister()
            del self.__laser_subs[topic]

        for topic in topics_to_add:
            sub = rospy.Subscriber('/' + topic, numpy_msg(sensor_msgs.msg.LaserScan), partial(self.handle_laser_scan, topic), queue_size=1)
            self.__laser_subs[topic] = sub

    def on_message(self, data, connection):
        try:
            if data['id'] == 'retrieve_all':
                self._retrieve_all(connection)
        except KeyError as ex:
            pass

    def handle_robot_running(self, msg):
        self.robot_running = msg.data

    def handle_robot_svg(self, msg):
        try:
            self.robot_svg = json.loads(msg.data)
        except Exception:
            pass
        else:
            self._send_robot_svg()

    def handle_laser_pose(self, timer_event):
        if not self.robot_running:
            return

        updated = False

        for frame_id in self.tf_frames:
            try:
                (trans, rot) = self.tf_listener.lookupTransform('base', frame_id, rospy.Time())
                rpy = tf.transformations.euler_from_quaternion(rot)
                pose = {
                    'x': trans[0],
                    'y': trans[1],
                    'theta': rpy[2],
                    'flip': abs(abs(rpy[0]) - math.pi) < 1e-3,
                }
            except Exception as ex:
                with self._get_throttler(frame_id + '_pose_error', 5.0):
                    logger.error('Error obtaining transformation of laser frame: %s', ex)
            else:
                if self.laser_pose.get(frame_id) != pose:
                    self.laser_pose[frame_id] = pose
                    updated = True

        if updated:
            self._send_laser_pose()

        self.tf_frames = []

    def handle_laser_scan(self, topic, msg):
        frame_id = msg.header.frame_id.lstrip('/')

        # a throttler in disguise
        if frame_id not in self.tf_frames:
            self.tf_frames.append(frame_id)

            self.laser_scan = {
                'scan_id': topic,
                'frame_id': frame_id,
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'ranges': base64.b64encode(msg.ranges.tobytes()).decode(),
            }

            if self.show_lidar_intensities and msg.intensities:
                self.laser_scan['intensities'] = base64.b64encode(msg.intensities.tobytes()).decode()

            self._send_laser_scan()

    def _get_throttler(self, name, seconds=1):
        if not hasattr(self, '_throttlers'):
            self._throttlers = {}
        if name not in self._throttlers:
            self._throttlers[name] = throttler(seconds)
        return self._throttlers[name]

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

    def _send_laser_pose(self, connection=None):
        data = {
            'id': 'laser_pose',
            'laser_pose': self.laser_pose,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_laser_scan(self):
        data = {
            'id': 'laser_scan',
            'laser_scan': self.laser_scan,
        }
        self.broadcast(data)

    def _retrieve_all(self, connection):
        self._send_robot_svg(connection)
        self._send_laser_pose(connection)
