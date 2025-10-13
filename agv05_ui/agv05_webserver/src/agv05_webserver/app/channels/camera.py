from __future__ import absolute_import
from __future__ import unicode_literals

from image_geometry.cameramodels import PinholeCameraModel
from wraptor.context import throttle as throttler
import base64
import logging
import rospy
import sensor_msgs.msg
import std_msgs.msg

from .channel import Channel

logger = logging.getLogger(__name__)


class CameraChannel(Channel):
    id = 'camera'

    def __init__(self, *args, **kwargs):
        super(CameraChannel, self).__init__(*args, **kwargs)
        self.marker_type = ''
        self.__marker_type_sub = rospy.Subscriber('/marker_type', std_msgs.msg.String, self.handle_marker_type, queue_size=1)
        self.__marker_type_pub = rospy.Publisher('/marker_type', std_msgs.msg.String, queue_size=1)
        self.__cloud_flatten_sub = rospy.Subscriber('/cloud_flatten', sensor_msgs.msg.PointCloud2, self.handle_cloud_flatten, queue_size=1)
        self.__cloud_colored_sub = rospy.Subscriber('/cloud_colored', sensor_msgs.msg.PointCloud2, self.handle_cloud_colored, queue_size=1)

    def on_subscribe(self, connection):
        if self.get_num_subscribers() <= 1:
            self._start()

    def on_unsubscribe(self, connection):
        if self.get_num_subscribers() <= 1:
            self._stop()

    def _start(self):
        self._clear()
        self.__pose_timer = rospy.Timer(rospy.Duration(0.5), self.handle_pose)

    def _stop(self):
        self._clear()
        self.__pose_timer.shutdown()

    def _clear(self):
        self.tf_frames = []
        self.pose = {}

    def on_message(self, data, connection):
        try:
            if data['id'] == 'info':
                self._retrieve_info(data['topic'], connection)
                self._send_pose(connection)
                self._send_marker_type(connection)
            elif data['id'] == 'marker_type':
                self.__marker_type_pub.publish(std_msgs.msg.String(data=data['marker_type']))
        except KeyError as ex:
            pass

    def handle_pose(self, timer_event):
        updated = False

        for frame_id in self.tf_frames:
            try:
                (trans, rot) = self.tf_listener.lookupTransform('base', frame_id, rospy.Time())
                pose = {
                    'p': trans,
                    'q': rot,
                }
            except Exception as ex:
                with self._get_throttler(frame_id + '_pose_error', 5.0):
                    logger.error('Error obtaining transformation of camera frame %s: %s', frame_id, ex)
            else:
                if self.pose.get(frame_id) != pose:
                    self.pose[frame_id] = pose
                    updated = True

        if updated:
            self._send_pose()

        self.tf_frames = []

    def handle_marker_type(self, msg):
        self.marker_type = msg.data
        self._send_marker_type()

    def handle_cloud_flatten(self, msg):
        frame_id = msg.header.frame_id.lstrip('/')
        if self.get_num_subscribers() >= 1:
            self.cloud_flatten = {
                'frame_id': frame_id,
                'cloud': base64.b64encode(msg.data).decode(),
            }
            self._send_cloud_flatten()

    def handle_cloud_colored(self, msg):
        frame_id = msg.header.frame_id.lstrip('/')
        if self.get_num_subscribers() >= 1:
            self.cloud_colored = {
                'frame_id': frame_id,
                'cloud': base64.b64encode(msg.data).decode(),
            }
            self._send_cloud_colored()

    def _get_throttler(self, name, seconds=1):
        if not hasattr(self, '_throttlers'):
            self._throttlers = {}
        if name not in self._throttlers:
            self._throttlers[name] = throttler(seconds)
        return self._throttlers[name]

    def _send_pose(self, connection=None):
        data = {
            'id': 'pose',
            'pose': self.pose,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_marker_type(self, connection=None):
        data = {
            'id': 'marker_type',
            'marker_type': self.marker_type,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_cloud_flatten(self):
        data = {
            'id': 'cloud_flatten',
            'cloud_flatten': self.cloud_flatten,
        }
        self.broadcast(data)

    def _send_cloud_colored(self):
        data = {
            'id': 'cloud_colored',
            'cloud_colored': self.cloud_colored,
        }
        self.broadcast(data)

    def _retrieve_info(self, topic, connection):
        try:
            msg = rospy.wait_for_message(topic + 'camera_info', sensor_msgs.msg.CameraInfo, timeout=0.5)
        except Exception as ex:
            return
        frame_id = msg.header.frame_id
        if frame_id not in self.tf_frames:
            self.tf_frames.append(frame_id)

        model = PinholeCameraModel()
        model.fromCameraInfo(msg)
        info = {
            'topic': topic,
            'frame_id': frame_id,
            'width': model.width,
            'height': model.height,
            'cx': model.cx(),
            'cy': model.cy(),
            'fx': model.fx(),
            'fy': model.fy(),
        }

        data = {
            'id': 'info',
            'topic': topic,
            'info': info,
        }
        self.send(data, connection)
