from __future__ import absolute_import
from __future__ import unicode_literals

from wraptor.decorators import throttle
import distutils.util
import logging
import os
import rospy
import geometry_msgs.msg

from .channel import Channel


class AmclChannel(Channel):
    id = 'amcl'

    def __init__(self, *args, **kwargs):
        super(AmclChannel, self).__init__(*args, **kwargs)
        self.particle_cloud = []
        try:
            self.enable = distutils.util.strtobool(os.environ['SHOW_AMCL_PARTICLES'])
        except Exception:
            self.enable = False

    def on_subscribe(self, connection):
        if not self.enable:
            return

        if self.get_num_subscribers() <= 1:
            self._start()

    def on_unsubscribe(self, connection):
        if not self.enable:
            return

        if self.get_num_subscribers() <= 1:
            self._stop()

    def on_message(self, data, connection):
        if not self.enable:
            return

        try:
            if data['id'] == 'retrieve_all':
                self._retrieve_all(connection)
        except KeyError as ex:
            pass

    def _retrieve_all(self, connection):
        self._send_particle_cloud(connection)

    def _start(self):
        self.__particle_cloud_sub = rospy.Subscriber('/particlecloud', geometry_msgs.msg.PoseArray, lambda msg: self.handle_particle_cloud(msg), queue_size=1)

    def _stop(self):
        self.__particle_cloud_sub.unregister()

    @throttle(0.5, instance_method=True)
    def handle_particle_cloud(self, msg):
        self.particle_cloud = [{
            'position': {
                'x': p.position.x,
                'y': p.position.y,
                'z': p.position.z,
            },
            'orientation': {
                'x': p.orientation.x,
                'y': p.orientation.y,
                'z': p.orientation.z,
                'w': p.orientation.w,
            },
        } for p in msg.poses]
        self._send_particle_cloud()

    def _send_particle_cloud(self, connection=None):
        data = {
            'id': 'particle_cloud',
            'particle_cloud': self.particle_cloud,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)
