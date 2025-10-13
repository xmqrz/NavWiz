from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
import rospy
import tf
import ujson as json


class Channel(object):
    id = '__channel__'
    agv05_audio_player = django_settings.AGV05_AUDIO_PLAYER
    agv05_executor = django_settings.AGV05_EXECUTOR
    _tf_listener = None

    def __init__(self, manager):
        self.manager = manager

    @property
    def tf_listener(self):
        if not Channel._tf_listener:
            Channel._tf_listener = tf.TransformListener(True, rospy.Duration(1.0))
        return Channel._tf_listener

    def on_subscribe(self, connection):
        pass

    def on_unsubscribe(self, connection):
        pass

    def on_message(self, data, connection):
        raise NotImplementedError()

    def send(self, data, connection):
        connection.send(json.dumps({
            'channel': self.id,
            'data': data
        }))

    def broadcast(self, data):
        self.manager.broadcast(self.id, json.dumps({
            'channel': self.id,
            'data': data
        }))

    def get_num_subscribers(self):
        return len(self.manager.subscriptions.get(self.id, set()))
