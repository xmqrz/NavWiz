from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
import logging
import six

from .camera import CameraChannel
from .channel import Channel
from .fms import FmsChannel
from .io import IoChannel
from .log import LogChannel
from .module import ModuleChannel
from .notification import NotificationChannel
from .popup import PopupChannel
from .remote_assist import RemoteAssistChannel
from .remote_io import RemoteIoChannel
from .status import StatusChannel

if not django_settings.TRACKLESS:
    from .map import MapChannel
else:
    from .map_x import MapXChannel
    from .amcl import AmclChannel


logger = logging.getLogger(__name__)


class ChannelManager(object):
    channels = [CameraChannel, FmsChannel, IoChannel, LogChannel, ModuleChannel, NotificationChannel,
                PopupChannel, RemoteAssistChannel, RemoteIoChannel, StatusChannel]
    if not django_settings.TRACKLESS:
        channels += [MapChannel]
    else:
        channels += [MapXChannel, AmclChannel]

    def __init__(self):
        self.channels = {}
        self.subscriptions = {}

        for cls in type(self).channels:
            self.add_channel(cls)

    def add_channel(self, cls):
        if not issubclass(cls, Channel):
            logger.error('Not a valid channel [%s].', cls)
            return

        if cls.id in self.channels:
            logger.error(
                'Another channel with the same id as this channel [%s] has been registered.', cls)
            return

        self.channels[cls.id] = cls(self)

    def broadcast(self, channel, msg):
        try:
            subs = self.subscriptions[channel]
            if subs:
                conn = next(iter(subs))
                conn.broadcast(subs, msg)
        except KeyError:
            pass

    def publish(self, channel, data, connection):
        try:
            self.channels[channel].on_message(data, connection)
        except KeyError:
            pass

    def subscribe(self, channel, connection):
        logger.info('Subscribe to "%s" (id=%d)', channel, connection.id)
        try:
            ch = self.channels[channel]

            if channel not in self.subscriptions:
                self.subscriptions[channel] = set()
            self.subscriptions[channel].add(connection)

            ch.on_subscribe(connection)
        except KeyError:
            pass

    def unsubscribe(self, channel, connection):
        logger.info('Unsubscribe from "%s" (id=%d)', channel, connection.id)
        try:
            if connection in self.subscriptions[channel]:
                self.channels[channel].on_unsubscribe(connection)
                self.subscriptions[channel].remove(connection)
        except KeyError:
            pass

    def unsubscribe_all(self, connection):
        logger.info('Unsubscribe from all (id=%d)', connection.id)
        for channel, subs in six.iteritems(self.subscriptions):
            try:
                if connection in subs:
                    self.channels[channel].on_unsubscribe(connection)
                    subs.remove(connection)
            except (AttributeError, KeyError):
                pass
