from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.channels.log import LogChannel as SystemLogChannel
from .channel import Channel


class LogChannel(SystemLogChannel, Channel):
    pass
