from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.channels import ChannelManager as SystemChannelManager


class ChannelManager(SystemChannelManager):
    channels = SystemChannelManager.channels
