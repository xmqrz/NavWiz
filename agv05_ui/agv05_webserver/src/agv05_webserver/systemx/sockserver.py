from __future__ import absolute_import
from __future__ import unicode_literals

import agv05_webserver.system.sockserver

from .channels import ChannelManager


class Connection(agv05_webserver.system.sockserver.Connection):
    channel_manager_cls = ChannelManager
