from __future__ import absolute_import
from __future__ import unicode_literals

import rospy
import std_msgs.msg
import ujson as json

from .channel import Channel


class LiveAppChannel(Channel):
    id = 'live-app'

    def __init__(self, *args, **kwargs):
        super(LiveAppChannel, self).__init__(*args, **kwargs)

        self.connected_client = None
        self.__in_pipe = rospy.Subscriber(self.agv05_executor + '/live_app_out', std_msgs.msg.String, self.handle_in_pipe)  # no queue_size, but limited to default buff_size=65536.
        self.__out_pipe = rospy.Publisher(self.agv05_executor + '/live_app_in', std_msgs.msg.String, queue_size=10)

    def on_subscribe(self, connection):
        if not self.connected_client:
            self.connected_client = connection

    def on_unsubscribe(self, connection):
        if connection == self.connected_client:
            self.connected_client = None
            connections = self.manager.get_subscriptions(self.id)
            for c in connections:
                if c == connection:
                    continue
                self.connected_client = c
                break

    def on_message(self, data, connection):
        if connection != self.connected_client:
            return
        self.__out_pipe.publish(std_msgs.msg.String(data=json.dumps(data)))

    def handle_in_pipe(self, msg):
        if not self.connected_client:
            return

        try:
            self.send(json.loads(msg.data), self.connected_client)
        except ValueError:
            pass
