from __future__ import absolute_import
from __future__ import unicode_literals

import rospy
import std_msgs.msg
import ujson as json

from .channel import Channel


class FmsChannel(Channel):
    id = 'fms'

    def __init__(self, *args, **kwargs):
        super(FmsChannel, self).__init__(*args, **kwargs)

        self.__in_pipe = rospy.Subscriber(self.agv05_executor + '/fms_out', std_msgs.msg.String, self.handle_in_pipe)  # no queue_size, but limited to default buff_size=65536.
        self.__out_pipe = rospy.Publisher(self.agv05_executor + '/fms_in', std_msgs.msg.String, queue_size=10)

    def on_subscribe(self, connection):
        pass

    def on_unsubscribe(self, connection):
        pass

    def on_message(self, data, connection):
        self.__out_pipe.publish(std_msgs.msg.String(data=json.dumps(data)))

    def handle_in_pipe(self, msg):
        try:
            self.broadcast(json.loads(msg.data))
        except ValueError:
            pass
