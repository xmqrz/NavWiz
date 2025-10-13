from __future__ import absolute_import
from __future__ import unicode_literals

import rospy
import std_msgs.msg
import ujson as json

from .channel import Channel


class PopupChannel(Channel):
    id = 'popup'

    def __init__(self, *args, **kwargs):
        super(PopupChannel, self).__init__(*args, **kwargs)
        self.__is_robot_running = False
        self.popup = {'type': 'none'}

        # Create all ROS subscribers and publishers
        self.__robot_running_sub = rospy.Subscriber(self.agv05_executor + '/robot_running', std_msgs.msg.Bool, self.handle_robot_running, queue_size=1)
        self.__in_pipe = rospy.Subscriber(self.agv05_executor + '/panel_control_out', std_msgs.msg.String, self.handle_in_pipe)  # no queue size, but limited to default buff_size=65536.
        self.__out_pipe = rospy.Publisher(self.agv05_executor + '/panel_control_in', std_msgs.msg.String, queue_size=10)

    def on_subscribe(self, connection):
        self.retrieve_all(connection)

    def on_unsubscribe(self, connection):
        pass

    def on_message(self, data, connection):
        try:
            if data['type'] == 'retrieve_all':
                self.retrieve_all(connection)
            else:
                self.__out_pipe.publish(std_msgs.msg.String(data=json.dumps(data)))
        except KeyError:
            pass

    def handle_robot_running(self, msg):
        self.__is_robot_running = msg.data

    def handle_in_pipe(self, msg):
        try:
            self.popup = json.loads(msg.data)
            self._update_popup_timer()
            self.broadcast(self.popup)
        except ValueError:
            pass

    def retrieve_all(self, connection):
        if not self.__is_robot_running:
            return
        self._update_popup_timer()
        self.send(self.popup, connection)

    def _update_popup_timer(self):
        until = self.popup.get('until')
        if until:
            self.popup['timer'] = max(0, int(until - rospy.get_time()))
        else:
            self.popup['timer'] = -1
