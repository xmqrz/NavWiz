from __future__ import absolute_import
from __future__ import unicode_literals

from std_msgs.msg import Int8, UInt8, UInt16MultiArray
import rospy

from .channel import Channel


class RemoteIoChannel(Channel):
    id = 'remote-io'

    def __init__(self, *args, **kwargs):
        super(RemoteIoChannel, self).__init__(*args, **kwargs)
        self.num_of_remoteio = 10
        self.inputs = [0 for i in range(self.num_of_remoteio)]
        self.outputs = [0 for i in range(self.num_of_remoteio)]

        # Create publisher to publish number of subscribers to agv05_remote_io node
        self.subs_count_pub = rospy.Publisher(
            'agv05/remote_io/subscriber_count', Int8, latch=True, queue_size=1)

        # Create all required ROS subscribers, and keep them alive
        self.remote_io_inputs_sub = rospy.Subscriber(
            'agv05/remote_io/inputs', UInt16MultiArray, self.handle_inputs, queue_size=1)
        self.remote_io_outputs_sub = rospy.Subscriber(
            'agv05/remote_io/outputs', UInt16MultiArray, self.handle_outputs, queue_size=1)
        self.__robot_running_sub = rospy.Subscriber(
            "agv05_executor/robot_running", UInt8, self.handle_robot_running, queue_size=1)

    def on_subscribe(self, connection):
        data = Int8()
        data.data = self.get_num_subscribers()
        self.subs_count_pub.publish(data)
        self._retrieve_all(connection)

    def on_unsubscribe(self, connection):
        data = Int8()
        data.data = self.get_num_subscribers() - 1
        self.subs_count_pub.publish(data)

    def reset_inputs_outputs(self):
        self.inputs = [0 for i in range(self.num_of_remoteio)]
        self.outputs = [0 for i in range(self.num_of_remoteio)]

    def handle_inputs(self, msg):
        self.inputs = msg.data
        self.broadcast({
            'id': 'remote-io',
            'inputs': self.inputs,
            'outputs': self.outputs,
        })

    def handle_outputs(self, msg):
        self.outputs = msg.data
        self.broadcast({
            'id': 'remote-io',
            'inputs': self.inputs,
            'outputs': self.outputs,
        })

    def handle_robot_running(self, msg):
        if msg.data:
            # Reset inputs and outputs when the robot is started or rebooted
            self.reset_inputs_outputs()

    def _send_inputs_outputs(self, connection=None):
        if not self.outputs or not self.inputs:
            return

        data = {
            'id': 'remote-io',
            'inputs': self.inputs,
            'outputs': self.outputs,
        }

        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _retrieve_all(self, connection):
        self._send_inputs_outputs(connection)
