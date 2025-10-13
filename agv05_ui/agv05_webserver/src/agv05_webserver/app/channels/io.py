from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Variable, db_auto_reconnect
import agv05_msgs.msg
import rospy
import std_msgs.msg
import ujson as json

from .channel import Channel


class IoChannel(Channel):
    id = 'io'

    def __init__(self, *args, **kwargs):
        super(IoChannel, self).__init__(*args, **kwargs)
        self.num_of_ports = 8
        self.bits_per_port = 16
        self.inputs = [0 for i in range(self.num_of_ports)]
        self.outputs = [0 for i in range(self.num_of_ports)]
        self.io_name = {}
        self.reload_variable()

        # Create all required ROS subscribers, and keep them alive
        self.__subs = [rospy.Subscriber("agv05/io/port%d/input" % (i), std_msgs.msg.UInt16, self.handle_inputs, i, queue_size=10) for i in range(1, self.num_of_ports + 1)]
        self.__subs.extend([rospy.Subscriber("agv05/io/port%d/output" % (i), agv05_msgs.msg.OutputPort, self.handle_outputs, i, queue_size=10) for i in range(1, self.num_of_ports + 1)])
        self.__robot_running_sub = rospy.Subscriber("agv05_executor/robot_running", std_msgs.msg.UInt8, self.handle_robot_running, queue_size=1)

    def on_subscribe(self, connection):
        self.reload_variable()
        self._retrieve_all(connection)

    def on_unsubscribe(self, connection):
        pass

    @db_auto_reconnect()
    def reload_variable(self):
        try:
            self.io_name = json.loads(Variable.objects.get(name=Variable.IO_NAME).value)
        except Exception:
            pass

    def update_inputs(self, msg, port):
        self.inputs[port - 1] = msg.data

    def update_outputs(self, msg, port):
        self.outputs[port - 1] = self.outputs[port - 1] ^ ((self.outputs[port - 1] ^ msg.output) & msg.output_mask)

    def reset_inputs_outputs(self):
        self.inputs = [0 for i in range(self.num_of_ports)]
        self.outputs = [0 for i in range(self.num_of_ports)]

    def handle_inputs(self, msg, port):
        self.update_inputs(msg, port)
        self.broadcast({
            'id': 'io',
            'inputs': self.inputs,
            'outputs': self.outputs,
            'io_name': self.io_name
        })

    def handle_outputs(self, msg, port):
        self.update_outputs(msg, port)
        self.broadcast({
            'id': 'io',
            'inputs': self.inputs,
            'outputs': self.outputs,
            'io_name': self.io_name
        })

    def handle_robot_running(self, msg):
        if msg.data:
            # Reset inputs and outputs when the robot is started or rebooted
            self.reset_inputs_outputs()

    def _send_inputs_outputs(self, connection=None):
        if not self.outputs or not self.inputs:
            return

        data = {
            'id': 'io',
            'inputs': self.inputs,
            'outputs': self.outputs,
            'io_name': self.io_name
        }

        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _retrieve_all(self, connection):
        self._send_inputs_outputs(connection)
