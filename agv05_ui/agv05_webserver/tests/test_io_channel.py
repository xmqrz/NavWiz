#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

import agv05_msgs.msg
import django
import os
import random
import rospy
import rostest
import std_msgs.msg
import unittest

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "agv05_webserver.settings-test")
django.setup()  # This must come before IoChannel

if True:  # prevent autopep8 messing up the import sequence
    from agv05_webserver.app.channels.io import IoChannel


class TestIoChannel(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Queue size reference: https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#
        cls.io_channel = IoChannel(None)
        cls.io_channel.broadcast = lambda *args: None  # Since ChannelManager is not involved, there is no broadcast() function
        cls.io_channel.send = lambda *args: None  # Since ChannelManager is not involved, there is no send() function
        cls.max_uint = pow(2, cls.io_channel.bits_per_port)
        cls.io_input_pubs = [rospy.Publisher("agv05/io/port%d/input" % (i), std_msgs.msg.UInt16, queue_size=10) for i in range(1, cls.io_channel.num_of_ports + 1)]
        cls.io_output_pubs = [rospy.Publisher("agv05/io/port%d/output" % (i), agv05_msgs.msg.OutputPort, queue_size=10) for i in range(1, cls.io_channel.num_of_ports + 1)]
        cls.robot_running_pub = rospy.Publisher("agv05_executor/robot_running", std_msgs.msg.UInt8, queue_size=1)

        is_ready = False
        while not is_ready:
            is_ready = True
            for pub in cls.io_input_pubs:
                if pub.get_num_connections() == 0:
                    is_ready = False
                    rospy.sleep(0.2)  # Give some time for initialization
                    break

        is_ready = False
        while not is_ready:
            is_ready = True
            for pub in cls.io_output_pubs:
                if pub.get_num_connections() == 0:
                    is_ready = False
                    rospy.sleep(0.2)  # Give some time for initialization
                    break

    def setUp(self):
        self.io_channel.reset_inputs_outputs()  # Reset before starting each test

    def test_update_inputs(self):
        expected_list = [random.randint(0, self.max_uint) for i in range(self.io_channel.num_of_ports)]
        # Test all 8 ports
        for i in range(self.io_channel.num_of_ports):
            self.io_channel.update_inputs(std_msgs.msg.UInt16(data=expected_list[i]), i + 1)
            rospy.sleep(0.1)
        self.assertEqual(self.io_channel.inputs, expected_list)

    def test_handle_inputs(self):
        expected_list = [random.randint(0, self.max_uint) for i in range(self.io_channel.num_of_ports)]
        # Test all 8 ports
        for i in range(self.io_channel.num_of_ports):
            self.io_channel.handle_inputs(std_msgs.msg.UInt16(data=expected_list[i]), i + 1)
            rospy.sleep(0.1)
        self.assertEqual(self.io_channel.inputs, expected_list)

    def test_handle_inputs_through_topic(self):
        expected_list = [random.randint(0, self.max_uint) for i in range(self.io_channel.num_of_ports)]
        # Test all 8 ports
        for i in range(self.io_channel.num_of_ports):
            self.io_input_pubs[i].publish(std_msgs.msg.UInt16(data=expected_list[i]))
            rospy.sleep(0.2)  # Wait self.io_channel.inputs to be updated
        self.assertEqual(self.io_channel.inputs, expected_list)

    def test_update_outputs(self):
        o_test = [random.randint(0, self.max_uint) for i in range(self.io_channel.num_of_ports)]
        o_mask_test = [random.randint(0, self.max_uint) for i in range(self.io_channel.num_of_ports)]
        expected_list = [o_test[i] & o_mask_test[i] for i in range(self.io_channel.num_of_ports)]
        # Test all 8 ports
        for i in range(self.io_channel.num_of_ports):
            self.io_channel.update_outputs(agv05_msgs.msg.OutputPort(output=o_test[i], output_mask=o_mask_test[i]), i + 1)
        self.assertEqual(self.io_channel.outputs, expected_list)

    def test_handle_outputs(self):
        o_test = [random.randint(0, self.max_uint) for i in range(self.io_channel.num_of_ports)]
        o_mask_test = [random.randint(0, self.max_uint) for i in range(self.io_channel.num_of_ports)]
        expected_list = [o_test[i] & o_mask_test[i] for i in range(self.io_channel.num_of_ports)]
        # Test all 8 ports
        for i in range(self.io_channel.num_of_ports):
            self.io_channel.handle_outputs(agv05_msgs.msg.OutputPort(output=o_test[i], output_mask=o_mask_test[i]), i + 1)
        self.assertEqual(self.io_channel.outputs, expected_list)

    def test_handle_outputs_through_topic(self):
        o_test = [random.randint(0, self.max_uint) for i in range(self.io_channel.num_of_ports)]
        o_mask_test = [random.randint(0, self.max_uint) for i in range(self.io_channel.num_of_ports)]
        expected_list = [o_test[i] & o_mask_test[i] for i in range(self.io_channel.num_of_ports)]
        # Test all 8 ports
        for i in range(self.io_channel.num_of_ports):
            self.io_output_pubs[i].publish(agv05_msgs.msg.OutputPort(output=o_test[i], output_mask=o_mask_test[i]))
        rospy.sleep(1)  # Wait self.io_channel.outputs to be updated
        self.assertEqual(self.io_channel.outputs, expected_list)

    def test_reset(self):
        self.io_channel.inputs = [1 for i in range(self.io_channel.num_of_ports)]
        self.io_channel.outputs = [1 for i in range(self.io_channel.num_of_ports)]
        self.io_channel.reset_inputs_outputs()
        rospy.sleep(0.1)  # Wait self.io_channel.inputs and self.io_channel.outputs to be updated
        expected_list = [0 for i in range(self.io_channel.num_of_ports)]
        self.assertEqual(self.io_channel.inputs, expected_list)
        self.assertEqual(self.io_channel.outputs, expected_list)

    # Inputs outputs should be reset when the robot is started
    def test_handle_robot_running(self):
        self.io_channel.inputs = [1 for i in range(self.io_channel.num_of_ports)]
        self.io_channel.outputs = [1 for i in range(self.io_channel.num_of_ports)]
        self.robot_running_pub.publish(std_msgs.msg.UInt8(data=1))  # simulate robot started
        rospy.sleep(0.1)  # Wait self.io_channel.inputs and self.io_channel.outputs to be updated
        expected_list = [0 for i in range(self.io_channel.num_of_ports)]
        self.assertEqual(self.io_channel.inputs, expected_list)
        self.assertEqual(self.io_channel.outputs, expected_list)


if __name__ == "__main__":
    rospy.init_node('test_io_channel')
    rostest.rosrun('agv05_webserver', 'test_io_channel', TestIoChannel)
