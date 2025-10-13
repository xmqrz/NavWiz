#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from pymodbus.datastore import ModbusSparseDataBlock
from agv05_msgs.msg import OutputPort
import rospy


class CoilOutput(ModbusSparseDataBlock):
    '''custom pymodbus datastore for coil output'''

    address_count = 16

    @classmethod
    def create(klass):
        return klass()

    def __init__(self):
        self.outputs = 0
        rospy.Subscriber('agv05/io/port1/output', OutputPort, self.callbackOutput)
        self.output_pub = rospy.Publisher('agv05/io/port1/output', OutputPort, queue_size=10)

    def default(self, count, value=False):
        pass

    def reset(self):
        pass

    def validate(self, address, count=1):
        return address >= 1 and count >= 1 and address + count <= self.address_count + 1

    def setValues(self, address, values):
        output_mask = 2**len(values) - 1 << address - 1
        output = 0
        for addr, value in enumerate(values, start=address):
            if value:
                output += 1 << addr - 1
        self.output_pub.publish(output, output_mask)

    def getValues(self, address, count=1):
        return [bool(self.outputs & (1 << addr - 1)) for addr in range(address, address + count)]

    def callbackOutput(self, msg):
        # bit swap
        self.outputs = self.outputs ^ ((self.outputs ^ msg.output) & msg.output_mask)
