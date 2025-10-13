#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from pymodbus.datastore import ModbusSparseDataBlock
from std_msgs.msg import UInt16
import rospy


class DiscreteInput(ModbusSparseDataBlock):
    '''custom pymodbus datastore for discrete input'''

    address_count = 16

    @classmethod
    def create(klass):
        return klass()

    def __init__(self):
        self.inputs = 0
        rospy.Subscriber('agv05/io/port1/input', UInt16, self.callbackInput)

    def default(self, count, value=False):
        pass

    def reset(self):
        pass

    def validate(self, address, count=1):
        return address >= 1 and count >= 1 and address + count <= self.address_count + 1

    def setValues(self, address, values):
        pass

    def getValues(self, address, count=1):
        return [bool(self.inputs & (1 << addr - 1)) for addr in range(address, address + count)]

    def callbackInput(self, msg):
        self.inputs = msg.data
