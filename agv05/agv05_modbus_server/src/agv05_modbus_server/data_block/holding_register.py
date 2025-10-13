#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from pymodbus.datastore import ModbusSparseDataBlock
from agv05_executive_msgs.srv import GetRegister, SetRegister
import rospy


class HoldingRegister(ModbusSparseDataBlock):
    '''custom pymodbus datastore for holding register'''

    # max 0xFFFF
    address_count = 50

    @classmethod
    def create(klass):
        return klass()

    def __init__(self):

        self.__get_register = rospy.ServiceProxy('/agv05_executor/get_register', GetRegister, persistent=False)
        self.__set_register = rospy.ServiceProxy('/agv05_executor/set_register', SetRegister, persistent=False)

        # build register mapping.
        self.reg_addr_map = {}
        for i in range(10):
            self.reg_addr_map[i + 1] = 'A%d' % i
            self.reg_addr_map[i + 11] = 'B%d' % i
            self.reg_addr_map[i + 21] = 'C%d' % i
            self.reg_addr_map[i + 31] = 'GB%d' % i
            self.reg_addr_map[i + 41] = 'PS%d' % i

        rospy.loginfo('agv05_modbus_server: Register map %s' % self.reg_addr_map)

    def default(self, count, value=False):
        pass

    def reset(self):
        pass

    def validate(self, address, count=1):
        if count == 0 or address <= 0 or address + count > self.address_count + 1:
            return False
        return True

    def getValues(self, address, count=1):
        return [self.getValue(addr) for addr in range(address, address + count)]

    def setValues(self, address, values):
        for addr, value in enumerate(values, start=address):
            self.setValue(addr, value)

    def getValue(self, address):
        if address in self.reg_addr_map:
            return self.__get_register(self.reg_addr_map[address]).value
        return 0

    def setValue(self, address, value):
        if address in self.reg_addr_map:
            self.__set_register(self.reg_addr_map[address], value)
