#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from pymodbus.datastore import ModbusSparseDataBlock
from agv05_msgs.msg import SafetyTriggers
import threading
import rospy


class InputRegister(ModbusSparseDataBlock):
    '''custom pymodbus datastore for holding register'''

    # max 0xFFFF
    address_count = 1

    @classmethod
    def create(klass):
        return klass()

    def __init__(self):

        self._lock_safety = threading.Lock()

        # 1 - SafetyTrigger
        with self._lock_safety:
            self.safetyTrigger = 0
        rospy.Subscriber('agv05/safety/safety_triggers', SafetyTriggers, self.callbackSafetyTrigger)

    def default(self, count, value=False):
        pass

    def reset(self):
        pass

    def validate(self, address, count=1):
        if count == 0 or address <= 0 or address + count > self.address_count + 1:
            return False
        return True

    def setValues(self, address, values):
        pass

    def getValues(self, address, count=1):
        return [self.getValue(addr) for addr in range(address, address + count)]

    def getValue(self, address):
        if address == 1:  # SafetyTrigger
            with self._lock_safety:
                return self.safetyTrigger
        return 0

    def callbackSafetyTrigger(self, msg):
        _safetyTrigger = (
            msg.bumper_front << 0 |
            msg.bumper_rear << 1 |
            msg.emergency_button << 2 |
            msg.safety_in_1 << 3 |
            msg.safety_in_2 << 3 |
            msg.nav_trigger << 4 |
            msg.charger_connected << 5 |
            msg.system_error << 6 |
            msg.motor_fault << 7 |
            msg.wheel_slippage << 8
        )

        with self._lock_safety:
            self.safetyTrigger = _safetyTrigger
