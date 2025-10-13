from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import rospy


class ModbusRead(Skill):

    class Meta:
        name = 'Modbus Read'
        params = [
            {
                'name': 'modbus',
                'type': 'int',
                'description': 'Modbus device (1-10)',
                'min': 1,
                'max': 10,
                'default': 1,
            },
            {
                'name': 'type',
                'type': 'int',
                'description': 'Modbus object type (0=coil, 1=discrete input)',
                'min': 0,
                'max': 1,
                'default': 0,
            },
            {
                'name': 'address',
                'type': 'int',
                'description': 'Read address (0-65535)',
                'min': 0,
                'max': 65535,
                'default': 0,
            },
        ]
        outcomes = ['True', 'False', 'Failed']
        mutexes = []

    def __str__(self):
        return 'Reading modbus %d %s address %d.' % (self.modbus, ('coil', 'discrete input')[self.type], self.address)

    def execute(self, ud):
        rospy.loginfo('%s', self)

        request_id = self.robot.modbus.read(self.modbus, self.type, self.address)
        while not self.preempt_requested():
            rospy.sleep(0.01)
            result = self.robot.modbus.get_result(request_id)
            if result is False:
                return 'Failed'
            elif result is not None:
                return 'True' if result else 'False'
        return 'Preempted'


class ModbusReadRegister(Skill):

    class Meta:
        name = 'Modbus Read Register'
        params = [
            {
                'name': 'modbus',
                'type': 'int',
                'description': 'Modbus device (1-10)',
                'min': 1,
                'max': 10,
                'default': 1,
            },
            {
                'name': 'type',
                'type': 'int',
                'description': 'Modbus object type (0=holding register, 1=input register)',
                'min': 0,
                'max': 1,
                'default': 0,
            },
            {
                'name': 'address',
                'type': 'int',
                'description': 'Read address (0-65535)',
                'min': 0,
                'max': 65535,
                'default': 0,
            },
            {
                'name': 'register',
                'type': 'Register',
                'description': 'Register',
            },
            {
                'name': 'input_mask',
                'type': 'int',
                'description': 'Input mask',
                'min': 0,
                'max': 65535,
                'default': 65535,
            },
        ]
        outcomes = ['Done', 'Failed']
        mutexes = []

    def __str__(self):
        return 'Reading modbus %d %s register address %d.' % (self.modbus, ('holding', 'input')[self.type], self.address)

    def execute(self, ud):
        rospy.loginfo('%s', self)

        request_id = self.robot.modbus.read(self.modbus, self.type + 2, self.address)
        while not self.preempt_requested():
            rospy.sleep(0.01)
            result = self.robot.modbus.get_result(request_id)
            if result is False:
                return 'Failed'
            elif result is not None:
                self.register.value = (result & (self.input_mask & 0xffff))
                return 'Done'
        return 'Preempted'


class ModbusWrite(Skill):

    class Meta:
        name = 'Modbus Write'
        params = [
            {
                'name': 'modbus',
                'type': 'int',
                'description': 'Modbus device (1-10)',
                'min': 1,
                'max': 10,
                'default': 1,
            },
            {
                'name': 'address',
                'type': 'int',
                'description': 'Write address (0-65535)',
                'min': 0,
                'max': 65535,
                'default': 0,
            },
            {
                'name': 'state',
                'type': 'bool',
                'description': 'Coil state',
                'default': False,
            },
        ]
        outcomes = ['Done', 'Failed']
        mutexes = []

    def __str__(self):
        return 'Writing remote io %d coil address %d to %s.' % (self.modbus, self.address, ['LO', 'HI'][self.state])

    def execute(self, ud):
        rospy.loginfo('%s', self)

        request_id = self.robot.modbus.write(self.modbus, 0, self.address, self.state)
        while not self.preempt_requested():
            rospy.sleep(0.01)
            result = self.robot.modbus.get_result(request_id)
            if result is False:
                return 'Failed'
            elif result is not None:
                return 'Done'
        return 'Preempted'


class ModbusWriteRegister(Skill):

    class Meta:
        name = 'Modbus Write Register'
        params = [
            {
                'name': 'modbus',
                'type': 'int',
                'description': 'Modbus device (1-10)',
                'min': 1,
                'max': 10,
                'default': 1,
            },
            {
                'name': 'address',
                'type': 'int',
                'description': 'Write address (0-65535)',
                'min': 0,
                'max': 65535,
                'default': 0,
            },
            {
                'name': 'register',
                'type': 'Register',
                'description': 'Register',
            },
            {
                'name': 'output_mask',
                'type': 'int',
                'description': 'Output mask',
                'min': 0,
                'max': 65535,
                'default': 65535,
            },
        ]
        outcomes = ['Done', 'Failed']
        mutexes = []

    def __str__(self):
        return 'Writing modbus %d register address %d.' % (self.modbus, self.address)

    def execute(self, ud):
        rospy.loginfo('%s', self)

        request_id = self.robot.modbus.write(self.modbus, 1, self.address, self.register.value, self.output_mask)
        while not self.preempt_requested():
            rospy.sleep(0.01)
            result = self.robot.modbus.get_result(request_id)
            if result is False:
                return 'Failed'
            elif result is not None:
                return 'Done'
        return 'Preempted'
