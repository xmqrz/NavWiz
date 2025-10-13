from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import rospy


class RemoteIoRead(Skill):

    class Meta:
        name = 'Remote Io Read'
        params = [
            {
                'name': 'remote_io',
                'type': 'int',
                'description': 'Remote IO device (1-10)',
                'min': 1,
                'max': 10,
                'default': 1,
                'version': 1,
            },
            {
                'name': 'input',
                'type': 'int',
                'description': 'Input pin (0-15)',
                'min': 0,
                'max': 15,
                'default': 0,
            },
        ]
        outcomes = ['True', 'False', 'Failed']
        mutexes = []

    def __str__(self):
        return 'Reading remote io %d input %d.' % (self.remote_io, self.input)

    def execute(self, ud):
        rospy.loginfo('%s', self)

        request_id = self.robot.remote_io.read_input(self.remote_io)
        while not self.preempt_requested():
            rospy.sleep(0.01)
            result = self.robot.remote_io.get_result(request_id)
            if result is False:
                return 'Failed'
            elif result is not None:
                return 'True' if result & (1 << self.input) else 'False'
        return 'Preempted'


class RemoteIoReadRegister(Skill):

    class Meta:
        name = 'Remote Io Read Register'
        params = [
            {
                'name': 'remote_io',
                'type': 'int',
                'description': 'Remote IO device (1-10)',
                'min': 1,
                'max': 10,
                'default': 1,
                'version': 1,
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
                'default': 255,
            },
        ]
        outcomes = ['Done', 'Failed']
        mutexes = []

    def __str__(self):
        return 'Reading remote io %d input register.' % self.remote_io

    def execute(self, ud):
        rospy.loginfo('%s', self)

        request_id = self.robot.remote_io.read_input(self.remote_io)
        while not self.preempt_requested():
            rospy.sleep(0.01)
            result = self.robot.remote_io.get_result(request_id)
            if result is False:
                return 'Failed'
            elif result is not None:
                self.register.value = (result & (self.input_mask & 0xffff))
                return 'Done'
        return 'Preempted'


class RemoteIoWrite(Skill):

    class Meta:
        name = 'Remote Io Write'
        params = [
            {
                'name': 'remote_io',
                'type': 'int',
                'description': 'Remote IO device (1-10)',
                'min': 1,
                'max': 10,
                'default': 1,
                'version': 1,
            },
            {
                'name': 'output',
                'type': 'int',
                'description': 'Output pin (0-15)',
                'min': 0,
                'max': 15,
                'default': 0,
            },
            {
                'name': 'state',
                'type': 'bool',
                'description': 'Output state',
                'default': False,
            }
        ]
        outcomes = ['Done', 'Failed']
        mutexes = []

    def __str__(self):
        return 'Writing remote io %d output %d to %s.' % (self.remote_io, self.output, ['LO', 'HI'][self.state])

    def execute(self, ud):
        rospy.loginfo('%s', self)

        data = 0xffff if self.state else 0
        mask = 1 << self.output
        request_id = self.robot.remote_io.write_output(self.remote_io, data, mask)

        while not self.preempt_requested():
            rospy.sleep(0.01)
            result = self.robot.remote_io.get_result(request_id)
            if result is False:
                return 'Failed'
            elif result is not None:
                return 'Done'
        return 'Preempted'


class RemoteIoWriteRegister(Skill):

    class Meta:
        name = 'Remote Io Write Register'
        params = [
            {
                'name': 'remote_io',
                'type': 'int',
                'description': 'Remote IO device (1-10)',
                'min': 1,
                'max': 10,
                'default': 1,
                'version': 1,
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
                'default': 255,
            },
        ]
        outcomes = ['Done', 'Failed']
        mutexes = []

    def __str__(self):
        return 'Writing remote io %d register.' % self.remote_io

    def execute(self, ud):
        rospy.loginfo('%s', self)

        request_id = self.robot.remote_io.write_output(self.remote_io, self.register.value, self.output_mask)
        while not self.preempt_requested():
            rospy.sleep(0.01)
            result = self.robot.remote_io.get_result(request_id)
            if result is False:
                return 'Failed'
            elif result is not None:
                return 'Done'
        return 'Preempted'
