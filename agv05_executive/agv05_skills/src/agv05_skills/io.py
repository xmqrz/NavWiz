from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import rospy


class IoRead(Skill):

    class Meta:
        name = 'Io Read'
        params = [
            {
                'name': 'port',
                'type': 'int',
                'description': 'Input port (1-8)',
                'min': 1,
                'max': 8,
                'default': 1,
            },
            {
                'name': 'input',
                'type': 'int',
                'description': 'Input pin (0-15)',
                'min': 0,
                'max': 15,
                'default': 0,
                'version': 1,
            },
        ]
        outcomes = ['True', 'False']
        mutexes = []

    def __str__(self):
        return 'Reading input %d-%d.' % (self.port, self.input)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        if self.robot.io.read_input(self.port, self.input):
            return 'True'
        else:
            return 'False'


class IoReadRegister(Skill):

    class Meta:
        name = 'Io Read Register'
        params = [
            {
                'name': 'port',
                'type': 'int',
                'description': 'Input port (1-8)',
                'min': 1,
                'max': 8,
                'default': 1,
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
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Reading input register from port %d.' % self.port

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        result = self.robot.io.read_input_register(self.port)
        self.register.value = result & self.input_mask
        return 'Done'


class IoWrite(Skill):

    class Meta:
        name = 'Io Write'
        params = [
            {
                'name': 'port',
                'type': 'int',
                'description': 'Output port (1-8)',
                'min': 1,
                'max': 8,
                'default': 1,
            },
            {
                'name': 'output',
                'type': 'int',
                'description': 'Output pin (0-15)',
                'min': 0,
                'max': 15,
                'default': 0,
                'version': 1,
            },
            {
                'name': 'state',
                'type': 'bool',
                'description': 'Output state',
                'default': False,
            }
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Writing output %d-%d to %s.' % (self.port, self.output, ['LO', 'HI'][self.state])

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.robot.io.write_output(self.port, self.output, self.state)
        return 'Done'


class IoWriteRegister(Skill):

    class Meta:
        name = 'Io Write Register'
        params = [
            {
                'name': 'port',
                'type': 'int',
                'description': 'Output port (1-8)',
                'min': 1,
                'max': 8,
                'default': 1,
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
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Writing output register to port %d.' % self.port

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.robot.io.write_output_register(self.port, self.register.value, self.output_mask)
        return 'Done'
