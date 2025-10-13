from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import rospy
import time


class SetRegister(Skill):

    class Meta:
        name = 'Set Register Value'
        params = [
            {
                'name': 'register',
                'type': 'Register',
                'description': 'Register',
            },
            {
                'name': 'value',
                'type': 'int',
                'description': 'Value to be assigned to the register',
                'default': 0,
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Setting register "%s" to the value (%d).' % (self.register.id, self.value)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.register.value = self.value
        return 'Done'


class CompareAndSetRegister(Skill):

    class Meta:
        name = 'Compare and Set Register'
        params = [
            {
                'name': 'register',
                'type': 'Register',
                'description': 'Register',
            },
            {
                'name': 'expected_value',
                'type': 'int',
                'description': 'Expected value',
            },
            {
                'name': 'new_value',
                'type': 'int',
                'description': 'New value'
            },
        ]
        outcomes = ['Equal', 'Not Equal']
        mutexes = []

    def __str__(self):
        return 'Compare register "%s" and (%d), and if equal set register to (%d).' % \
            (self.register.id, self.expected_value, self.new_value)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        equal = self.register.compare_and_set(self.expected_value, self.new_value)
        return 'Equal' if equal else 'Not Equal'


class IncrRegister(Skill):

    class Meta:
        name = 'Increment Register'
        params = [
            {
                'name': 'register',
                'type': 'Register',
                'description': 'Register',
            },
            {
                'name': 'value',
                'type': 'int',
                'description': 'Increment value',
                'default': 1,
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Increment register "%s" by (%d).' % (self.register.id, self.value)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.register.increment(self.value)
        return 'Done'


class DecrRegister(Skill):

    class Meta:
        name = 'Decrement Register'
        params = [
            {
                'name': 'register',
                'type': 'Register',
                'description': 'Register',
            },
            {
                'name': 'value',
                'type': 'int',
                'description': 'Decrement value',
                'default': 1,
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Decrement register "%s" by (%d).' % (self.register.id, self.value)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.register.decrement(self.value)
        return 'Done'


class AddRegister(Skill):

    class Meta:
        name = 'Add Register'
        params = [
            {
                'name': 'destination',
                'type': 'Register',
                'description': 'Destination register where the result is stored.',
            },
            {
                'name': 'source',
                'type': 'Register',
                'description': 'Add its value to the destination register.',
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Add register "%s" to "%s".' % (self.source.id, self.destination.id)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.destination.increment(self.source.value)
        return 'Done'


class SubtractRegister(Skill):

    class Meta:
        name = 'Subtract Register'
        params = [
            {
                'name': 'destination',
                'type': 'Register',
                'description': 'Destination register where the result is stored.',
            },
            {
                'name': 'source',
                'type': 'Register',
                'description': 'Subtract its value from the destination register.',
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Subtract register "%s" from "%s".' % (self.source.id, self.destination.id)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.destination.decrement(self.source.value)
        return 'Done'


class MultiplyRegister(Skill):

    class Meta:
        name = 'Multiply Register'
        params = [
            {
                'name': 'destination',
                'type': 'Register',
                'description': 'Destination register where the result is stored.',
            },
            {
                'name': 'source',
                'type': 'Register',
                'description': 'Multiply the destination register by this value.',
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Multiply register "%s" by "%s.' % (self.destination.id, self.source.id)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.destination.multiply(self.source.value)
        return 'Done'


class DivideRegister(Skill):

    class Meta:
        name = 'Divide Register'
        params = [
            {
                'name': 'destination',
                'type': 'Register',
                'description': 'Destination register where the quotient is stored.',
            },
            {
                'name': 'source',
                'type': 'Register',
                'description': 'Divide the destination register by this value.',
            },
            {
                'name': 'remainder',
                'type': 'Register',
                'description': 'Destination register where the remainder is stored.',
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Divide register "%s" by "%s", with remainder stored in "%s".' % (
            self.destination.id, self.source.id, self.remainder.id)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.remainder.value = self.destination.divide(self.source.value)
        return 'Done'


class CopyRegister(Skill):

    class Meta:
        name = 'Copy Register'
        params = [
            {
                'name': 'destination',
                'type': 'Register',
                'description': 'Destination register where the value is copied.',
            },
            {
                'name': 'source',
                'type': 'Register',
                'description': 'Register to be copied from.',
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Copy from register "%s" into register "%s".' % (self.source.id, self.destination.id)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.destination.value = self.source.value
        return 'Done'


class DisplayRegister(Skill):

    class Meta:
        name = 'Display Register Value'
        params = [
            {
                'name': 'register',
                'type': 'Register',
                'description': 'Register',
            },
            {
                'name': 'message_format',
                'type': 'str',
                'description': 'The message text. Any occurence of "%d" will be substituted by the register\'s value.',
                'default': '%d',
            },
            {
                'name': 'timeout',
                'type': 'int',
                'description': 'Display duration in seconds.',
                'default': 30,
                'min': -1,
                'max': 9999,
            },
        ]
        outcomes = ['OK', 'Timeout']
        mutexes = ['panel']

    def __init__(self, *args, **kwargs):
        super(DisplayRegister, self).__init__(*args, **kwargs)
        if self.timeout < 0:
            self.timeout = None

    def __str__(self):
        if self.timeout is None:
            return 'Display message with register "%s" forever until user acknowledge.' % self.register.id
        else:
            return 'Display message with register "%s" until user acknowledge unless timeout at %d second(s).' % (self.register.id, self.timeout)

    def execute(self, ud):
        rospy.loginfo('%s', self)

        if not hasattr(self.robot, 'panel'):
            rospy.logwarn('DisplayRegister: Robot panel not available. Waiting for timeout only.')
            if self.preempt_evt.wait(self.timeout):
                return 'Preempted'
            else:
                return 'Timeout'

        start_time = time.time()
        fragments = self.message_format.split('%d')
        message = ('%s' % self.register.value).join(fragments)
        with self.robot.panel.show_popup_alert(message, self.timeout):
            while not self.preempt_requested():
                if self.timeout is not None:
                    if time.time() - start_time > self.timeout:
                        return 'Timeout'

                res = self.robot.panel.wait_button_release(1)
                if res == 'start_button':
                    return 'OK'

            return 'Preempted'
