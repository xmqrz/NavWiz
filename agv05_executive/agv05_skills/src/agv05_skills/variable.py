from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import rospy


class SetBool(Skill):

    class Meta:
        name = 'Set Bool'
        params = [
            {
                'name': 'bool_variable',
                'type': 'vbool',
                'description': 'Bool variable',
            },
            {
                'name': 'value',
                'type': 'bool',
                'description': 'Value to be assigned to the variable',
                'default': False,
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Setting variable "%s" to the value (%s).' % (self.bool_variable.id, self.value)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.bool_variable.value = self.value
        return 'Done'


class SetInt(Skill):

    class Meta:
        name = 'Set Integer'
        params = [
            {
                'name': 'int_variable',
                'type': 'vint',
                'description': 'Integer variable',
            },
            {
                'name': 'value',
                'type': 'int',
                'description': 'Value to be assigned to the variable',
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Setting variable "%s" to the value (%d).' % (self.int_variable.id, self.value)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.int_variable.value = self.value
        return 'Done'


class SetIntFromRegister(Skill):

    class Meta:
        name = 'Set Integer From Register'
        params = [
            {
                'name': 'int_variable',
                'type': 'vint',
                'description': 'Integer variable',
            },
            {
                'name': 'register',
                'type': 'Register',
                'description': 'Register to be copied to the variable',
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Setting variable "%s" to the value of register "%s".' % (self.int_variable.id, self.register.id)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.int_variable.value = self.register.value
        return 'Done'


class SetDouble(Skill):

    class Meta:
        name = 'Set Double'
        params = [
            {
                'name': 'double_variable',
                'type': 'vdouble',
                'description': 'Double variable',
            },
            {
                'name': 'value',
                'type': 'double',
                'description': 'Value to be assigned to the variable',
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Setting variable "%s" to the value (%f).' % (self.double_variable.id, self.value)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.double_variable.value = self.value
        return 'Done'


class SetStrMixin(Skill):

    class Meta:
        name = 'Set String'
        params = [
            {
                'name': 'str_variable',
                'type': 'vstr',
                'description': 'String variable',
            },
            {
                'name': 'value',
                'type': 'str',
                'description': 'Value to be assigned to the variable',
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Setting variable "%s" to the value (%s).' % (self.str_variable.id, self.value)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.str_variable.value = str(self.value)
        return 'Done'


class SetStr(SetStrMixin):
    pass


class SetStrFromInt(SetStrMixin):

    class Meta(SetStrMixin.Meta):
        name = 'Set String from Int'
        params = [
            {
                'name': 'str_variable',
                'type': 'vstr',
                'description': 'String variable',
            },
            {
                'name': 'value',
                'type': 'int',
                'description': 'Value to be assigned to the variable',
            },
        ]


class SetStrFromDouble(SetStrMixin):

    class Meta(SetStrMixin.Meta):
        name = 'Set String from Double'
        params = [
            {
                'name': 'str_variable',
                'type': 'vstr',
                'description': 'String variable',
            },
            {
                'name': 'value',
                'type': 'double',
                'description': 'Value to be assigned to the variable',
            },
        ]


class SetStrFromStation(SetStrMixin):

    class Meta(SetStrMixin.Meta):
        name = 'Set String from Station'
        params = [
            {
                'name': 'str_variable',
                'type': 'vstr',
                'description': 'String variable',
            },
            {
                'name': 'value',
                'type': 'Station',
                'description': 'Value to be assigned to the variable',
            },
        ]


class SetStation(Skill):

    class Meta:
        name = 'Set Station'
        params = [
            {
                'name': 'station_variable',
                'type': 'vStation',
                'description': 'Station variable',
            },
            {
                'name': 'value',
                'type': 'Station',
                'description': 'Station to be assigned to the variable',
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Setting variable "%s" to the station (%s).' % (self.station_variable.id, self.value)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.station_variable.value = self.value
        return 'Done'


class ComposeString(Skill):

    class Meta:
        name = 'Compose String'
        params = [
            {
                'name': 'template',
                'type': 'str',
                'description': 'Template string where the first "%s" will be replaced with the value',
            },
            {
                'name': 'value',
                'type': 'str',
                'description': 'Value to be used in template',
            },
            {
                'name': 'output_str_variable',
                'type': 'vstr',
                'description': 'Output string variable',
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Setting variable "%s" to the value (%s).' % (self.output_str_variable.id, self.__compose())

    def __compose(self):
        return self.template.replace('%s', self.value, 1)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.output_str_variable.value = self.__compose()
        return 'Done'
