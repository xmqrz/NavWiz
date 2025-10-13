from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import rospy


bi_outcomes = ['Equal', 'Not Equal']
tri_outcomes = ['Less Than', 'Equal', 'Greater Than']


def compare_bi_outcomes(u1, u2):
    if u1 == u2:
        return 'Equal'
    else:
        return 'Not Equal'


def compare_tri_outcomes(u1, u2):
    if u1 < u2:
        return 'Less Than'
    elif u1 > u2:
        return 'Greater Than'
    else:
        return 'Equal'


class CompareRegisters(Skill):

    class Meta:
        name = 'Compare Registers'
        params = [
            {
                'name': 'register_1',
                'type': 'Register',
                'description': 'Register 1',
            },
            {
                'name': 'register_2',
                'type': 'Register',
                'description': 'Register 2',
            },
        ]
        outcomes = tri_outcomes
        mutexes = []

    def __str__(self):
        return 'Comparing registers "%s" and "%s".' % (self.register_1.id, self.register_2.id)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        return compare_tri_outcomes(self.register_1.value, self.register_2.value)


class CompareRegisterInt(Skill):

    class Meta:
        name = 'Compare Register and Integer'
        params = [
            {
                'name': 'register',
                'type': 'Register',
                'description': 'Register',
            },
            {
                'name': 'integer',
                'type': 'int',
                'description': 'Integer',
            },
        ]
        outcomes = tri_outcomes
        mutexes = []

    def __str__(self):
        return 'Comparing register "%s" and integer (%d).' % (self.register.id, self.integer)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        return compare_tri_outcomes(self.register.value, self.integer)


class CompareInts(Skill):

    class Meta:
        name = 'Compare Integers'
        params = [
            {
                'name': 'integer_1',
                'type': 'int',
                'description': 'Integer 1',
            },
            {
                'name': 'integer_2',
                'type': 'int',
                'description': 'Integer 2',
            },
        ]
        outcomes = tri_outcomes
        mutexes = []

    def __str__(self):
        return 'Comparing integers (%d) and (%d).' % (self.integer_1, self.integer_2)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        return compare_tri_outcomes(self.integer_1, self.integer_2)


class CompareDoubles(Skill):

    class Meta:
        name = 'Compare Doubles'
        params = [
            {
                'name': 'double_1',
                'type': 'double',
                'description': 'Double 1',
            },
            {
                'name': 'double_2',
                'type': 'double',
                'description': 'Double 2',
            },
        ]
        outcomes = tri_outcomes
        mutexes = []

    def __str__(self):
        return 'Comparing doubles (%s) and (%s).' % (self.double_1, self.double_2)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        return compare_tri_outcomes(self.double_1, self.double_2)


class CompareStations(Skill):

    class Meta:
        name = 'Compare Stations'
        params = [
            {
                'name': 'station_1',
                'type': 'Station',
                'description': 'Station 1',
            },
            {
                'name': 'station_2',
                'type': 'Station',
                'description': 'Station 2',
            },
        ]
        outcomes = bi_outcomes
        mutexes = []

    def __str__(self):
        return 'Comparing stations "%s" and "%s".' % (self.station_1, self.station_2)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        return compare_bi_outcomes(self.station_1, self.station_2)


class CompareStationPrefix(Skill):

    class Meta:
        name = 'Compare Station Prefix'
        params = [
            {
                'name': 'station',
                'type': 'Station',
                'description': 'Station',
            },
            {
                'name': 'station_prefix',
                'type': 'str',
                'description': 'Station prefix',
            },
        ]
        outcomes = bi_outcomes
        mutexes = []

    def __str__(self):
        return 'Comparing station "%s" and prefix "%s".' % (self.station, self.station_prefix)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        return compare_bi_outcomes(self.station.startswith(self.station_prefix), True)


class CompareStationSuffix(Skill):

    class Meta:
        name = 'Compare Station Suffix'
        params = [
            {
                'name': 'station',
                'type': 'Station',
                'description': 'Station',
            },
            {
                'name': 'station_suffix',
                'type': 'str',
                'description': 'Station suffix',
            },
        ]
        outcomes = bi_outcomes
        mutexes = []

    def __str__(self):
        return 'Comparing station "%s" and suffix "%s".' % (self.station, self.station_suffix)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        return compare_bi_outcomes(self.station.endswith(self.station_suffix), True)


class CompareStrs(Skill):

    class Meta:
        name = 'Compare Strings'
        params = [
            {
                'name': 'string_1',
                'type': 'str',
                'description': 'String 1',
            },
            {
                'name': 'string_2',
                'type': 'str',
                'description': 'String 2',
            },
        ]
        outcomes = bi_outcomes
        mutexes = []

    def __str__(self):
        return 'Comparing strings "%s" and "%s".' % (self.string_1, self.string_2)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        return compare_bi_outcomes(self.string_1, self.string_2)


class CompareBools(Skill):

    class Meta:
        name = 'Compare Booleans'
        params = [
            {
                'name': 'boolean_1',
                'type': 'bool',
                'description': 'Boolean 1',
            },
            {
                'name': 'boolean_2',
                'type': 'bool',
                'description': 'Boolean 2',
            },
        ]
        outcomes = bi_outcomes
        mutexes = []

    def __str__(self):
        return 'Comparing booleans (%s) and (%s).' % (self.boolean_1, self.boolean_2)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        return compare_bi_outcomes(self.boolean_1, self.boolean_2)
