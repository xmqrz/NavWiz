from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import datetime
import rospy


class CheckTime(Skill):

    class Meta:
        name = 'Check Time'
        params = [
            {
                'name': 'start_hour',
                'type': 'int',
                'description': 'Hour field of the start time',
                'min': 0,
                'max': 23,
                'default': 0,
            },
            {
                'name': 'start_minute',
                'type': 'int',
                'description': 'Minute field of the start time',
                'min': 0,
                'max': 59,
                'default': 0,
            },
            {
                'name': 'start_second',
                'type': 'int',
                'description': 'Second field of the start time',
                'min': 0,
                'max': 59,
                'default': 0,
            },
            {
                'name': 'end_hour',
                'type': 'int',
                'description': 'Hour field of the end time',
                'min': 0,
                'max': 23,
                'default': 23,
            },
            {
                'name': 'end_minute',
                'type': 'int',
                'description': 'Minute field of the end time',
                'min': 0,
                'max': 59,
                'default': 59,
            },
            {
                'name': 'end_second',
                'type': 'int',
                'description': 'Second field of the end time',
                'min': 0,
                'max': 59,
                'default': 59,
            },
        ]
        outcomes = ['Between', 'Outside']
        mutexes = []

    def __init__(self, *args, **kwargs):
        super(CheckTime, self).__init__(*args, **kwargs)
        self.start_time = datetime.time(self.start_hour, self.start_minute, self.start_second)
        self.end_time = datetime.time(self.end_hour, self.end_minute, self.end_second)

    def __str__(self):
        return 'Checking the time now whether it is between %s and %s' % (self.start_time, self.end_time)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        now = datetime.datetime.now().time()
        if self.start_time <= self.end_time:
            return 'Between' if self.start_time <= now < self.end_time else 'Outside'
        else:
            return 'Outside' if self.end_time <= now < self.start_time else 'Between'
