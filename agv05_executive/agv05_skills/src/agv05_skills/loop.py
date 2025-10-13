from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import rospy


class Counter(Skill):

    class Meta:
        name = 'Counter'
        params = [
            {
                'name': 'register',
                'type': 'Register',
                'description': 'Register to use for the counter.',
            },
            {
                'name': 'limit',
                'type': 'int',
                'description': 'Number of counts.',
                'default': 5,
                'min': 0,
            },
        ]
        outcomes = ['Counted', 'Limit Reached']
        mutexes = []

    def __str__(self):
        return 'Counting on register %s' % self.register.id

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.register.increment(1)
        if self.register.value > self.limit:
            return 'Limit Reached'
        else:
            return 'Counted'
