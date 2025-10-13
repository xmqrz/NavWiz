from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import logging
import rospy


class LogMessage(Skill):

    class Meta:
        name = 'Log Message'
        params = [
            {
                'name': 'level',
                'type': 'int',
                'description': 'Severity level (1=INFO, 2=WARNING, 3=ERROR)',
                'default': 1,
                'min': 1,
                'max': 3,
            },
            {
                'name': 'message',
                'type': 'str',
                'description': 'Message'
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __init__(self, *args, **kwargs):
        super(LogMessage, self).__init__(*args, **kwargs)
        self._level = {
            1: logging.INFO,
            2: logging.WARNING,
            3: logging.ERROR,
        }.get(self.level)

    def __str__(self):
        return 'Logging %s message.' % logging.getLevelName(self._level)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.robot.panel.log(self._level, self.message)
        return 'Done'
