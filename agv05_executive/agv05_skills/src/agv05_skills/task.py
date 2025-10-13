from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import rospy


class UpdateTaskProgress(Skill):

    class Meta:
        name = 'Update Task Progress'
        params = [
            {
                'name': 'progress',
                'type': 'str',
                'description': 'Task progress',
            },
        ]
        outcomes = ['Done']
        mutexes = ['task']

    def __str__(self):
        return 'Updating task progress to: %s .' % self.progress

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        if getattr(self.robot.models, 'update_task_progress', None):
            self.robot.models.update_task_progress(self.progress)
        return 'Done'


class AbortTask(Skill):

    class Meta:
        name = 'Abort Task'
        params = [
            {
                'name': 'error_msg',
                'type': 'str',
                'description': 'Error message',
            },
        ]
        outcomes = ['Done']
        mutexes = ['task']

    def __str__(self):
        return 'Aborting task via task skill: %s' % self.error_msg

    def execute(self, ud):
        rospy.loginfo('%s', self)
        self.fail(self.error_msg)
