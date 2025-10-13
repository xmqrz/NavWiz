from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import rospy


class PlayMusic(Skill):

    class Meta:
        name = 'Play Music'
        params = [
            {
                'name': 'playlist',
                'type': 'int',
                'description': 'Playlist No. (1 - 10)',
                'min': 1,
                'max': 10,
                'default': 1,
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Playing music from playlist %d' % self.playlist

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.robot.audio.play_music(self.playlist)
        rospy.sleep(0.1)
        return 'Done'


class PauseMusic(Skill):

    class Meta:
        name = 'Pause Music'
        params = []
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Pausing music'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.robot.audio.pause_music()
        rospy.sleep(0.1)
        return 'Done'


class StopMusic(Skill):

    class Meta:
        name = 'Stop Music'
        params = []
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Stopping music'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.robot.audio.stop_music()
        rospy.sleep(0.1)
        return 'Done'


class PlayAlarm(Skill):

    class Meta:
        name = 'Play Alarm'
        params = []
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Playing alarm'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.robot.audio.play_alarm()
        rospy.sleep(0.1)
        return 'Done'


class StopAlarm(Skill):

    class Meta:
        name = 'Stop Alarm'
        params = []
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Stopping alarm'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.robot.audio.stop_alarm()
        rospy.sleep(0.1)
        return 'Done'
