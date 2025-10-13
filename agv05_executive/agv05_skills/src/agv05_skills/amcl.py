from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
from std_msgs.msg import Bool
import rospy
import time


class ReflectorMatcher(object):

    def __init__(self):
        self.__matched = False
        self.__pub = rospy.Publisher('/match_reflector', Bool, queue_size=1, latch=True)
        self.__sub = rospy.Subscriber('/reflector_matched', Bool, self.handle_matched, queue_size=1)

    def start(self):
        self.__matched = False
        self.__pub.publish(True)

    def stop(self):
        self.__pub.publish(False)

    def is_matched(self):
        return self.__matched

    def handle_matched(self, msg):
        self.__matched = msg.data


matcher = ReflectorMatcher()


class MatchReflectors(Skill):

    class Meta:
        name = 'Match Reflectors'
        params = [
            {
                'name': 'timeout',
                'type': 'double',
                'description': 'Timeout duration in seconds.',
                'default': 1.0,
                'min': 0.1,
                'max': 10.0,
            },
        ]
        outcomes = ['Done', 'Failed']
        mutexes = []

    def __str__(self):
        return 'Matching reflectors'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        matcher.start()
        start_time = time.time()
        while not self.preempt_evt.wait(0.1):
            if matcher.is_matched():
                return 'Done'
            if time.time() - start_time > self.timeout:
                matcher.stop()
                return 'Failed'
        return 'Preempted'
