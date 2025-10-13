from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import rospy
import time


wait_params = [
    {
        'name': 'message',
        'type': 'str',
        'description': 'Message to be displayed.',
    },
    {
        'name': 'timeout',
        'type': 'int',
        'description': 'Wait duration in seconds.',
        'default': 30,
        'min': -1,
        'max': 9999,
    },
]


class Wait(Skill):

    class Meta:
        name = 'Wait'
        params = wait_params
        outcomes = ['Timeout']
        mutexes = ['panel']

    def __init__(self, *args, **kwargs):
        super(Wait, self).__init__(*args, **kwargs)
        if self.timeout < 0:
            self.timeout = None

    def __str__(self):
        if self.timeout is None:
            return '!!! Wait forever'
        else:
            return 'Wait for %d second(s).' % self.timeout

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if not hasattr(self.robot, 'panel'):
            if self.preempt_evt.wait(self.timeout):
                return 'Preempted'
            else:
                return 'Timeout'

        with self.robot.panel.show_popup_non_interactive(self.message, self.timeout):
            if self.preempt_evt.wait(self.timeout):
                return 'Preempted'
            else:
                return 'Timeout'


class WaitAcknowledge(Skill):

    class Meta:
        name = 'Wait Acknowledge'
        params = wait_params
        outcomes = ['OK', 'Timeout']
        mutexes = ['panel']

    def __init__(self, *args, **kwargs):
        super(WaitAcknowledge, self).__init__(*args, **kwargs)
        if self.timeout < 0:
            self.timeout = None

    def __str__(self):
        if self.timeout is None:
            return 'Wait forever until user acknowledge.'
        else:
            return 'Wait until user acknowledge unless timeout at %d second(s).' % self.timeout

    def execute(self, ud):
        rospy.loginfo('%s', self)

        if not hasattr(self.robot, 'panel'):
            rospy.logwarn('WaitAcknowledge: Robot panel not available. Waiting for timeout only.')
            if self.preempt_evt.wait(self.timeout):
                return 'Preempted'
            else:
                return 'Timeout'

        start_time = time.time()
        with self.robot.panel.show_popup_alert(self.message, self.timeout):
            while not self.preempt_requested():
                if self.timeout is not None:
                    if time.time() - start_time > self.timeout:
                        return 'Timeout'

                res = self.robot.panel.wait_button_release(1)
                if res == 'start_button':
                    return 'OK'

            return 'Preempted'


class WaitConfirm(Skill):

    class Meta:
        name = 'Wait Confirm'
        params = wait_params
        outcomes = ['Yes', 'No', 'Timeout']
        mutexes = ['panel']

    def __init__(self, *args, **kwargs):
        super(WaitConfirm, self).__init__(*args, **kwargs)
        if self.timeout < 0:
            self.timeout = None

    def __str__(self):
        if self.timeout is None:
            return 'Wait forever until button pressed.'
        else:
            return 'Wait until button pressed unless timeout at %d second(s).' % self.timeout

    def execute(self, ud):
        rospy.loginfo('%s', self)

        if not hasattr(self.robot, 'panel'):
            rospy.logwarn('WaitConfirm: Robot panel not available. Waiting for timeout only.')
            if self.preempt_evt.wait(self.timeout):
                return 'Preempted'
            else:
                return 'Timeout'

        start_time = time.time()
        with self.robot.panel.show_popup_confirm(self.message, self.timeout):
            while not self.preempt_requested():
                if self.timeout is not None:
                    if time.time() - start_time > self.timeout:
                        return 'Timeout'

                res = self.robot.panel.wait_button_release(1)
                if res == 'start_button':
                    return 'Yes'
                elif res == 'stop_button':
                    return 'No'

            return 'Preempted'


class WaitKeypad(Skill):

    class Meta:
        name = 'Wait Keypad'
        params = wait_params
        outcomes = ['Key %c' % k for k in '0123456789AB'] + ['Timeout']
        mutexes = ['panel']

    def __init__(self, *args, **kwargs):
        super(WaitKeypad, self).__init__(*args, **kwargs)
        if self.timeout < 0:
            self.timeout = None

    def __str__(self):
        if self.timeout is None:
            return 'Wait forever until keypad pressed.'
        else:
            return 'Wait until keypad pressed unless timeout at %d second(s).' % self.timeout

    def execute(self, ud):
        rospy.loginfo('%s', self)

        if not hasattr(self.robot, 'panel'):
            rospy.logwarn('WaitKeypad: Robot panel not available. Waiting for timeout only.')
            if self.preempt_evt.wait(self.timeout):
                return 'Preempted'
            else:
                return 'Timeout'

        start_time = time.time()
        with self.robot.panel.show_popup_keypad(self.message, self.timeout):
            while not self.preempt_requested():
                if self.timeout is not None:
                    if time.time() - start_time > self.timeout:
                        return 'Timeout'

                res = self.robot.panel.wait_keypad(1)
                if res:
                    return res.title().replace('_', ' ')

            return 'Preempted'


class WaitResume(Skill):

    class Meta:
        name = 'Wait Resume'
        params = wait_params[:1]
        outcomes = ['Done']
        mutexes = ['panel']

    def __str__(self):
        return 'Wait forever until resumed.'

    def execute(self, ud):
        rospy.loginfo('%s', self)

        self.is_paused(True)

        if not hasattr(self.robot, 'panel'):
            return self._wait_resume()

        with self.robot.panel.show_popup_non_interactive(self.message, None):
            return self._wait_resume()

    def _wait_resume(self):
        while self.is_paused():
            if self.preempt_evt.wait(1.0):
                return 'Preempted'
        return 'Done'


class Sleep(Skill):

    class Meta:
        name = 'Sleep'
        params = [
            {
                'name': 'timeout',
                'type': 'double',
                'description': 'Sleep duration in seconds.',
                'default': 1.0,
                'min': 0,
                'max': 9999,
            },
        ]
        outcomes = ['Timeout']
        mutexes = []

    def __str__(self):
        return 'Sleep for %f second(s).' % self.timeout

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_evt.wait(self.timeout):
            return 'Preempted'
        else:
            return 'Timeout'
