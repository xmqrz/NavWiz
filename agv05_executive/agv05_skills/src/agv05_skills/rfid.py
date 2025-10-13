from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import rospy

from .plan import ResetAgvPosition


class ReadRFIDAndCompareString(Skill):

    class Meta:
        name = 'Read RFID and Compare String'
        params = [
            {
                'name': 'rfid_location',
                'type': 'int',
                'description': 'Use front(0) or rear(1) RFID to use respective RFID.',
                'default': 0,
                'min': 0,
                'max': 1,
            },
            {
                'name': 'string',
                'type': 'str',
                'description': 'String to compare',
            },
            {
                'name': 'fail_task',
                'type': 'bool',
                'description': 'Whether to abort task when the strings are not equal.',
            },
        ]
        outcomes = ['Success', 'Fail']
        mutexes = []

    def __str__(self):
        return 'Read RFID and Compare String "%s".' % (self.string)

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        if self.rfid_location == 0:
            rfid_string = self.robot.rfid.front_rfid
        else:
            rfid_string = self.robot.rfid.rear_rfid

        if rfid_string == self.string:
            return 'Success'
        elif not self.fail_task:
            return 'Fail'
        else:
            self.fail('RFID differs. Read %s, supposed to be %s' % (
                rfid_string if rfid_string else '(empty)', self.string))


class ReadRFIDAndResetAgvPosition(ResetAgvPosition):
    HEADING_WARNING_STATION = ResetAgvPosition.HEADING_WARNING
    HEADING_WARNING_RFID = 'Resetting AGV position using RFID without a direction suffix, yet the current direction of AGV is unknown.'

    class Meta:
        name = 'Read RFID and Reset AGV Position'
        params = [
            {
                'name': 'rfid_location',
                'type': 'int',
                'description': 'Use front(0) or rear(1) RFID to use respective RFID.',
                'default': 0,
                'min': 0,
                'max': 1,
            },
        ]
        outcomes = ['Success', 'Fail']
        mutexes = []

    def __str__(self):
        return 'Read RFID and Reset AGV position.'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        if self.rfid_location == 0:
            rfid_string = self.robot.rfid.front_rfid
            flip_heading = False
        else:
            rfid_string = self.robot.rfid.rear_rfid
            flip_heading = True

        location = self.robot.models.get_location_from_rfid(rfid_string)
        if location:
            self.HEADING_WARNING = self.HEADING_WARNING_RFID
            self.set_position(*location, flip_heading=flip_heading)
            return 'Success'

        location = self.robot.models.get_location_from_station(rfid_string)
        if location:
            self.set_position(*location, flip_heading=flip_heading)
            return 'Success'

        return 'Fail'


class ResetRFID(Skill):

    class Meta:
        name = 'Reset RFID'
        params = [
            {
                'name': 'rfid_location',
                'type': 'int',
                'description': 'Use front(0) or rear(1) RFID to reset respective RFID.',
                'default': 0,
                'min': 0,
                'max': 1,
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Resetting RFID value.'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        # reset front or rear rfid
        self.robot.rfid._reset_rfid(self.rfid_location)
        return 'Done'
