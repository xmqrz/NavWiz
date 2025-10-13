from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import rospy
import std_msgs.msg
import subprocess
import time
import os


class DockCharger(Skill):
    TIMEOUT = 7

    class Meta:
        name = 'Dock Charger'
        params = []
        outcomes = ['Success', 'Fail']
        mutexes = []

    def __str__(self):
        return 'Docking to charger'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        self.robot.power.enable_charging(True)
        end_time = time.time() + float(rospy.get_param('/agv05_executor/dock_charger_timeout', self.TIMEOUT))
        charger_active = False

        while not self.preempt_requested():
            rospy.sleep(1.0)
            if self.robot.power.charging_status is True:
                return 'Success'
            elif self.robot.power.charging_status is None:
                charger_active = True
            elif self.robot.power.charging_status is False and (time.time() >= end_time or charger_active):
                return 'Fail'

        return 'Preempted'


class UndockCharger(Skill):

    class Meta:
        name = 'Undock Charger'
        params = []
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Undocking from charger'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        self.robot.power.enable_charging(False)

        while not self.preempt_requested():
            rospy.sleep(1.0)
            # return Done for both success / not success
            if self.robot.power.charging_status is False:
                return 'Done'
            if self.robot.power.charging_status is True:
                return 'Done'

        return 'Preempted'


class CheckBattery(Skill):

    class Meta:
        name = 'Check Battery'
        params = [
            {
                'name': 'percentage',
                'type': 'double',
                'description': 'Battery in %',
                'default': 25.0,
                'min': 0.0,
                'max': 100.0,
            },
        ]
        outcomes = ['Above', 'Below']
        mutexes = []

    def __str__(self):
        return 'Checking battery'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        if self.robot.power.battery_percentage >= self.percentage:
            return 'Above'
        else:
            return 'Below'


class CalibrateBattery(Skill):

    class Meta:
        name = 'Calibrate Battery'
        params = []
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Calibrating Battery'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        self.robot.power.recalibrate_battery()
        return 'Done'


class RebootAGV(Skill):

    class Meta:
        name = 'Reboot AGV'
        params = [
            {
                'name': 'mode',
                'type': 'int',
                'description': 'Reboot type: 0 - Soft Reboot, 1 - Hard Reboot',
                'default': 0,
                'min': 0,
                'max': 1,
            },
        ]
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Rebooting AGV'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'
        if self.mode == 0:
            subprocess.Popen(['sudo', 'service', 'kiosk', 'restart'], preexec_fn=os.setsid)
            subprocess.Popen(['sudo', 'supervisorctl', 'restart', 'agv05:*'], preexec_fn=os.setsid)
        elif self.mode == 1:
            subprocess.Popen(['sudo', 'reboot'])

        return 'Done'


class PowerOffAGV(Skill):

    class Meta:
        name = 'Power Off AGV'
        params = []
        outcomes = ['Done']
        mutexes = []

    def __str__(self):
        return 'Powering Off AGV'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        # Turn off embedded first, followed by PC itself.
        shutdown_pub = rospy.Publisher('/shutdown', std_msgs.msg.Bool, queue_size=1, latch=True)
        shutdown_pub.publish(std_msgs.msg.Bool(data=True))

        rospy.sleep(3.0)
        subprocess.Popen(['sudo', 'poweroff'])
        return 'Done'
