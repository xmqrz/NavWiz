from __future__ import absolute_import
from __future__ import unicode_literals

import agv05_msgs.msg
import rospy

from .agv05 import Agv05
from .base import TrackedBase
from .robot import TrackedMixin

Goal = agv05_msgs.msg.NavActionGoal
NavControl = agv05_msgs.msg.NavControl


class TrackedAgv05Base(TrackedBase):
    manual_cmd_vel_topic = '/agv05/nav/manual_cmd_vel'
    action_server = '/nav_action'
    action_spec = agv05_msgs.msg.NavActionAction
    nav_control = '/agv05/nav/nav_control'

    def __init__(self, robot):
        super(TrackedAgv05Base, self).__init__(robot)
        self.__nav_control_pub = rospy.Publisher(self.nav_control, NavControl, queue_size=1)

    def forward(self, constraints, straight=True):
        params = {
            'nav': Goal.NAV_FORWARD if straight else Goal.NAV_BEZIER_FORWARD,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.forward()

    def reverse(self, constraints, straight=True):
        params = {
            'nav': Goal.NAV_REVERSE if straight else Goal.NAV_BEZIER_REVERSE,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.reverse()

    def branch_forward_left(self, constraints, straight=True, track_dir=True):
        params = {
            'nav': Goal.NAV_FORWARD_LEFT if straight else Goal.NAV_BEZIER_FORWARD_LEFT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        if track_dir:
            self.map_tracker.branch_forward_left()
        else:
            self.map_tracker.forward()

    def branch_forward_right(self, constraints, straight=True, track_dir=True):
        params = {
            'nav': Goal.NAV_FORWARD_RIGHT if straight else Goal.NAV_BEZIER_FORWARD_RIGHT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        if track_dir:
            self.map_tracker.branch_forward_right()
        else:
            self.map_tracker.forward()

    def branch_reverse_left(self, constraints, straight=True, track_dir=True):
        params = {
            'nav': Goal.NAV_REVERSE_LEFT if straight else Goal.NAV_BEZIER_REVERSE_LEFT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        if track_dir:
            self.map_tracker.branch_reverse_left()
        else:
            self.map_tracker.reverse()

    def branch_reverse_right(self, constraints, straight=True, track_dir=True):
        params = {
            'nav': Goal.NAV_REVERSE_RIGHT if straight else Goal.NAV_BEZIER_REVERSE_RIGHT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        if track_dir:
            self.map_tracker.branch_reverse_right()
        else:
            self.map_tracker.reverse()

    def rotate_left(self, constraints):
        params = {
            'nav': Goal.NAV_ROTATE_LEFT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.rotate_left()

    def rotate_right(self, constraints):
        params = {
            'nav': Goal.NAV_ROTATE_RIGHT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.rotate_right()

    def uturn_left(self, constraints):
        params = {
            'nav': Goal.NAV_UTURN_LEFT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.uturn_left()

    def uturn_right(self, constraints):
        params = {
            'nav': Goal.NAV_UTURN_RIGHT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.uturn_right()

    def rotate3q_left(self, constraints):
        params = {
            'nav': Goal.NAV_ROTATE3Q_LEFT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.rotate3q_left()

    def rotate3q_right(self, constraints):
        params = {
            'nav': Goal.NAV_ROTATE3Q_RIGHT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.rotate3q_right()

    def search_line_left(self, constraints):
        params = {
            'nav': Goal.NAV_SEARCH_LINE_LEFT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def search_line_right(self, constraints):
        params = {
            'nav': Goal.NAV_SEARCH_LINE_RIGHT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def wait_traffic(self, constraints={}):
        params = {
            'nav': Goal.NAV_WAIT_TRAFFIC,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def free_motor(self, constraints={}):
        params = {
            'nav': Goal.NAV_FREE_MOTOR,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.set_confused()

    def manual_control(self, constraints={}):
        params = {
            'nav': Goal.NAV_MANUAL_CONTROL,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.set_confused()

    def line_calibrate(self, constraints={}):
        params = {
            'nav': Goal.NAV_LINE_CALIBRATE,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.set_confused()

    def line_tune_pid(self, constraints={}):
        params = {
            'nav': Goal.NAV_LINE_TUNE_PID,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.set_confused()

    def select_nav_profile(self, profile, constraints={}):
        if profile < 1 or profile > 5:
            return

        params = {
            'nav': Goal.NAV_SELECT_NAV_PROFILE1 + profile - 1,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def select_laser_profile(self, profile, constraints={}):
        if profile < 1 or profile > 10:
            return

        params = {
            'nav': Goal.NAV_SELECT_LASER_PROFILE1 + profile - 1,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def scan_laser_area(self, area, constraints={}):
        if area < 1 or area > 31:
            return

        params = {
            'nav': Goal.NAV_SCAN_LASER_AREA1 + area - 1,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def stop(self, constraints={}, make_confused=True):
        self.cancel_goal()
        self.__nav_control_pub.publish(
            NavControl(control=NavControl.CONTROL_ABORT))
        params = {
            'nav': Goal.NAV_IDLE,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.stop()
        if make_confused:
            self.map_tracker.set_confused()

    def stop_wait_traffic(self, constraints={}):
        self.stop(constraints, False)

    def cancel_forward_flag(self):
        self.__nav_control_pub.publish(
            NavControl(control=NavControl.CONTROL_CANCEL_FORWARD_FLAG))
        params = {
            'nav': Goal.NAV_IDLE,
        }
        self.make_and_send_goal(**params)

    def pause(self):
        self.__nav_control_pub.publish(
            NavControl(control=NavControl.CONTROL_PAUSE))

    def resume(self):
        self.__nav_control_pub.publish(
            NavControl(control=NavControl.CONTROL_CONTINUE))

    def safety_resume(self):
        self.__nav_control_pub.publish(
            NavControl(control=NavControl.CONTROL_SAFETY_RESUME))

    # override methods in ActionClientMixin
    def wait_for_result(self, timeout=rospy.Duration()):
        result = super(TrackedAgv05Base, self).wait_for_result(timeout)
        if result:
            self.map_tracker.stop()
        return result


class TrackedAgv05(TrackedMixin, Agv05):
    name = 'tracked_agv05'
    base_cls = TrackedAgv05Base
