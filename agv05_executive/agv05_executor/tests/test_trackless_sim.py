#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_msgs.msg import NavxActionGoal as Goal
from django.conf import settings as django_settings
from geometry_msgs.msg import Pose2D
import django
import os
import rospy
import rostest
import sys
import unittest

os.environ['TRACKLESS'] = '1'
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "agv05_webserver.settings-test")
django.setup()

if True:  # prevent autopep8 messing up the import sequence
    from agv05_executor.models.map_tracker_x import MapTrackerX
    from agv05_executor.robots.trackless_sim import TracklessSim, TracklessSimBase

    from test_tracked_sim import TestTrackedSim


class TestTracklessSim(TestTrackedSim):

    @classmethod
    def setUpClass(cls):
        cls.robot = TracklessSim()
        cls.base = cls.robot.base

    def test_init(self):
        self.assertTrue(django_settings.TRACKLESS)
        self.assertTrue(self.robot.trackless)
        self.assertIsInstance(self.robot.base, TracklessSimBase)
        self.assertIsInstance(self.robot.base.map_tracker, MapTrackerX)

    def test_dynamic_waypoint_forward(self):
        self._before_motion()
        end = {'x': 10, 'y': 4}
        heading = 30
        self.base.dynamic_waypoint_forward(end, heading, {
            'speed': 0.6,
            'goal_tolerance': -1.0,
            'enable_sensor': True,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_DYNAMIC_FORWARD)

        self._check_result()
        self._after_motion()

    def test_dynamic_waypoint_reverse(self):
        self._before_motion()
        end = {'x': -4.5, 'y': 9.2}
        heading = 290
        self.base.dynamic_waypoint_reverse(end, heading, {
            'speed': 0.6,
            'goal_tolerance': 0.2,
            'enable_sensor': True,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_DYNAMIC_REVERSE)

        self._check_result()
        self._after_motion()

    def test_dynamic_line_forward(self):
        self._before_motion()
        start = {'x': -2.1, 'y': -3.2}
        end = {'x': 3.4, 'y': -9.0}
        self.base.dynamic_line_forward(start, end, {
            'speed': 0.9,
            'goal_tolerance': 0.1,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Forward')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_DYNAMIC_LINE_FORWARD)

        # total duration = distance / speed + deceleration_time = 7.99 / 0.9 + 0.9 = 8.88 = 9.78
        self._check_result(duration=9.78)
        self._after_motion()

    def test_dynamic_line_reverse(self):
        self._before_motion()
        start = {'x': 6.3, 'y': 3.8}
        end = {'x': 3.9, 'y': 4.0}
        self.base.dynamic_line_reverse(start, end, {
            'speed': 0.4,
            'goal_tolerance': -2.0,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Reverse')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_DYNAMIC_LINE_REVERSE)

        # total duration = distance / speed + deceleration_time = 2.41 / 0.4 + 0.4 = 6.42
        self._check_result(duration=6.42)
        self._after_motion()

    def test_dynamic_bezier_forward(self):
        self._before_motion()
        start = {'x': -2.1, 'y': -3.2}
        end = {'x': 3.4, 'y': -9.0}
        cp1 = {'x': -1.0, 'y': -4.3}
        cp2 = {'x': 2.3, 'y': -7.6}
        self.base.dynamic_bezier_forward(start, end, cp1, cp2, {
            'speed': 0.5,
            'goal_tolerance': -2.0,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Forward')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_DYNAMIC_BEZIER_FORWARD)

        # total duration = distance / speed + deceleration_time = 8.0 / 0.5 + 0.5 = 16.5
        self._check_result(duration=16.5)
        self._after_motion()

    def test_dynamic_bezier_reverse(self):
        self._before_motion()
        start = {'x': 6.3, 'y': 3.8}
        end = {'x': 3.9, 'y': 4.0}
        cp1 = {'x': 5.8, 'y': 3.6}
        cp2 = {'x': 4.5, 'y': 4.4}
        self.base.dynamic_bezier_reverse(start, end, cp1, cp2, {
            'speed': 0.6,
            'goal_tolerance': 0.1,
            'enable_sensor': True,
            'next_motion': Goal.MOTION_NONSTOP_BEZIER,
            'next_speed': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Reverse')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_DYNAMIC_BEZIER_REVERSE)

        # total duration = distance / speed = 2.46 / 0.6 = 4.1
        self._check_result(duration=4.1)
        self._after_motion()

    def test_move_forward(self):
        self._before_motion()
        start = {'x': 0, 'y': 0}
        end = {'x': 3, 'y': -4}
        self.base.move_forward(start, end, {
            'speed': 10.0,  # default to 0.7
            'enable_sensor': True,
            'next_motion': Goal.MOTION_NONSTOP_BEZIER,
            'next_speed': 0,
            'sense_line': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Forward')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_FORWARD)

        # total duration = distance / speed = 5 / 0.7 = 7.14
        self._check_result(duration=7.14)
        self._after_motion()

    def test_move_reverse(self):
        self._before_motion()
        start = {'x': 0, 'y': 0}
        end = {'x': 3, 'y': 4}
        self.base.move_reverse(start, end, {
            'speed': 3.0,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
            'sense_line': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Reverse')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_REVERSE)

        # total duration = distance / speed + deceleration_time = 5 / 3.0 + 1.0 = 2.67
        self._check_result(duration=2.67)
        self._after_motion()

    def test_bezier_forward(self):
        self._before_motion()
        start = {'x': 0, 'y': 0}
        end = {'x': 3, 'y': -4}
        cp1 = {'x': 0, 'y': -2}
        cp2 = {'x': 1, 'y': -4}
        self.base.bezier_forward(start, end, cp1, cp2, {
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': Goal.MOTION_NONSTOP,
            'next_speed': 0,
            'sense_line': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Forward')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_BEZIER_FORWARD)

        # total duration = distance / speed = 5.58 / 0.6 = 9.30
        self._check_result(duration=9.30)
        self._after_motion()

    def test_bezier_reverse(self):
        self._before_motion()
        start = {'x': 0, 'y': 0}
        end = {'x': 3, 'y': 4}
        cp1 = {'x': 0, 'y': 1}
        cp2 = {'x': 3, 'y': 3}
        self.base.bezier_reverse(start, end, cp1, cp2, {
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
            'sense_line': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Reverse')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_BEZIER_REVERSE)

        # total duration = distance / speed + deceleration_time = 5.08 / 0.6 + 0.6 = 9.07
        self._check_result(duration=9.07)
        self._after_motion()

    def test_forward_dock(self):
        self._before_motion()
        marker_type = 'reflector__laser1'
        target = {'x': 1, 'y': 0, 'theta': 0}
        alignment_distance = 0
        self.base.forward_dock(marker_type, target, alignment_distance, {
            'speed': 0.2,
            'enable_sensor': False,
            'io_trigger_type': 0,
            'error_io_trigger_type': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_FORWARD_DOCK)

        self._check_result()
        self._after_motion()

    def test_reverse_dock(self):
        self._before_motion()
        marker_type = 'reflector__laser1'
        target = {'x': 1, 'y': 0, 'theta': 0}
        alignment_distance = 1.0
        self.base.reverse_dock(marker_type, target, alignment_distance, {
            'speed': 0.2,
            'enable_sensor': False,
            'io_trigger_type': 0,
            'error_io_trigger_type': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_REVERSE_DOCK)

        self._check_result()
        self._after_motion()

    def test_forward_undock(self):
        self._before_motion()
        marker_type = 'vmarker__laser2'
        target = {'x': 1, 'y': 0, 'theta': 0}
        alignment_distance = 1.0
        undock_end = {'x': 3, 'y': 0, 'theta': 0}
        self.base.forward_undock(marker_type, target, alignment_distance, undock_end, {
            'speed': 0.2,
            'enable_sensor': True,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_FORWARD_UNDOCK)

        self._check_result()
        self._after_motion()

    def test_reverse_undock(self):
        self._before_motion()
        marker_type = 'vmarker__laser2'
        target = {'x': 1, 'y': 0, 'theta': 0.1}
        alignment_distance = 1.0
        undock_end = {'x': 3, 'y': 0.2, 'theta': 0}
        self.base.reverse_undock(marker_type, target, alignment_distance, undock_end, {
            'speed': 0.2,
            'enable_sensor': True,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_REVERSE_UNDOCK)

        self._check_result()
        self._after_motion()

    def test_free_rotate_left(self):
        self._before_motion()
        end = {'x': 2.3, 'y': 5.6}
        heading = 180
        self.base.set_initial_pose(Pose2D(2.3, 5.6, 0))
        self.base.free_rotate_left(end, heading, {
            'enable_sensor': True,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Rotate Left')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_ROTATE_LEFT)

        # total duration = angle / angular speed + deceleration_time = rad(180) / 0.7 + 0.7 = 5.19
        self._check_result(duration=5.19)
        self._after_motion

    def test_free_rotate_right(self):
        self._before_motion()
        end = {'x': 12.3, 'y': -5.9}
        heading = 230
        self.base.set_initial_pose(Pose2D(12.3, -5.9, 0))
        self.base.free_rotate_right(end, heading, {
            'enable_sensor': True,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Rotate Right')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_ROTATE_RIGHT)

        # total duration = angle / angular speed + deceleration_time = rad(360 - 230) / 0.7 + 0.7 = 3.94
        self._check_result(duration=3.94)
        self._after_motion

    def test_wait_for_result_continuation_during_move_forward(self):
        self._before_motion()
        start = {'x': 0, 'y': 0}
        end = {'x': 3, 'y': -4}
        self.base.move_forward(start, end, {
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
            'sense_line': 0,
        })

        self._in_motion()

        # total duration = distance / speed + deceleration_time = 5.0 / 0.6 + 0.6 = 8.93
        self._check_result(wait=rospy.Duration(3.0), result=False)
        self._check_result(duration=5.93)
        self._after_motion()

    def test_preempt_move_forward(self):
        self._before_motion()
        start = {'x': 0, 'y': 0}
        end = {'x': 3, 'y': -4}
        self.base.move_forward(start, end, {
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
            'sense_line': 0,
        })

        self._in_motion()

        timer = rospy.Timer(rospy.Duration(2.0), lambda ev: self.base.stop(), oneshot=True)
        self._check_result(duration=2.0)
        self._after_motion()

        # distance = speed * time = 0.6 * 2.0 = 1.2
        pose = self.base.get_pose()
        distance = abs(complex(pose.x, pose.y))
        self.assertAlmostEqual(distance, 1.2, delta=0.234)

    def test_pause_and_preempt_move_reverse(self):
        self._before_motion()
        start = {'x': 0, 'y': 0}
        end = {'x': 3, 'y': 4}
        self.base.move_reverse(start, end, {
            'speed': 3.0,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
            'sense_line': 0,
        })

        self._in_motion()

        timer1 = rospy.Timer(rospy.Duration(0.5), lambda ev: self.base.pause(), oneshot=True)
        timer2 = rospy.Timer(rospy.Duration(3.0), lambda ev: self.base.stop(), oneshot=True)
        self._check_result(duration=3.0)
        self._after_motion()

        # distance = speed * time = 3.0 * 0.5 = 1.5
        pose = self.base.get_pose()
        distance = abs(complex(pose.x, pose.y))
        self.assertAlmostEqual(distance, 1.5, delta=1.17)

    def test_preempt_free_rotate_left(self):
        self._before_motion()
        end = {'x': 2.3, 'y': 5.6}
        heading = 340
        self.base.set_initial_pose(Pose2D(2.3, 5.6, 0))
        self.base.free_rotate_left(end, heading, {
            'enable_sensor': True,
        })

        self._in_motion()

        timer = rospy.Timer(rospy.Duration(2.0), lambda ev: self.base.stop(), oneshot=True)
        self._check_result(duration=2.0)
        self._after_motion()

        # distance = speed * time = 0.7 * 2.0 = 1.4rad
        pose = self.base.get_pose()
        self.assertAlmostEqual(pose.theta, 1.4, delta=0.273)

    def test_pause_and_preempt_free_rotate_right(self):
        self._before_motion()
        end = {'x': 12.3, 'y': -5.9}
        heading = 230
        self.base.set_initial_pose(Pose2D(12.3, -5.9, 0))
        self.base.free_rotate_right(end, heading, {
            'enable_sensor': True,
        })

        self._in_motion()

        timer1 = rospy.Timer(rospy.Duration(0.5), lambda ev: self.base.pause(), oneshot=True)
        timer2 = rospy.Timer(rospy.Duration(3.0), lambda ev: self.base.stop(), oneshot=True)
        self._check_result(duration=3.0)
        self._after_motion()

        # distance = speed * time = 0.7 * 0.5 = 0.35rad
        pose = self.base.get_pose()
        self.assertAlmostEqual(pose.theta, -0.35, delta=0.195)


if __name__ == '__main__':
    rospy.init_node('test_trackless_sim', anonymous=True)
    rostest.rosrun('agv05_executor', 'test_trackless_sim', TestTracklessSim, sys.argv)
