#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_msgs.msg import NavActionGoal as Goal
from django.conf import settings as django_settings
import django
import math
import networkx as nx
import os
import rospy
import rostest
import sys
import unittest

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "agv05_webserver.settings-test")
django.setup()

if True:  # prevent autopep8 messing up the import sequence
    from agv05_executor.models.map_tracker import MapTracker
    from agv05_executor.robots.tracked_sim import TrackedSim, TrackedSimBase


class TestTrackedSim(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.robot = TrackedSim()
        cls.base = cls.robot.base

    def test_init(self):
        self.assertFalse(django_settings.TRACKLESS)
        self.assertFalse(self.robot.trackless)
        self.assertIsInstance(self.robot.base, TrackedSimBase)
        self.assertIsInstance(self.robot.base.map_tracker, MapTracker)

    def test_forward(self):
        self._before_motion()
        self.base.forward({
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Forward')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_FORWARD)

        self._check_result()
        self._after_motion()

    def test_reverse(self):
        self._before_motion()
        self.base.reverse({
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Reverse')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_REVERSE)

        self._check_result()
        self._after_motion()

    def test_branch_forward_left(self):
        self._before_motion()
        self.base.branch_forward_left({
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Branch Forward Left')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_FORWARD_LEFT)

        self._check_result()
        self._after_motion()

    def test_branch_forward_right(self):
        self._before_motion()
        self.base.branch_forward_right({
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Branch Forward Right')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_FORWARD_RIGHT)

        self._check_result()
        self._after_motion()

    def test_branch_reverse_left(self):
        self._before_motion()
        self.base.branch_reverse_left({
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Branch Reverse Left')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_REVERSE_LEFT)

        self._check_result()
        self._after_motion()

    def test_branch_reverse_right(self):
        self._before_motion()
        self.base.branch_reverse_right({
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Branch Reverse Right')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_REVERSE_RIGHT)

        self._check_result()
        self._after_motion()

    def test_rotate_left(self):
        self._before_motion()
        self.base.rotate_left({
            'enable_sensor': True,
            'rotate_align_sensor': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Rotate Left')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_ROTATE_LEFT)

        self._check_result()
        self._after_motion()

    def test_rotate_right(self):
        self._before_motion()
        self.base.rotate_right({
            'enable_sensor': True,
            'rotate_align_sensor': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Rotate Right')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_ROTATE_RIGHT)

        self._check_result()
        self._after_motion()

    def test_uturn_left(self):
        self._before_motion()
        self.base.uturn_left({
            'enable_sensor': True,
            'rotate_align_sensor': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Rotate Left')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_UTURN_LEFT)

        self._check_result()
        self._after_motion()

    def test_uturn_right(self):
        self._before_motion()
        self.base.uturn_right({
            'enable_sensor': True,
            'rotate_align_sensor': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Rotate Right')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_UTURN_RIGHT)

        self._check_result()
        self._after_motion()

    def test_rotate3q_left(self):
        self._before_motion()
        self.base.rotate3q_left({
            'enable_sensor': True,
            'rotate_align_sensor': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Rotate Left')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_ROTATE3Q_LEFT)

        self._check_result()
        self._after_motion()

    def test_rotate3q_right(self):
        self._before_motion()
        self.base.rotate3q_right({
            'enable_sensor': True,
            'rotate_align_sensor': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), 'Rotate Right')
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_ROTATE3Q_RIGHT)

        self._check_result()
        self._after_motion()

    def test_search_line_left(self):
        self._before_motion()
        self.base.search_line_left({
            'enable_sensor': True,
            'rotate_align_sensor': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_SEARCH_LINE_LEFT)

        self._check_result()
        self._after_motion()

    def test_search_line_right(self):
        self._before_motion()
        self.base.search_line_right({
            'enable_sensor': True,
            'rotate_align_sensor': 0,
        })

        self._in_motion()
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_SEARCH_LINE_RIGHT)

        self._check_result()
        self._after_motion()

    def test_wait_traffic(self):
        self._before_motion()
        self.base._wait_traffic = False
        self.base.wait_traffic()

        self._in_motion(move=False)
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_WAIT_TRAFFIC)
        self.assertTrue(self.base._wait_traffic)

        self._check_result(wait=rospy.Duration(2.0), result=False)

        timer = rospy.Timer(rospy.Duration(1.0), lambda ev: self.base.stop_wait_traffic(), oneshot=True)
        self._check_result(duration=1.0)
        self._after_motion()
        self.assertFalse(self.base._wait_traffic)

    def test_free_motor(self):
        self._before_motion()
        self.base.free_motor()

        self._in_motion()
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_FREE_MOTOR)

        self.base.stop()
        self._after_motion()

    def test_manual_control(self):
        self._before_motion()
        self.base.manual_control()

        self._in_motion()
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_MANUAL_CONTROL)

        self.base.stop()
        self._after_motion()

    def test_line_calibrate(self):
        self._before_motion()
        self.base.line_calibrate()

        self._in_motion()
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_LINE_CALIBRATE)

        self._check_result()
        self._after_motion()

    def test_select_nav_profile(self):
        self._before_motion()
        self.base.select_nav_profile(1)

        self._in_motion(move=False)
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_SELECT_NAV_PROFILE1)

        self._check_result(duration=0.1, duration_tolerance=0.2)
        self._after_motion()

    def test_select_laser_profile(self):
        self._before_motion()
        self.base.select_laser_profile(4)

        self._in_motion(move=False)
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_SELECT_LASER_PROFILE1 + 4 - 1)

        self._check_result(duration=0.1, duration_tolerance=0.2)
        self._after_motion()

    def test_scan_laser_area(self):
        self._before_motion()
        self.base.scan_laser_area(17)

        self._in_motion(move=False)
        self.assertEqual(self.base.get_motion(), None)
        self.assertIsInstance(self.base.goal, Goal)
        self.assertEqual(self.base.goal.nav, Goal.NAV_SCAN_LASER_AREA1 + 17 - 1)

        self._check_result(duration=0.1, duration_tolerance=0.2)
        self._after_motion()

    def test_wait_for_result_continuation_during_forward(self):
        self._before_motion()
        self.base.forward({
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()

        self._check_result(wait=rospy.Duration(1.0), result=False)
        self._check_result(duration=0.5)
        self._after_motion()

    def test_preempt_forward(self):
        self._before_motion()
        self.base.forward({
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()

        timer = rospy.Timer(rospy.Duration(1.0), lambda ev: self.base.stop(), oneshot=True)
        self._check_result(duration=1.0)
        self._after_motion()

    def test_pause_reverse(self):
        self._before_motion()
        self.base.reverse({
            'speed': 0.6,
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()

        timer1 = rospy.Timer(rospy.Duration(1.0), lambda ev: self.base.pause(), oneshot=True)
        timer2 = rospy.Timer(rospy.Duration(2.5), lambda ev: self.base.resume(), oneshot=True)
        self._check_result(duration=3.0)
        self._after_motion()

    def test_pause_and_preempt_reverse(self):
        self._before_motion()
        self.base.reverse({
            'speed': 10.0,  # default to 0.7
            'enable_sensor': True,
            'next_motion': 0,
            'next_speed': 0,
        })

        self._in_motion()

        timer1 = rospy.Timer(rospy.Duration(1.0), lambda ev: self.base.pause(), oneshot=True)
        timer2 = rospy.Timer(rospy.Duration(2.0), lambda ev: self.base.stop(), oneshot=True)
        self._check_result(duration=2.0)
        self._after_motion()

    def test_pause_and_preempt_reverse_in_map(self):
        g = nx.DiGraph()
        g.add_node(0, {'x': 0, 'y': 0})
        g.add_node(1, {'x': 0, 'y': 1})
        g.add_node(2, {'x': -1, 'y': 0})
        g.add_edge(0, 1, {  # for TrackedSim
            'direction': 2,  # Direction.South
            'transform': 0,
            'shape': 0,  # PathShape.STRAIGHT
            'distance': 1,
            'branch': 2,  # PathBranch.NONE
        })
        g.add_edge(0, 2, {  # for TracklessSim
            'cp1': None,
            'cp2': None,
            'shape': 0,  # PathShape.STRAIGHT
            'dynamic': False,
            'tracked': True,
            'distance': 1,
            'branch': 2,  # PathBranch.NONE
        })

        try:
            self.base.map_tracker.set_map(g)
            self.base.map_tracker.location = (0, 0)
            self.base.map_tracker.prev_location = (0, 0)
            self.base.map_tracker.aware = True

            initial_pose = self.base.map_tracker.get_pose()
            self.base.set_initial_pose(initial_pose)
            self.test_pause_and_preempt_reverse()

            # distance = speed * time = 0.7 * 1.0 = 0.7m
            pose = self.base.get_pose()
            distance = abs(complex(pose.x, pose.y))
            self.assertAlmostEqual(distance, 0.7, delta=0.273)
        finally:
            # cleanup
            self.robot.base.map_tracker = type(self.robot.base.map_tracker)()
            self.robot.base.reset_pose()

    def test_preempt_rotate_left(self):
        self._before_motion()
        self.base.rotate_left({
            'enable_sensor': True,
            'rotate_align_sensor': 0,
        })

        self._in_motion()

        timer = rospy.Timer(rospy.Duration(1.0), lambda ev: self.base.stop(), oneshot=True)
        self._check_result(duration=1.0)
        self._after_motion()

    def test_pause_rotate_right(self):
        self._before_motion()
        self.base.rotate_right({
            'enable_sensor': True,
            'rotate_align_sensor': 0,
        })

        self._in_motion()

        timer1 = rospy.Timer(rospy.Duration(1.0), lambda ev: self.base.pause(), oneshot=True)
        timer2 = rospy.Timer(rospy.Duration(2.5), lambda ev: self.base.resume(), oneshot=True)
        self._check_result(duration=3.0)
        self._after_motion()

    def test_pause_and_preempt_rotate_right(self):
        self._before_motion()
        self.base.rotate_right({
            'enable_sensor': True,
            'rotate_align_sensor': 0,
        })

        self._in_motion()

        timer1 = rospy.Timer(rospy.Duration(1.0), lambda ev: self.base.pause(), oneshot=True)
        timer2 = rospy.Timer(rospy.Duration(2.0), lambda ev: self.base.stop(), oneshot=True)
        self._check_result(duration=2.0)
        self._after_motion()

    def test_pause_and_preempt_rotate_right_in_map(self):
        g = nx.DiGraph()
        g.add_node(0, {'x': 0, 'y': 0})

        try:
            self.base.map_tracker.set_map(g)
            self.base.map_tracker.location = (0, 2)
            self.base.map_tracker.prev_location = (0, 2)
            self.base.map_tracker.aware = True

            initial_pose = self.base.map_tracker.get_pose()
            self.base.set_initial_pose(initial_pose)
            self.test_pause_and_preempt_rotate_right()

            # distance = speed * time = 0.7 * 1.0 = 0.7rad
            pose = self.base.get_pose()
            distance = (pose.theta - initial_pose.theta) * [1, -1][self.robot.trackless] % (2 * math.pi)
            self.assertAlmostEqual(distance, 0.7, delta=0.273)
        finally:
            # cleanup
            self.robot.base.map_tracker = type(self.robot.base.map_tracker)()
            self.robot.base.reset_pose()

    def _before_motion(self):
        self.base.stopped = True
        self.base.paused = True
        self.robot.power.charging_status = True

    def _in_motion(self, move=True):
        self.assertFalse(self.base.stopped)
        self.assertFalse(self.base.paused)
        self.assertEqual(self.robot.power.charging_status, not move)

    def _check_result(self, wait=None, result=True, duration=1.5, duration_tolerance=0.39):
        s = rospy.get_time()
        if wait is None:
            self.assertEqual(self.base.wait_for_result(), result)
        else:
            duration = wait.to_sec()
            self.assertEqual(self.base.wait_for_result(wait), result)

        elapsed = rospy.get_time() - s
        self.assertAlmostEqual(elapsed, duration, delta=duration_tolerance)

    def _after_motion(self):
        self.assertEqual(self.base.get_motion(), None)
        self.assertEqual(self.base.goal, None)


if __name__ == '__main__':
    rospy.init_node('test_tracked_sim', anonymous=True)
    rostest.rosrun('agv05_executor', 'test_tracked_sim', TestTrackedSim, sys.argv)
