#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from django.core.management import call_command
from django.test.utils import setup_databases
from six.moves import zip
import django
import os
import rospy
import rostest
import shapely.geometry
import sys

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "agv05_webserver.settings-test")
django.setup()
setup_databases(verbosity=2, interactive=False)

if True:  # prevent autopep8 messing up the import sequence
    from agv05_executor.models.map_tracker import MapTracker, Motion
    from agv05_webserver.system.models import Direction, PathFacing, PathShape
    from agv05_webserver.system.signals import signals_suppressed

    # relative imports
    from fixtures import *
    from utils import *


def setUpModule():
    # disable rospy.Service
    global _ori_rospy_service
    _ori_rospy_service = rospy.Service
    rospy.Service = lambda *a, **kw: None

    # disable signals
    signals_suppressed[0] = True

    # load fixtures and data
    call_command('loaddata', 'default_users.json', verbosity=0, database='default')


def tearDownModule():
    # re-enable signals
    signals_suppressed[0] = False

    # re-enable rospy.Service
    rospy.Service = _ori_rospy_service


class TestMapTracker(TestCase):

    @classmethod
    def setUpClass(cls):
        super(TestMapTracker, cls).setUpClass()

        nonstop = (Motion.ZERO, True, Motion.ZERO, Motion.ZERO, Motion.ZERO)
        left = (Motion.L1, False, Motion.L1, Motion.L1, 0)
        left_final = (Motion.L1, False, 0, Motion.L1, 0)
        l2 = (Motion.L2, False, 0, Motion.L2, 0)
        l3 = (Motion.L3, False, 0, 0, 0)
        right = (Motion.R1, False, Motion.R1, 0, Motion.R1)
        right_final = (Motion.R1, False, 0, 0, Motion.R1)
        r2 = (Motion.R2, False, 0, 0, Motion.R2)
        r3 = (Motion.R3, False, 0, 0, 0)
        uturn = (Motion.L2, False, 0, Motion.L2, Motion.R2)
        invalid = (0, False, 0, 0, 0)
        cls.results = (nonstop, left, left_final, l2, l3, right, right_final, r2, r3, uturn, invalid)

        e1 = (0, 1, {
            'direction': Direction.NORTH,
            'transform': 0,
            'facing': PathFacing.FREE,
            'shape': PathShape.STRAIGHT,
        })
        e2 = (1, 2, {
            'direction': Direction.NORTH,
            'transform': 1,
            'facing': PathFacing.FREE,
            'shape': PathShape.BEND_RIGHT,
        })
        e3 = (2, 3, {
            'direction': Direction.EAST,
            'transform': 3,
            'facing': PathFacing.FREE,
            'shape': PathShape.BEND_LEFT,
        })
        e4 = (3, 4, {
            'direction': Direction.SOUTH,
            'transform': 0,
            'facing': PathFacing.FREE,
            'shape': PathShape.S_CURVE,
        })
        e5 = (4, 5, {
            'direction': Direction.WEST,
            'transform': 1,
            'facing': PathFacing.FREE,
            'shape': PathShape.BEND_RIGHT,
        })
        e6t = (5, 6, {
            'direction': Direction.NA,
            'shape': PathShape.TELEPORT,
            'teleport': 0,
        })
        e6tn = (5, 6, {
            'direction': Direction.NA,
            'shape': PathShape.TELEPORT,
            'teleport': 1,
        })
        e7t = (6, 0, {
            'direction': Direction.NA,
            'shape': PathShape.TELEPORT,
            'teleport': 2,
        })
        cls.paths = (e1, e2, e3, e4, e5, e6t, e6tn, e7t)

        stations_assoc = {
            'S1': (5, Direction.SOUTH),
            'S2': (6, Direction.NA),
            'S3': (0, Direction.EAST),
        }

        teleports = [{
            'start': 'S1',
            'end': 'S2',
            'nonStopTransition': False,
        }, {
            'start': 'S1',
            'end': 'S2',
            'nonStopTransition': True,
        }, {
            'start': 'S2',
            'end': 'S3',
            'nonStopTransition': False,
        }]

        q = (-0.51, -0.275, 0.52, 0.275)
        geoms = {
            0: q,
            q: shapely.geometry.box(*q),
        }
        geoms[q].nrn = cls.nrn = {j: 0 for j in range(7)}

        # differential (unrestricted)
        cls.mt = mt = MapTracker()
        mt.stations_assoc = stations_assoc
        mt.teleports = teleports
        mt.geoms = geoms
        mt.allowed_motions = [
            'forward', 'reverse',
            'rotate_left', 'rotate_right',
            'uturn_left', 'uturn_right',
        ]

        # trailer (no reverse and no u-turn)
        cls.mt2 = mt2 = MapTracker()
        mt2.stations_assoc = stations_assoc
        mt2.teleports = teleports
        mt2.geoms = geoms
        mt2.allowed_motions = [
            'forward', 'rotate_left', 'rotate_right',
        ]

        # no right-turn
        cls.mt3 = mt3 = MapTracker()
        mt3.stations_assoc = stations_assoc
        mt3.teleports = teleports
        mt3.geoms = geoms
        mt3.allowed_motions = [
            'forward', 'reverse', 'rotate_left', 'uturn_left',
        ]

        # no left-turn
        cls.mt4 = mt4 = MapTracker()
        mt4.stations_assoc = stations_assoc
        mt4.teleports = teleports
        mt4.geoms = geoms
        mt4.allowed_motions = [
            'forward', 'reverse', 'rotate_right', 'uturn_right',
        ]

    def ct(self, e1, e2, *args, **kwargs):
        return (
            self.mt.check_transition(e1, e2, *args, **kwargs),
            self.mt.check_transition(e1, e2, *args, zero=True, **kwargs),
            self.mt2.check_transition(e1, e2, *args, **kwargs),
            self.mt3.check_transition(e1, e2, *args, **kwargs),
            self.mt4.check_transition(e1, e2, *args, **kwargs),
        )

    def ctr(self, e1, e2, *args, **kwargs):
        return (
            self.mt.check_transition(e1, e2, *args, reverse=True, **kwargs),
            self.mt.check_transition(e1, e2, *args, reverse=True, zero=True, **kwargs),
            self.mt2.check_transition(e1, e2, *args, reverse=True, **kwargs),
            self.mt3.check_transition(e1, e2, *args, reverse=True, **kwargs),
            self.mt4.check_transition(e1, e2, *args, reverse=True, **kwargs),
        )

    def tearDown(self):
        e1, e2, e3, e4, e5, e6t, e6tn, e7t = self.paths
        e1[2]['facing'] = PathFacing.FREE
        e2[2]['facing'] = PathFacing.FREE
        e3[2]['facing'] = PathFacing.FREE
        e4[2]['facing'] = PathFacing.FREE

        nrn = self.nrn
        for j in nrn:
            nrn[j] = 0

    def test_check_transition_two_headings(self):
        nonstop, left, left_final, l2, l3, right, right_final, r2, r3, uturn, invalid = self.results
        ct, ctr = self.ct, self.ctr
        eq, er = self.expectEqual, self.expectRaises

        er(AssertionError, ct, Direction.NA, Direction.SOUTH)
        er(AssertionError, ctr, Direction.NA, Direction.SOUTH)
        eq(ct(Direction.EAST, Direction.NA), nonstop)
        eq(ctr(Direction.WEST, Direction.NA), nonstop)

        eq(ct(Direction.EAST, Direction.EAST), nonstop)
        eq(ctr(Direction.EAST, Direction.EAST), nonstop)
        eq(ct(Direction.WEST, Direction.NORTH), right_final)
        eq(ctr(Direction.WEST, Direction.SOUTH), left_final)
        eq(ct(Direction.NORTH, Direction.SOUTH), uturn)
        eq(ctr(Direction.SOUTH, Direction.NORTH), uturn)

    def test_check_transition_heading_and_path(self):
        nonstop, left, left_final, l2, l3, right, right_final, r2, r3, uturn, invalid = self.results
        e1, e2, e3, e4, e5, e6t, e6tn, e7t = self.paths
        ct, ctr = self.ct, self.ctr
        eq, er = self.expectEqual, self.expectRaises

        er(AssertionError, ct, Direction.NA, e1)
        er(AssertionError, ctr, Direction.NA, e3)
        er(AssertionError, ct, Direction.NA, e6t)

        eq(ct(Direction.NORTH, e1), nonstop)
        eq(ctr(Direction.SOUTH, e2), nonstop)
        eq(ct(Direction.SOUTH, e3), left)
        eq(ctr(Direction.WEST, e4), right)
        eq(ct(Direction.EAST, e5), uturn)
        eq(ctr(Direction.EAST, e3), uturn)

        eq(ct(Direction.WEST, e6t), left_final)
        eq(ctr(Direction.NORTH, e6t), uturn)
        eq(ct(Direction.WEST, e6tn), left)
        eq(ctr(Direction.NORTH, e6tn), uturn)
        eq(ct(Direction.WEST, e7t), nonstop)
        eq(ctr(Direction.WEST, e7t), nonstop)

        e1[2]['facing'] = PathFacing.FORWARD_UNI
        e2[2]['facing'] = PathFacing.FORWARD_UNI
        e3[2]['facing'] = PathFacing.REVERSE_UNI
        e4[2]['facing'] = PathFacing.REVERSE_UNI

        eq(ct(Direction.NORTH, e1), nonstop)
        eq(ctr(Direction.SOUTH, e2), uturn)
        eq(ct(Direction.SOUTH, e3), right)
        eq(ctr(Direction.WEST, e4), right)

    def test_check_transition_path_and_heading(self):
        nonstop, left, left_final, l2, l3, right, right_final, r2, r3, uturn, invalid = self.results
        e1, e2, e3, e4, e5, e6t, e6tn, e7t = self.paths
        ct, ctr = self.ct, self.ctr
        eq, er = self.expectEqual, self.expectRaises

        eq(ct(e1, Direction.NA), nonstop)
        eq(ctr(e3, Direction.NA), nonstop)
        eq(ctr(e7t, Direction.NA), nonstop)

        eq(ct(e1, Direction.NORTH), nonstop)
        eq(ctr(e2, Direction.WEST), nonstop)
        eq(ct(e3, Direction.WEST), left_final)
        eq(ctr(e4, Direction.EAST), right_final)
        eq(ct(e5, Direction.SOUTH), uturn)
        eq(ctr(e5, Direction.NORTH), uturn)

        er(AssertionError, ct, e6t, Direction.EAST)
        er(AssertionError, ctr, e6t, Direction.NA)
        eq(ct(e7t, Direction.WEST), uturn)
        eq(ctr(e7t, Direction.NORTH), left_final)

        e1[2]['facing'] = PathFacing.FORWARD_UNI
        e2[2]['facing'] = PathFacing.FORWARD_UNI
        e3[2]['facing'] = PathFacing.REVERSE_UNI
        e4[2]['facing'] = PathFacing.REVERSE_UNI

        eq(ct(e1, Direction.NORTH), nonstop)
        eq(ctr(e2, Direction.WEST), uturn)
        eq(ct(e3, Direction.WEST), right_final)
        eq(ctr(e4, Direction.EAST), right_final)

    def test_check_transition_two_paths(self):
        nonstop, left, left_final, l2, l3, right, right_final, r2, r3, uturn, invalid = self.results
        e1, e2, e3, e4, e5, e6t, e6tn, e7t = self.paths
        ct, ctr = self.ct, self.ctr
        eq, er = self.expectEqual, self.expectRaises

        eq(ct(e1, e2), nonstop)
        eq(ct(e2, e3), nonstop)
        eq(ct(e3, e4), uturn)
        eq(ct(e4, e5), right)
        eq(ct(e5, e1), nonstop)
        eq(ct(e5, e5), left)

        eq(ctr(e1, e2), nonstop)
        eq(ctr(e2, e3), nonstop)
        eq(ctr(e3, e4), uturn)
        eq(ctr(e4, e5), right)
        eq(ctr(e5, e1), nonstop)
        eq(ctr(e5, e5), left)

        er(AssertionError, ct, e6t, e1)
        er(AssertionError, ctr, e6t, e2)
        eq(ct(e7t, e1), left)
        eq(ctr(e7t, e1), right)
        eq(ct(e7t, e3), nonstop)
        eq(ctr(e7t, e3), uturn)

        eq(ct(e2, e6t), right_final)
        eq(ctr(e2, e6t), left_final)
        eq(ct(e4, e6t), nonstop)
        eq(ctr(e4, e6t), uturn)
        eq(ct(e2, e6tn), right)
        eq(ctr(e2, e6tn), left)
        eq(ct(e4, e6tn), nonstop)
        eq(ctr(e4, e6tn), uturn)
        eq(ct(e4, e7t), nonstop)
        eq(ctr(e4, e7t), nonstop)

        er(AssertionError, ct, e6t, e7t)
        er(AssertionError, ctr, e6t, e7t)
        eq(ct(e7t, e6t), right_final)
        eq(ctr(e7t, e6t), right_final)
        eq(ct(e7t, e6tn), right)
        eq(ctr(e7t, e6tn), right)

        e1[2]['facing'] = PathFacing.FORWARD_UNI
        e2[2]['facing'] = PathFacing.FORWARD_UNI
        e3[2]['facing'] = PathFacing.REVERSE_UNI
        e4[2]['facing'] = PathFacing.REVERSE_UNI

        eq(ct(e1, e2), nonstop)
        eq(ct(e2, e3), uturn)
        eq(ct(e3, e4), uturn)
        eq(ct(e4, e5), left)
        eq(ct(e5, e1), nonstop)
        eq(ct(e4, e1), nonstop)

        eq(ctr(e1, e2), nonstop)
        eq(ctr(e2, e3), uturn)
        eq(ctr(e3, e4), uturn)
        eq(ctr(e4, e5), right)
        eq(ctr(e5, e1), uturn)
        eq(ctr(e4, e1), nonstop)

        er(AssertionError, ct, e6t, e1)
        er(AssertionError, ctr, e6t, e2)
        eq(ct(e7t, e1), left)
        eq(ctr(e7t, e1), left)
        eq(ct(e7t, e3), uturn)
        eq(ctr(e7t, e3), uturn)

        eq(ct(e2, e6t), right_final)
        eq(ctr(e2, e6t), right_final)
        eq(ct(e4, e6t), uturn)
        eq(ctr(e4, e6t), uturn)
        eq(ct(e2, e6tn), right)
        eq(ctr(e2, e6tn), right)
        eq(ct(e4, e6tn), uturn)
        eq(ctr(e4, e6tn), uturn)
        eq(ct(e4, e7t), nonstop)
        eq(ctr(e4, e7t), nonstop)

    def test_check_transition_with_nrz(self):
        nonstop, left, left_final, l2, l3, right, right_final, r2, r3, uturn, invalid = self.results
        e1, e2, e3, e4, e5, e6t, e6tn, e7t = self.paths
        ct, ctr = self.ct, self.ctr
        eq, er = self.expectEqual, self.expectRaises

        def _nr(nr):
            return nr | (nr & 3) << 4

        nrn = self.nrn
        nrn[3] = _nr(0b1000)  # WSEN
        nrn[4] = _nr(0b0001)
        nrn[5] = _nr(0b0011)

        # two headings
        eq(ct(Direction.EAST, Direction.NA, j=3), nonstop)         # nrn[3]
        eq(ctr(Direction.WEST, Direction.NA, j=4), nonstop)        # nrn[4]
        eq(ct(Direction.WEST, Direction.NORTH, j=3), l3)           # nrn[3]
        eq(ctr(Direction.WEST, Direction.SOUTH, j=4), left_final)  # nrn[4]
        eq(ct(Direction.NORTH, Direction.SOUTH, j=5), l2)          # nrn[5]
        eq(ctr(Direction.SOUTH, Direction.NORTH, j=5), r2)         # nrn[5]

        # heading and path
        eq(ct(Direction.NORTH, e1), nonstop)
        eq(ctr(Direction.SOUTH, e2), nonstop)
        eq(ctr(Direction.WEST, e4), l3)          # nrn[3]
        eq(ct(Direction.EAST, e5), r2)           # nrn[4]
        eq(ct(Direction.WEST, e6t), left_final)  # nrn[5]
        eq(ctr(Direction.NORTH, e6tn), l2)       # nrn[5]

        # path and heading
        eq(ct(e3, Direction.WEST), r3)    # nrn[3]
        eq(ctr(e4, Direction.EAST), l3)   # nrn[4]
        eq(ct(e5, Direction.SOUTH), l2)   # nrn[5]
        eq(ctr(e5, Direction.NORTH), r2)  # nrn[5]

        # two paths
        eq(ct(e3, e4), r2)           # nrn[3]
        eq(ct(e4, e5), right)        # nrn[4]
        eq(ctr(e3, e4), l2)          # nrn[3]
        eq(ctr(e4, e5), l3)          # nrn[4]
        eq(ct(e7t, e5), r2)          # nrn[4]
        eq(ctr(e7t, e6tn), invalid)  # nrn[5]


class TestModels(TestCase):

    @classmethod
    def setUpClass(cls):
        super(TestModels, cls).setUpClass()

        # robot : differential (unrestricted)
        # robot2: trailer (no reverse and no u-turn)
        # robot3: no right-turn
        cls.robot, cls.robot2, cls.robot3 = setup_robots()

        cls.home = cls.robot.models.get_agv_home_location()
        cls.locations = [cls.robot.models.get_location_from_station(s) for s in
            ['S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'S7', 'S8', 'S9', 'T1', 'T2']]

    def tearDown(self):
        self.robot.base.set_dimension_profile(0)
        self.robot2.base.set_dimension_profile(0)
        self.robot3.base.set_dimension_profile(0)

    def test_validate_and_apply(self):
        # can only run this on robot3, which is based on the last DB state
        self.assertIs(self.robot3.models.validate(), True)
        self.assertIs(self.robot3.models.apply(), True)

    def test_from_cache(self):
        # can only run this on robot3, which is based on the last DB state
        self.assertIs(self.robot3.models.from_cache(), True)

    def test_geoms(self):
        geoms = self.robot.models.geoms
        eq, er = self.expectEqual, self.expectRaises

        q = (-0.51, -0.275, 0.52, 0.275)
        eq(geoms[0], q)
        eq(geoms[1], (q, (-0.8, -0.4, 0.2, 0.4)))
        eq(geoms[2], ((-0.61, -0.395, 0.72, 0.275 + 0.15), (-0.6, -0.42, 0.5, 0.3 + 0.15)))
        for i in range(3, 6):
            eq(geoms[i], (q, (-0.5, -0.3, -0.1 + 0.4, 0.3)))

        for i in range(6):
            if i in [1, 2]:
                continue
            eq(geoms[geoms[i]].nrn, {})

        def _nr(nr):
            return nr | (nr & 3) << 4

        nrn = geoms[geoms[1]].nrn
        eq(len(nrn), 3)
        eq(nrn[0], _nr(0b0001))
        eq(nrn[1], _nr(0b0110))
        eq(nrn[9], _nr(0b1111))

        nrn = geoms[geoms[2]].nrn
        eq(len(nrn), 2)
        eq(nrn[1], _nr(0b1111))
        eq(nrn[9], _nr(0b1111))

    def test_home_facing(self):
        home = self.home
        s1, s2, s3, s4, s5, s6, s7, s8, s9, t1, t2 = self.locations

        eq, er = self.expectEqual, self.expectRaises

        def gp(*s1s2):
            j1, j2 = next(zip(*s1s2))
            path = self.robot.models.get_path(j1, j2)
            eq(path, self.robot2.models.get_path(j1, j2))
            eq(path, self.robot3.models.get_path(j1, j2))
            return path

        eq(gp(home, s1)['facing'], PathFacing.FORWARD_UNI)
        er(KeyError, gp, home, s8)
        eq(gp(home, s9)['facing'], PathFacing.FORWARD_UNI)
        eq(gp(s1, home)['facing'], PathFacing.REVERSE_UNI)
        eq(gp(s8, home)['facing'], PathFacing.FORWARD_UNI)
        eq(gp(s9, home)['facing'], PathFacing.REVERSE_UNI)

    def test_validate_constrained_path(self):
        home = self.home
        home_na = (home[0], Direction.NA)
        home_rotated = (home[0], Direction.SOUTH)
        s1, s2, s3, s4, s5, s6, s7, s8, s9, t1, t2 = self.locations

        am, am3 = (
            self.robot.models.allowed_motions,
            self.robot3.models.allowed_motions,
        )
        eq, er = self.expectEqual, self.expectRaises

        def vcp(start, end, path):
            path = next(zip(*path))
            return (
                self.robot.models.validate_constrained_path(start, end, path),
                self.robot2.models.validate_constrained_path(start, end, path),
                self.robot3.models.validate_constrained_path(start, end, path),
            )

        def vcpr(start, end, path):
            path = next(zip(*path))
            return (
                self.robot.models.validate_constrained_path(start, end, path, reverse=True),
                self.robot2.models.validate_constrained_path(start, end, path, reverse=True),
                self.robot3.models.validate_constrained_path(start, end, path, reverse=True),
            )

        # unknown start direction
        er(AssertionError, vcp, home_na, s1, [home_na, s1])
        er(AssertionError, vcpr, s5, s4, [s5, s4])

        # ban rotation at home
        eq(vcp(home, home_rotated, [home]), (False, False, False))
        eq(vcpr(s1, home_rotated, [s1, home]), (False, False, False))
        eq(vcpr(home_rotated, home, [home]), (False, False, False))
        eq(vcp(home_rotated, s1, [home, s1]), (False, False, False))

        # same spot
        eq(vcp(home, home, [home]), (True, True, True))
        eq(vcpr(home, home_na, [home]), (True, True, True))
        eq(vcp(s4, s4, [s4]), (True, True, True))
        eq(vcpr(t2, t2, [t2]), (True, True, True))

        eq(vcp(s1, (s1[0], Direction.NORTH), [s1]), (True, False, True))
        eq(vcpr(s1, (s1[0], Direction.SOUTH), [s1, s5, s1]), (True, False, True))
        eq(vcp(t1, (t1[0], Direction.WEST), [t1]), (True, False, True))
        eq(vcpr(t1, (t1[0], Direction.NA), [t1]), (True, True, True))

        # traversals
        eq(vcp(home, s9, [home, s9]), (True, True, True))
        eq(vcp(s1, home, [s1, home]), (True, False, True))
        eq(vcp(s1, s9, [s1, home, s9]), (True, False, True))
        eq(vcp(s1, s9, [s1, s3, s5, s1, home, s9]), (True, False, False))
        eq(vcp(s3, s5, [s3, s5]), (True, True, False))
        eq(vcp((s5[0], Direction.NORTH), s3, [s5, s1, s3]), (True, True, False))  # with nonstop teleport

        for t in self.robot2.models.teleports:
            t['nonStopTransition'] = False
        try:
            reconstruct_line_graph(self.robot2.models)
            eq(vcp((s5[0], Direction.NORTH), s3, [s5, s1, s3]), (True, False, False))  # without nonstop teleport
        finally:
            for t in self.robot2.models.teleports:
                t['nonStopTransition'] = True
            reconstruct_line_graph(self.robot2.models)

        am.remove('reverse')
        am3.remove('reverse')
        try:
            reconstruct_line_graph(self.robot.models)
            reconstruct_line_graph(self.robot3.models)
            eq(vcp(s1, home, [s1, home]), (False, False, False))
            eq(vcp(s1, s9, [s1, home, s9]), (False, False, False))
            eq(vcp(s1, s9, [s1, s3, s5, s1, home, s9]), (False, False, False))
        finally:
            am.add('reverse')
            am3.add('reverse')
            reconstruct_line_graph(self.robot.models)
            reconstruct_line_graph(self.robot3.models)

        # reverse traversals
        eq(vcpr(home, s3, [home, s1, s3]), (True, True, True))
        eq(vcpr(home, s5, [home, s1, s5]), (True, False, True))
        eq(vcpr(s3, s5, [s3, s5]), (True, False, True))
        eq(vcpr((s5[0], Direction.SOUTH), s3, [s5, s1, s3]), (True, False, True))
        eq(vcpr((s8[0], Direction.EAST), home, [s8, home]), (True, True, True))
        eq(vcpr((s8[0], Direction.NORTH), home, [s8, s6, s8, home]), (True, False, True))
        eq(vcpr((s6[0], Direction.NORTH), s2, [s6, s8, home, s1, s2]), (True, False, True))

        # with payload (trolley), avoid no-rotate zones
        self.robot.base.set_dimension_profile(1)
        self.robot2.base.set_dimension_profile(1)
        self.robot3.base.set_dimension_profile(1)

        eq(vcp(s1, (s1[0], Direction.NORTH), [s1]), (True, False, False))
        eq(vcpr(s2, (s2[0], Direction.SOUTH), [s2]), (False, False, False))

        eq(vcp((s5[0], Direction.NORTH), s2, [s5, s1, s2]), (True, False, False))
        eq(vcpr((s5[0], Direction.SOUTH), s2, [s5, s1, s2]), (True, False, False))

        eq(vcp(home, (s4[0], Direction.NA), [home, s1, s2, s4]), (False, False, False))
        eq(vcpr(home, (s4[0], Direction.NA), [home, s1, s2, s4]), (True, False, False))

        X = (10, 0)
        eq(vcp((s7[0], Direction.WEST), home, [s7, X, s6, s8, home]), (False, False, False))
        eq(vcp((s7[0], Direction.WEST), s6, [s7, X, s6]), (True, True, True))
        eq(vcp((s7[0], Direction.WEST), s6, [s7, X, s8, s6]), (True, True, True))

    def test_find_shortest_constrained_path(self):
        '''
        differential (unrestricted)
        '''
        X = 99999
        home = self.home
        home_na = (home[0], Direction.NA)
        home_rotated = (home[0], Direction.SOUTH)
        s1, s2, s3, s4, s5, s6, s7, s8, s9, t1, t2 = self.locations

        fsp = self.robot.models.find_shortest_path
        fscp = self.robot.models.find_shortest_constrained_path
        vcp = self.robot.models.validate_constrained_path
        eq, er = self.expectEqual, self.expectRaises

        def fp(start, end):
            path = fscp(start, end)[0]
            return (
                path,
                path == fsp(start[0], end[0]),
                vcp(start, end, path),
            )

        def fpr(start, end):
            path = fscp(start, end, reverse=True)[0]
            return (
                path,
                path == fsp(start[0], end[0]),
                vcp(start, end, path, reverse=True),
            )

        def _fill(v1, v2):
            return [v1[i] if v == X and i < len(v1) else (v[0] if isinstance(v, tuple) else v)
                for i, v in enumerate(v2)]

        def eqT(v1, v2):
            eq(v1, (_fill(v1[0], v2), True, True))

        def eqF(v1, v2):
            eq(v1, (_fill(v1[0], v2), False, False))

        def eqFT(v1, v2):
            eq(v1, (_fill(v1[0], v2), False, True))

        # unknown start direction
        er(AssertionError, fscp, home_na, s1)
        er(AssertionError, fscp, s5, s4, reverse=True)

        # ban rotation at home
        eqF(fp(home, home_rotated), [])
        eqF(fpr(s1, home_rotated), [])
        eqF(fpr(home_rotated, home), [])
        eqF(fp(home_rotated, s1), [])

        # same spot
        eqT(fp(home, home), [home])
        eqT(fpr(home, home_na), [home])
        eqT(fp(s4, s4), [s4])
        eqT(fpr(t2, t2), [t2])

        eqT(fp(s1, (s1[0], Direction.NORTH)), [s1])
        eqT(fpr(s1, (s1[0], Direction.SOUTH)), [s1])
        eqT(fp(t1, (t1[0], Direction.WEST)), [t1])
        eqT(fpr(t1, (t1[0], Direction.NA)), [t1])

        # traversals
        eqT(fp(home, s1), [home, s1])
        eqT(fp(s1, home), [s1, home])

        eqT(fp(home, s2), [home, s1, s2])
        eqT(fp(s2, home), [s2, s1, home])

        eqT(fp(home, s3), [home, s1, s3])
        eqT(fp(s3, home), [s3, s5, s1, home])

        eqT(fp(home, s4), [home, s1, s2, s4])
        eqFT(fp(s4, home), [s4, s3, s5, s1, home])

        eqT(fp(home, s5), [home, s1, s5])
        eqT(fp((s5[0], Direction.SOUTH), home), [s5, s1, home])

        eqFT(fp(home, s7), [home, s1, s3, X, X, X, s6, X, s7])
        eqT(fp((s7[0], Direction.WEST), home), [s7, X, s8, home])

        eqFT(fp(home, s8), [home, s1, s3, X, X, X, s6, s8])
        eqT(fp((s8[0], Direction.NORTH), home), [s8, home])

        eqT(fp(s1, s9), [s1, home, s9])
        eqT(fp((s9[0], Direction.WEST), s1), [s9, home, s1])

        eqT(fp(s2, s6), [s2, s4, s3, X, X, X, s6])
        eqT(fp((s6[0], Direction.NORTH), s2), [s6, s8, home, s1, s2])

        eqT(fp(s3, s4), [s3, s5, s1, s2, s4])
        eqT(fp(s4, s3), [s4, s3])

        eqT(fp(s3, s5), [s3, s5])
        eqT(fp((s5[0], Direction.SOUTH), s3), [s5, s3])

        eqT(fp(s3, s7), [s3, X, X, X, s6, X, s7])
        eqT(fp((s7[0], Direction.WEST), s3), [s7, X, s8, home, s1, s3])

        eqT(fp(t1, t2), [t1, t2])
        eqT(fp(t2, t1), [t2, t1])

        # reverse traversals
        eqT(fpr(home, s1), [home, s1])
        eqT(fpr(s1, home), [s1, home])

        eqT(fpr(home, s2), [home, s1, s2])
        eqT(fpr(s2, home), [s2, s1, home])

        eqT(fpr(home, s3), [home, s1, s3])
        eqT(fpr(s3, home), [s3, s5, s1, home])

        eqT(fpr(home, s4), [home, s1, s2, s4])
        eqT(fpr(s4, home), [s4, s2, s1, home])

        eqT(fpr(home, s5), [home, s1, s5])
        eqT(fpr((s5[0], Direction.SOUTH), home), [s5, s1, home])

        eqT(fpr(home, s7), [home, s1, s5, X, X, X, X, s6, X, s7])
        eqT(fpr((s7[0], Direction.WEST), home), [s7, X, s8, home])

        eqT(fpr(home, s8), [home, s1, s5, X, X, X, X, s6, s8])
        eqT(fpr((s8[0], Direction.NORTH), home), [s8, home])

        eqT(fpr(s1, s9), [s1, home, s9])
        eqT(fpr((s9[0], Direction.WEST), s1), [s9, home, s1])

        eqT(fpr(s2, s6), [s2, s4, s3, X, X, X, s6])
        eqT(fpr((s6[0], Direction.NORTH), s2), [s6, s8, home, s1, s2])

        eqT(fpr(s3, s4), [s3, s5, s1, s2, s4])
        eqT(fpr(s4, s3), [s4, s3])

        eqT(fpr(s3, s5), [s3, s5])
        eqT(fpr((s5[0], Direction.SOUTH), s3), [s5, s3])

        eqT(fpr(s3, s7), [s3, X, X, X, s6, X, s7])
        eqT(fpr((s7[0], Direction.WEST), s3), [s7, X, s8, home, s1, s3])

        eqT(fpr(t1, t2), [t1, t2])
        eqT(fpr(t2, t1), [t2, t1])

        # with payload (trolley), avoid no-rotate zones
        self.robot.base.set_dimension_profile(1)

        eqT(fp(s1, (s1[0], Direction.NORTH)), [s1])
        eqT(fpr(s1, (s1[0], Direction.SOUTH)), [s1])

        eqT(fp(home, s2), [home, s1, s2])
        eqT(fp(s2, home), [s2, s1, home])

        eqFT(fp(home, s7), [home, s1, s3, X, X, X, s6, s8, X, s7])
        eqT(fp((s7[0], Direction.WEST), home), [s7, X, s8, home])

        eqT(fpr(home, s2), [home, s1, s2])
        eqT(fpr(s2, home), [s2, s1, home])

        eqT(fpr(home, s5), [home, s1, s5])
        eqT(fpr((s5[0], Direction.SOUTH), home), [s5, s1, home])

    def test_find_shortest_constrained_path2(self, nonstop_teleports=True):
        '''
        trailer (no reverse and no u-turn)
        '''
        X = 99999
        home = self.home
        home_na = (home[0], Direction.NA)
        home_rotated = (home[0], Direction.SOUTH)
        s1, s2, s3, s4, s5, s6, s7, s8, s9, t1, t2 = self.locations

        fsp = self.robot2.models.find_shortest_path
        fscp = self.robot2.models.find_shortest_constrained_path
        vcp = self.robot2.models.validate_constrained_path
        eq, er = self.expectEqual, self.expectRaises

        def fp(start, end):
            path = fscp(start, end)[0]
            return (
                path,
                path == fsp(start[0], end[0]),
                vcp(start, end, path),
            )

        def fpr(start, end):
            path = fscp(start, end, reverse=True)[0]
            return (
                path,
                path == fsp(start[0], end[0]),
                vcp(start, end, path, reverse=True),
            )

        def _fill(v1, v2):
            return [v1[i] if v == X and i < len(v1) else (v[0] if isinstance(v, tuple) else v)
                for i, v in enumerate(v2)]

        def eqT(v1, v2):
            eq(v1, (_fill(v1[0], v2), True, True))

        def eqF(v1, v2):
            eq(v1, (_fill(v1[0], v2), False, False))

        def eqFT(v1, v2):
            eq(v1, (_fill(v1[0], v2), False, True))

        # unknown start direction
        er(AssertionError, fscp, home_na, s1)
        er(AssertionError, fscp, s5, s4, reverse=True)

        # ban rotation at home
        eqF(fp(home, home_rotated), [])
        eqF(fpr(s1, home_rotated), [])
        eqF(fpr(home_rotated, home), [])
        eqF(fp(home_rotated, s1), [])

        # same spot
        eqT(fp(home, home), [home])
        eqT(fpr(home, home_na), [home])
        eqT(fp(s4, s4), [s4])
        eqT(fpr(t2, t2), [t2])

        eqFT(fp(s1, (s1[0], Direction.NORTH)), [s1, s3, s5, s1])
        eqF(fpr(s1, (s1[0], Direction.SOUTH)), [])
        eqFT(fp(t1, (t1[0], Direction.WEST)), [t1, X, X, X, X, X, X, t1])
        eqT(fpr(t1, (t1[0], Direction.NA)), [t1])

        # traversals
        eqT(fp(home, s1), [home, s1])
        eqFT(fp(s1, home), [s1, s3, X, X, X, s6, s8, home])

        eqF(fp(home, s2), [])
        eqFT(fp(s2, home), [s2, s4, s3, X, X, X, s6, s8, home])

        eqT(fp(home, s3), [home, s1, s3])
        eqFT(fp(s3, home), [s3, X, X, X, s6, s8, home])

        eqFT(fp(home, s4), [home, s1, s3, s4])
        eqFT(fp(s4, home), [s4, s3, X, X, X, s6, s8, home])

        eqT(fp(home, s5), [home, s1, s5])
        if nonstop_teleports:
            eqFT(fp((s5[0], Direction.SOUTH), home), [s5, s3, s4, s3, X, X, X, s6, s8, home])
        else:
            eqF(fp((s5[0], Direction.SOUTH), home), [])

        eqFT(fp(home, s7), [home, s1, s3, X, X, X, s6, X, s7])
        eqFT(fp((s7[0], Direction.WEST), home), [s7, X, s6, s8, home])

        eqFT(fp(home, s8), [home, s1, s3, X, X, X, s6, s8])
        eqT(fp((s8[0], Direction.NORTH), home), [s8, home])

        eqFT(fp(s1, s9), [s1, s3, X, X, X, s6, s8, home, s9])
        eqF(fp((s9[0], Direction.WEST), s1), [])

        eqT(fp(s2, s6), [s2, s4, s3, X, X, X, s6])
        eqF(fp((s6[0], Direction.NORTH), s2), [])

        eqFT(fp(s3, s4), [s3, s4])
        if nonstop_teleports:
            eqFT(fp(s4, s3), [s4, s3, s5, s1, s3])
        else:
            eqFT(fp(s4, s3), [s4, s3, X, X, X, s6, s8, home, s1, s3])

        eqT(fp(s3, s5), [s3, s5])
        if nonstop_teleports:
            eqFT(fp((s5[0], Direction.SOUTH), s3), [s5, s3, s4, s3, s5, s1, s3])
        else:
            eqF(fp((s5[0], Direction.SOUTH), s3), [])

        eqT(fp(s3, s7), [s3, X, X, X, s6, X, s7])
        eqFT(fp((s7[0], Direction.WEST), s3), [s7, X, s6, s8, home, s1, s3])

        eqFT(fp(t1, t2), [t1, t2, X, X, X, X, t2])
        eqF(fp(t2, t1), [])

        # reverse traversals
        eqT(fpr(home, s1), [home, s1])
        eqF(fpr(s1, home), [])

        eqF(fpr(home, s2), [])
        eqF(fpr(s2, home), [])

        eqT(fpr(home, s3), [home, s1, s3])
        eqF(fpr(s3, home), [])

        eqFT(fpr(home, s4), [home, s1, s3, s4])
        eqF(fpr(s4, home), [])

        eqF(fpr(home, s5), [])
        eqF(fpr((s5[0], Direction.SOUTH), home), [])

        eqF(fpr(home, s7), [])
        eqF(fpr((s7[0], Direction.WEST), home), [])

        eqF(fpr(home, s8), [])
        eqT(fpr((s8[0], Direction.NORTH), home), [s8, home])

        eqF(fpr(s1, s9), [])
        eqF(fpr((s9[0], Direction.WEST), s1), [])

        eqF(fpr(s2, s6), [])
        eqF(fpr((s6[0], Direction.NORTH), s2), [])

        eqFT(fpr(s3, s4), [s3, s4])
        eqF(fpr(s4, s3), [])

        eqF(fpr(s3, s5), [])
        eqF(fpr((s5[0], Direction.SOUTH), s3), [])

        eqF(fpr(s3, s7), [])
        eqF(fpr((s7[0], Direction.WEST), s3), [])

        eqF(fpr(t1, t2), [])
        eqF(fpr(t2, t1), [])

        # with payload (trolley), avoid no-rotate zones
        self.robot2.base.set_dimension_profile(1)

        eqFT(fp(s1, (s1[0], Direction.NORTH)), [s1, s3, s5, s1])
        eqF(fpr(s1, (s1[0], Direction.SOUTH)), [])

        eqF(fp(home, s2), [])
        if nonstop_teleports:
            eqFT(fp(s2, home), [s2, s1, s5, s3, s4, s3, X, X, X, s6, s8, home])
        else:
            eqF(fp(s2, home), [])

        eqFT(fp(home, s7), [home, s1, s3, X, X, X, s6, s8, X, s7])
        if nonstop_teleports:
            eqFT(fp((s7[0], Direction.WEST), home), [s7, X, s8, s6, X, X, X, s3, s4, s3, X, X, X, s6, s8, home])
        else:
            eqF(fp((s7[0], Direction.WEST), home), [])

        eqF(fpr(home, s2), [])
        eqF(fpr(s2, home), [])

        eqF(fpr(home, s5), [])
        eqF(fpr((s5[0], Direction.SOUTH), home), [])

    def test_find_shortest_constrained_path2_without_nonstop_teleports(self):
        for t in self.robot2.models.teleports:
            t['nonStopTransition'] = False
        try:
            reconstruct_line_graph(self.robot2.models)
            self.test_find_shortest_constrained_path2(nonstop_teleports=False)
        finally:
            for t in self.robot2.models.teleports:
                t['nonStopTransition'] = True
            reconstruct_line_graph(self.robot2.models)

    def test_find_shortest_constrained_path3(self):
        '''
        no right-turn
        '''
        X = 99999
        home = self.home
        home_na = (home[0], Direction.NA)
        home_rotated = (home[0], Direction.SOUTH)
        s1, s2, s3, s4, s5, s6, s7, s8, s9, t1, t2 = self.locations

        fsp = self.robot3.models.find_shortest_path
        fscp = self.robot3.models.find_shortest_constrained_path
        vcp = self.robot3.models.validate_constrained_path
        eq, er = self.expectEqual, self.expectRaises

        def fp(start, end):
            path = fscp(start, end)[0]
            return (
                path,
                path == fsp(start[0], end[0]),
                vcp(start, end, path),
            )

        def fpr(start, end):
            path = fscp(start, end, reverse=True)[0]
            return (
                path,
                path == fsp(start[0], end[0]),
                vcp(start, end, path, reverse=True),
            )

        def _fill(v1, v2):
            return [v1[i] if v == X and i < len(v1) else (v[0] if isinstance(v, tuple) else v)
                for i, v in enumerate(v2)]

        def eqT(v1, v2):
            eq(v1, (_fill(v1[0], v2), True, True))

        def eqF(v1, v2):
            eq(v1, (_fill(v1[0], v2), False, False))

        def eqFT(v1, v2):
            eq(v1, (_fill(v1[0], v2), False, True))

        # unknown start direction
        er(AssertionError, fscp, home_na, s1)
        er(AssertionError, fscp, s5, s4, reverse=True)

        # ban rotation at home
        eqF(fp(home, home_rotated), [])
        eqF(fpr(s1, home_rotated), [])
        eqF(fpr(home_rotated, home), [])
        eqF(fp(home_rotated, s1), [])

        # same spot
        eqT(fp(home, home), [home])
        eqT(fpr(home, home_na), [home])
        eqT(fp(s4, s4), [s4])
        eqT(fpr(t2, t2), [t2])

        eqT(fp(s1, (s1[0], Direction.NORTH)), [s1])
        eqFT(fpr(s1, (s1[0], Direction.SOUTH)), [s1, s5, s1])
        eqT(fp(t1, (t1[0], Direction.WEST)), [t1])
        eqT(fpr(t1, (t1[0], Direction.NA)), [t1])

        # traversals
        eqT(fp(home, s1), [home, s1])
        eqT(fp(s1, home), [s1, home])

        eqT(fp(home, s2), [home, s1, s2])
        eqT(fp(s2, home), [s2, s1, home])

        eqT(fp(home, s3), [home, s1, s3])
        eqFT(fp(s3, home), [s3, s4, s3, X, X, X, s6, s8, s6, X, s8, home])

        eqFT(fp(home, s4), [home, s1, s3, s4])
        eqFT(fp(s4, home), [s4, s3, X, X, X, s6, s8, s6, X, s8, home])

        eqFT(fp(home, s5), [home, s1, s2, s1, s5])
        eqFT(fp((s5[0], Direction.SOUTH), home), [s5, X, X, X, X, s6, s8, s6, X, s8, home])

        eqFT(fp(home, s7), [home, s1, s2, s1, s5, X, X, X, X, s6, s8, s6, X, s7])
        eqT(fp((s7[0], Direction.WEST), home), [s7, X, s8, home])

        eqFT(fp(home, s8), [home, s1, s2, s1, s5, X, X, X, X, s6, s8])
        eqFT(fp((s8[0], Direction.NORTH), home), [s8, s6, X, s8, home])

        eqT(fp(s1, s9), [s1, home, s9])
        eqT(fp((s9[0], Direction.WEST), s1), [s9, home, s1])

        eqFT(fp(s2, s6), [s2, s1, s5, X, X, X, X, s6])
        eqFT(fp((s6[0], Direction.NORTH), s2), [s6, s8, s6, X, s8, home, s1, s2])

        eqFT(fp(s3, s4), [s3, s4])
        eqT(fp(s4, s3), [s4, s3])

        eqFT(fp(s3, s5), [s3, s4, s3, s5])
        eqFT(fp((s5[0], Direction.SOUTH), s3), [s5, X, X, X, X, s6, s8, s6, X, s8, home, s1, s3])

        eqFT(fp(s3, s7), [s3, s4, s3, X, X, X, s6, s8, s6, X, s7])
        eqT(fp((s7[0], Direction.WEST), s3), [s7, X, s8, home, s1, s3])

        eqFT(fp(t1, t2), [t1, t2, X, t2])
        eqFT(fp(t2, t1), [t2, X, t2, t1])

        # reverse traversals
        eqT(fpr(home, s1), [home, s1])
        eqT(fpr(s1, home), [s1, home])

        eqT(fpr(home, s2), [home, s1, s2])
        eqT(fpr(s2, home), [s2, s1, home])

        eqT(fpr(home, s3), [home, s1, s3])
        eqFT(fpr(s3, home), [s3, X, X, s5, s1, home])

        eqFT(fpr(home, s4), [home, s1, s3, s4])
        eqT(fpr(s4, home), [s4, s2, s1, home])

        eqT(fpr(home, s5), [home, s1, s5])
        eqT(fpr((s5[0], Direction.SOUTH), home), [s5, s1, home])

        eqFT(fpr(home, s7), [home, s1, s5, X, X, X, X, s6, s8, s6, X, s7])
        eqT(fpr((s7[0], Direction.WEST), home), [s7, X, s8, home])

        eqT(fpr(home, s8), [home, s1, s5, X, X, X, X, s6, s8])
        eqFT(fpr((s8[0], Direction.NORTH), home), [s8, s6, s8, home])

        eqT(fpr(s1, s9), [s1, home, s9])
        eqT(fpr((s9[0], Direction.WEST), s1), [s9, home, s1])

        eqFT(fpr(s2, s6), [s2, s1, s5, X, X, X, X, s6])
        eqT(fpr((s6[0], Direction.NORTH), s2), [s6, s8, home, s1, s2])

        eqFT(fpr(s3, s4), [s3, s4])
        eqFT(fpr(s4, s3), [s4, s2, s1, s3])

        eqT(fpr(s3, s5), [s3, s5])
        eqFT(fpr((s5[0], Direction.SOUTH), s3), [s5, s1, s3])

        eqFT(fpr(s3, s7), [s3, X, X, X, s6, s8, s6, X, s7])
        eqT(fpr((s7[0], Direction.WEST), s3), [s7, X, s8, home, s1, s3])

        eqT(fpr(t1, t2), [t1, t2])
        eqT(fpr(t2, t1), [t2, t1])

        # with payload (trolley), avoid no-rotate zones
        self.robot3.base.set_dimension_profile(1)

        eqFT(fp(s1, (s1[0], Direction.NORTH)), [s1, s2, s1, s5, s1])
        eqFT(fpr(s1, (s1[0], Direction.SOUTH)), [s1, s3, X, X, s5, s1])

        eqT(fp(home, s2), [home, s1, s2])
        eqT(fp(s2, home), [s2, s1, home])

        eqF(fp(home, s7), [])
        eqT(fp((s7[0], Direction.WEST), home), [s7, X, s8, home])

        eqF(fpr(home, s2), [])
        eqF(fpr(s2, home), [])

        eqFT(fpr(home, s5), [home, s1, s3, s5])
        eqT(fpr((s5[0], Direction.SOUTH), home), [s5, s1, home])


if __name__ == '__main__':
    rospy.init_node('test_models', anonymous=True)
    rostest.rosrun('agv05_executor', 'test_models', __name__, sys.argv)
