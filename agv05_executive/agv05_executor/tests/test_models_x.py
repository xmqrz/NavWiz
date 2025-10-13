#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from django.core.management import call_command
from django.test.utils import setup_databases
from six.moves import zip
import django
import networkx as nx
import os
import rospy
import rostest
import shapely.geometry
import sys

os.environ['TRACKLESS'] = '1'
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "agv05_webserver.settings-test")
django.setup()
setup_databases(verbosity=2, interactive=False)

if True:  # prevent autopep8 messing up the import sequence
    from agv05_executor.models.map_tracker_x import MapTrackerX, Motion
    from agv05_webserver.system.models import PathFacing
    from agv05_webserver.system.signals import signals_suppressed
    from agv05_webserver.systemx.models import Heading, PathShape

    # relative imports
    from fixtures_x import *
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


class TestMapTrackerX(TestCase):

    @classmethod
    def setUpClass(cls):
        super(TestMapTrackerX, cls).setUpClass()

        nonstop = (Motion.ZERO, True, Motion.ZERO, Motion.ZERO, Motion.ZERO)  # (355, 5)
        left_v = (Motion.L0, False, Motion.L0, Motion.L0, 0)                  # [5, 45)
        left_v_final = (Motion.L0, False, 0, Motion.L0, 0)                    # [5, 45)
        left = (Motion.L1, False, Motion.L1, Motion.L1, 0)                    # [45, 95]
        left_final = (Motion.L1, False, 0, Motion.L1, 0)                      # [45, 95]
        left_u = (Motion.L1, False, 0, Motion.L1, 0)                          # (95, 135]
        l2 = (Motion.L2, False, 0, Motion.L2, 0)                              # (135, 185)
        l3 = (Motion.L3, False, 0, 0, 0)                                      # [185, 355]
        right_v = (Motion.R0, False, Motion.R0, 0, Motion.R0)                 # (315, 355]
        right_v_final = (Motion.R0, False, 0, 0, Motion.R0)                   # (315, 355]
        right = (Motion.R1, False, Motion.R1, 0, Motion.R1)                   # [265, 315]
        right_final = (Motion.R1, False, 0, 0, Motion.R1)                     # [265, 315]
        right_u = (Motion.R1, False, 0, 0, Motion.R1)                         # [225, 265)
        r2 = (Motion.R2, False, 0, 0, Motion.R2)                              # (175, 225)
        r3 = (Motion.R3, False, 0, 0, 0)                                      # [5, 175]
        uturn = (Motion.L2, False, 0, Motion.L2, Motion.R2)                   # (175, 185)
        invalid = (0, False, 0, 0, 0)
        cls.results = (nonstop, left_v, left_v_final, left, left_final, left_u, l2, l3,
            right_v, right_v_final, right, right_final, right_u, r2, r3, uturn, invalid)

        graph = nx.DiGraph()
        graph.add_node(0, x=0.0, y=0.0)
        graph.add_node(1, x=-2.0, y=3.0)
        graph.add_node(2, x=-3.0, y=-4.0)
        graph.add_node(3, x=2.0, y=-2.0)
        graph.add_node(4, x=4.0, y=0.0)
        graph.add_node(5, x=6.0, y=1.2)

        e1 = (0, 1, {  # F: 123.7 deg, R: 303.7 deg
            'facing': PathFacing.FREE,
            'shape': PathShape.STRAIGHT,
        })
        e2 = (1, 2, {  # F: 213.7 deg -> 270.0 deg, R: 33.7 deg -> 90.0 deg
            'cp1': {'x': -3.2, 'y': 2.2},
            'cp2': {'x': -3.0, 'y': -2.0},
            'facing': PathFacing.FREE,
            'shape': PathShape.BEZIER,
        })
        e3 = (2, 3, {  # F: 45.0 deg, R: 225.0 deg
            'cp1': {'x': -1.0, 'y': -2.0},
            'cp2': {'x': 0.0, 'y': -4.0},
            'facing': PathFacing.FREE,
            'shape': PathShape.BEZIER,
        })
        e4 = (3, 4, {  # F: 45.0 deg, R: 225.0 deg
            'facing': PathFacing.FREE,
            'shape': PathShape.STRAIGHT,
        })
        e5 = (4, 5, {  # F: 31.0 deg, R: 211.0 deg
            'facing': PathFacing.FREE,
            'shape': PathShape.STRAIGHT,
        })
        e6t = (5, 6, {
            'shape': PathShape.TELEPORT,
            'teleport': 0,
        })
        e6tn = (5, 6, {
            'shape': PathShape.TELEPORT,
            'teleport': 1,
        })
        e7t = (6, 0, {
            'shape': PathShape.TELEPORT,
            'teleport': 2,
        })
        cls.paths = (e1, e2, e3, e4, e5, e6t, e6tn, e7t)

        stations_assoc = {
            'S1': (5, 50.0),
            'S2': (6, Heading.NA),
            'S3': (0, 330.0),
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
        cls.mt = mt = MapTrackerX()
        mt.graph = graph
        mt.stations_assoc = stations_assoc
        mt.teleports = teleports
        mt.geoms = geoms
        mt.allowed_motions = [
            'forward', 'reverse',
            'rotate_left', 'rotate_right',
            'uturn_left', 'uturn_right',
        ]

        # trailer (no reverse and no u-turn)
        cls.mt2 = mt2 = MapTrackerX()
        mt2.graph = graph
        mt2.stations_assoc = stations_assoc
        mt2.teleports = teleports
        mt2.geoms = geoms
        mt2.allowed_motions = [
            'forward', 'rotate_left', 'rotate_right',
        ]

        # no right-turn
        cls.mt3 = mt3 = MapTrackerX()
        mt3.graph = graph
        mt3.stations_assoc = stations_assoc
        mt3.teleports = teleports
        mt3.geoms = geoms
        mt3.allowed_motions = [
            'forward', 'reverse', 'rotate_left', 'uturn_left',
        ]

        # no left-turn
        cls.mt4 = mt4 = MapTrackerX()
        mt4.graph = graph
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

    def test_is_heading_equal(self):
        def he(h1, h2, truth):
            self.expectIs(self.mt.is_heading_equal(h1, h2), truth)

        he(0.0, 5.0, False)
        he(0.0, 4.999, True)
        he(45.0, 41.0, True)
        he(45.0, 50.0, False)
        he(179.0, 180.0, True)
        he(174.999, 180.0, False)
        he(174.999, 179.998, True)
        he(260.0, 10.0, False)
        he(330.0, 260.0, False)
        he(335.001, 340.0, True)

        he(340.0, -20.0, True)
        he(340.0, -15.001, True)
        he(340.0, -15.0, False)
        he(340.101, -14.9, True)
        he(355.001, 0.0, True)
        he(355.0, 0.0, False)
        he(355.001, 0.001, False)
        he(0.0, 355.001, True)
        he(0.0, 355.0, False)
        he(0.199, 355.2, True)

    def test_check_transition_two_headings(self):
        (nonstop, left_v, left_v_final, left, left_final, left_u, l2, l3,
            right_v, right_v_final, right, right_final, right_u, r2, r3,
            uturn, invalid) = self.results
        ct, ctr = self.ct, self.ctr
        eq, er = self.expectEqual, self.expectRaises

        er(AssertionError, ct, Heading.NA, 90.0)
        er(AssertionError, ctr, Heading.NA, 250.0)
        eq(ct(310.0, Heading.NA), nonstop)
        eq(ctr(140.0, Heading.NA), nonstop)

        eq(ct(150.0, 150.0), nonstop)  # 0.0
        eq(ctr(356.0, 0.9), nonstop)   # 4.9
        eq(ct(356.0, 1.0), left_v_final)    # 5.0
        eq(ctr(356.0, 40.9), left_v_final)  # 44.9
        eq(ct(356.0, 41.0), left_final)     # 45.0
        eq(ctr(336.0, 40.0), left_final)    # 64.0
        eq(ct(306.0, 40.0), left_final)     # 94.0
        eq(ctr(125.0, 300.0), l2)           # 175.0
        eq(ct(124.9, 300.0), uturn)  # 175.1
        eq(ctr(180.0, 0.0), uturn)   # 180.0
        eq(ct(40.0, 224.9), uturn)   # 184.9
        eq(ctr(40.0, 225.0), r2)             # 185.0
        eq(ct(170.0, 0.0), r2)               # 190.0
        eq(ctr(0.0, 269.0), right_final)     # 269.0
        eq(ct(30.0, 330.0), right_final)     # 300.0
        eq(ctr(150.0, 105.0), right_final)   # 315.0
        eq(ct(150.0, 105.1), right_v_final)  # 315.1
        eq(ctr(30.0, 25.0), right_v_final)   # 355.0
        eq(ct(0.9, 356.0), nonstop)   # 355.1
        eq(ctr(60.0, 59.9), nonstop)  # 359.9

    def test_check_transition_heading_and_path(self):
        (nonstop, left_v, left_v_final, left, left_final, left_u, l2, l3,
            right_v, right_v_final, right, right_final, right_u, r2, r3,
            uturn, invalid) = self.results
        e1, e2, e3, e4, e5, e6t, e6tn, e7t = self.paths
        ct, ctr = self.ct, self.ctr
        eq, er = self.expectEqual, self.expectRaises

        er(AssertionError, ct, Heading.NA, e1)
        er(AssertionError, ctr, Heading.NA, e3)
        er(AssertionError, ct, Heading.NA, e6t)

        eq(ct(120.0, e1), nonstop)  # 3.7
        eq(ctr(35.0, e2), nonstop)  # 358.7
        eq(ct(40.0, e3), left_v)    # 5.0
        eq(ctr(315.0, e4), right)   # 270.0
        eq(ct(296.0, e5), left)     # 95.0
        eq(ctr(315.0, e1), right_v)  # 348.7
        eq(ct(305.0, e3), left_u)   # 100.0
        eq(ctr(250.0, e2), l2)      # 143.7
        eq(ct(10.0, e2), r2)        # 203.7
        eq(ctr(215.0, e2), uturn)   # 178.7

        eq(ct(140.0, e6t), right_final)  # 270.0
        eq(ctr(230.0, e6t), uturn)       # 180.0
        eq(ct(140.0, e6tn), right)       # 270.0
        eq(ctr(230.0, e6tn), uturn)      # 180.0
        eq(ct(0.0, e7t), nonstop)
        eq(ctr(10.0, e7t), nonstop)

        e1[2]['facing'] = PathFacing.FORWARD_UNI
        e2[2]['facing'] = PathFacing.FORWARD_UNI
        e3[2]['facing'] = PathFacing.REVERSE_UNI
        e4[2]['facing'] = PathFacing.REVERSE_UNI

        eq(ct(120.0, e1), nonstop)   # 3.7
        eq(ctr(35.0, e2), uturn)     # 178.7
        eq(ct(40.0, e3), r2)         # 185.0
        eq(ctr(315.0, e4), right)    # 270.0
        eq(ct(305.0, e3), right)     # 280.0
        eq(ctr(250.0, e2), right_v)  # 323.7
        eq(ct(10.0, e2), r2)         # 203.7
        eq(ctr(215.0, e2), nonstop)  # 358.7

    def test_check_transition_path_and_heading(self):
        (nonstop, left_v, left_v_final, left, left_final, left_u, l2, l3,
            right_v, right_v_final, right, right_final, right_u, r2, r3,
            uturn, invalid) = self.results
        e1, e2, e3, e4, e5, e6t, e6tn, e7t = self.paths
        ct, ctr = self.ct, self.ctr
        eq, er = self.expectEqual, self.expectRaises

        eq(ct(e1, Heading.NA), nonstop)
        eq(ctr(e3, Heading.NA), nonstop)
        eq(ctr(e7t, Heading.NA), nonstop)

        eq(ct(e1, 120.0), nonstop)       # 356.3
        eq(ctr(e2, 94.9), nonstop)       # 4.9
        eq(ct(e3, 80.0), left_v_final)   # 35.0
        eq(ctr(e4, 0.0), left_u)         # 135.0
        eq(ct(e5, 216.0), r2)            # 185.0
        eq(ctr(e2, 270.0), uturn)        # 180.0
        eq(ct(e4, 40.0), right_v_final)  # 355.0
        eq(ctr(e1, 240.0), right_final)  # 296.3

        er(AssertionError, ct, e6t, 270.0)
        er(AssertionError, ctr, e6t, Heading.NA)
        eq(ct(e7t, 50.0), left_final)  # 80.0
        eq(ctr(e7t, 330.0), nonstop)   # 0.0

        e1[2]['facing'] = PathFacing.FORWARD_UNI
        e2[2]['facing'] = PathFacing.FORWARD_UNI
        e3[2]['facing'] = PathFacing.REVERSE_UNI
        e4[2]['facing'] = PathFacing.REVERSE_UNI

        eq(ct(e1, 120.0), nonstop)  # 356.3
        eq(ctr(e2, 94.9), uturn)    # 184.9
        eq(ct(e3, 80.0), r2)        # 215.0
        eq(ctr(e4, 0.0), left_u)    # 135.0
        eq(ct(e4, 40.0), l2)        # 175.0
        eq(ctr(e1, 240.0), left_u)  # 116.3

    def test_check_transition_two_paths(self):
        (nonstop, left_v, left_v_final, left, left_final, left_u, l2, l3,
            right_v, right_v_final, right, right_final, right_u, r2, r3,
            uturn, invalid) = self.results
        e1, e2, e3, e4, e5, e6t, e6tn, e7t = self.paths
        ct, ctr = self.ct, self.ctr
        eq, er = self.expectEqual, self.expectRaises

        eq(ct(e1, e2), left)     # 90.0
        eq(ct(e2, e3), left_u)   # 135.0
        eq(ct(e3, e4), nonstop)  # 0.0
        eq(ct(e4, e5), right_v)  # 346.0
        eq(ct(e5, e1), left)     # 92.7
        eq(ct(e5, e4), left_v)   # 14.0
        eq(ct(e2, e2), right)    # 303.7
        eq(ct(e3, e1), left)     # 78.7
        eq(ct(e3, e2), l2)       # 168.7

        eq(ctr(e1, e2), left)     # 90.0
        eq(ctr(e2, e3), left_u)   # 135.0
        eq(ctr(e3, e4), nonstop)  # 0.0
        eq(ctr(e4, e5), right_v)  # 346.0
        eq(ctr(e5, e1), left)     # 92.7
        eq(ctr(e5, e4), left_v)   # 14.0
        eq(ctr(e2, e2), right)    # 303.7
        eq(ctr(e3, e1), left)     # 78.7
        eq(ctr(e3, e2), l2)       # 168.7

        er(AssertionError, ct, e6t, e1)
        er(AssertionError, ct, e6t, e2)
        eq(ct(e7t, e1), l2)        # 153.7
        eq(ctr(e7t, e1), right_v)  # 333.7
        eq(ct(e7t, e3), left)      # 75.0
        eq(ctr(e7t, e3), right_u)  # 255.0

        eq(ct(e2, e6t), l2)              # 140.0
        eq(ctr(e2, e6t), right_v_final)  # 320.0
        eq(ct(e4, e6t), left_v_final)    # 5.0
        eq(ctr(e4, e6t), r2)             # 185.0
        eq(ct(e2, e6tn), l2)             # 140.0
        eq(ctr(e2, e6tn), right_v)       # 320.0
        eq(ct(e4, e6tn), left_v)         # 5.0
        eq(ctr(e4, e6tn), r2)            # 185.0
        eq(ct(e4, e7t), nonstop)
        eq(ctr(e4, e7t), nonstop)

        er(AssertionError, ct, e6t, e7t)
        er(AssertionError, ctr, e6t, e7t)
        eq(ct(e7t, e6t), left_final)   # 80.0
        eq(ctr(e7t, e6t), left_final)  # 80.0
        eq(ct(e7t, e6tn), left)        # 80.0
        eq(ctr(e7t, e6tn), left)       # 80.0

        e1[2]['facing'] = PathFacing.FORWARD_UNI
        e2[2]['facing'] = PathFacing.FORWARD_UNI
        e3[2]['facing'] = PathFacing.REVERSE_UNI
        e4[2]['facing'] = PathFacing.REVERSE_UNI

        eq(ct(e1, e2), left)     # 90.0
        eq(ct(e2, e3), right)    # 315.0
        eq(ct(e3, e4), nonstop)  # 0.0
        eq(ct(e4, e5), l2)       # 166.0
        eq(ct(e5, e1), left)     # 92.7
        eq(ct(e5, e4), r2)       # 194.0
        eq(ct(e2, e2), right)    # 303.7
        eq(ct(e3, e1), right_u)  # 258.7
        eq(ct(e3, e2), right_v)  # 348.7

        eq(ctr(e1, e2), left)     # 90.0
        eq(ctr(e2, e3), right)    # 315.0
        eq(ctr(e3, e4), nonstop)  # 0.0
        eq(ctr(e4, e5), right_v)  # 346.0
        eq(ctr(e5, e1), right)    # 272.7
        eq(ctr(e5, e4), left_v)   # 14.0
        eq(ctr(e2, e2), right)    # 303.7
        eq(ctr(e3, e1), right_u)  # 258.7
        eq(ctr(e3, e2), right_v)  # 348.7

        er(AssertionError, ct, e6t, e1)
        er(AssertionError, ct, e6t, e2)
        eq(ct(e7t, e1), l2)        # 153.7
        eq(ctr(e7t, e1), l2)       # 153.7
        eq(ct(e7t, e3), right_u)   # 255.0
        eq(ctr(e7t, e3), right_u)  # 255.0

        eq(ct(e2, e6t), l2)    # 140.0
        eq(ctr(e2, e6t), l2)   # 140.0
        eq(ct(e4, e6t), r2)    # 185.0
        eq(ctr(e4, e6t), r2)   # 185.0
        eq(ct(e2, e6tn), l2)   # 140.0
        eq(ctr(e2, e6tn), l2)  # 140.0
        eq(ct(e4, e6tn), r2)   # 185.0
        eq(ctr(e4, e6tn), r2)  # 185.0
        eq(ct(e4, e7t), nonstop)
        eq(ctr(e4, e7t), nonstop)

    def test_check_transition_with_nrz(self):
        (nonstop, left_v, left_v_final, left, left_final, left_u, l2, l3,
            right_v, right_v_final, right, right_final, right_u, r2, r3,
            uturn, invalid) = self.results
        e1, e2, e3, e4, e5, e6t, e6tn, e7t = self.paths
        ct, ctr = self.ct, self.ctr
        eq, er = self.expectEqual, self.expectRaises

        nrn = self.nrn
        nrn[0] = (1 << 16) - (1 << 9)  # 90 -> 150
        nrn[3] = 1 << 5                # 50
        nrn[4] = 1 << 35               # 350
        nrn[5] = 1 << 14               # 140

        # two headings
        eq(ct(310.0, Heading.NA, j=5), nonstop)   # -     : nrn[5]
        eq(ctr(140.0, Heading.NA, j=5), nonstop)  # -     : nrn[5]
        eq(ct(306.0, 40.0, j=4), r3)              # 94.0  : nrn[4]
        eq(ct(124.9, 300.0, j=4), l2)             # 175.1 : nrn[4]
        eq(ct(170.0, 0.0, j=3), l3)               # 190.0 : nrn[3]
        eq(ctr(150.0, 105.0, j=3), right_final)   # 315.0 : nrn[3]

        # heading and path
        eq(ct(118.8, e1), nonstop)   # 4.9   : nrn[0]
        eq(ctr(35.0, e2), nonstop)   # 358.7 : -
        eq(ctr(315.0, e4), right)    # 270.0 : nrn[3]
        eq(ct(296.0, e5), r3)        # 95.0  : nrn[4]
        eq(ct(140.0, e6t), invalid)  # 270.0 : nrn[5]
        eq(ctr(230.0, e6tn), l2)     # 180.0 : nrn[5]

        # path and heading
        eq(ct(e3, 80.0), r3)             # 35.0  : nrn[3]
        eq(ctr(e4, 0.0), r3)             # 135.0 : nrn[4]
        eq(ct(e5, 216.0), r2)            # 185.0 : nrn[5]
        eq(ct(e4, 40.0), right_v_final)  # 355.0 : nrn[4]

        # two paths
        eq(ct(e3, e4), nonstop)   # 0.0   : nrn[3]
        eq(ct(e4, e5), right_v)   # 346.0 : nrn[4]
        eq(ctr(e3, e4), nonstop)  # 0.0   : nrn[3]
        eq(ctr(e4, e5), right_v)  # 346.0 : nrn[4]
        eq(ct(e7t, e3), left)     # 75.0  : -
        eq(ctr(e7t, e6tn), left)  # 80.0  : nrn[5]


class TestModelsX(TestCase):

    @classmethod
    def setUpClass(cls):
        super(TestModelsX, cls).setUpClass()

        # robot : differential (unrestricted)
        # robot2: trailer (no reverse and no u-turn)
        # robot3: no reverse
        cls.robot, cls.robot2, cls.robot3 = setup_robots()

        cls.home = cls.robot.models.get_agv_home_location()
        cls.locations = [cls.robot.models.get_location_from_station(s) for s in
            ['S1', 'S2', 'S3', 'S4', 'S5', 'T1', 'T2', 'U0', 'U1', 'U2', 'U3', 'U4', 'U5']]

    @classmethod
    def tearDownClass(cls):
        remove_ocgs()
        super(TestModelsX, cls).tearDownClass()

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
        eq(geoms[2], ((-0.61, -0.275 - 0.15, 0.72, 0.395), (-0.6, -0.3 - 0.15, 0.5, 0.42)))
        for i in range(3, 6):
            eq(geoms[i], (q, (-0.5, -0.3, -0.1 + 0.4, 0.3)))

        hit_all = (1 << 36) - 1
        for i in range(6):
            if i in [1, 2]:
                continue
            eq(geoms[geoms[i]].nrn, {10: hit_all})

        nrn = geoms[geoms[1]].nrn
        eq(len(nrn), 5)
        eq(nrn[3], (
            1 << 11 | 1 << 16        # 110, 160
        ))
        eq(nrn[4], (
            (1 << 6) - 1 |           # -> 50
            (1 << 18) - (1 << 13) |  # 130 ->
            (1 << 24) - (1 << 18) |  # -> 230
            (1 << 36) - (1 << 31)    # 310 ->
        ))
        eq(nrn[6], (
            (1 << 27) - (1 << 19)    # 190 -> 260
        ))
        eq(nrn[10], hit_all)
        eq(nrn[100001], (
            1 << 20 | 1 << 25        # 200, 250
        ))

        nrn = geoms[geoms[2]].nrn
        eq(len(nrn), 3)
        eq(nrn[4], (
            (1 << 7) - 1 |           # -> 60
            (1 << 18) - (1 << 12) |  # 120 ->
            (1 << 25) - (1 << 18) |  # -> 240
            (1 << 36) - (1 << 30)    # 300 ->
        ))
        eq(nrn[6], (
            1 << 2 | (1 << 9) - (1 << 7)  # 20, 70 -> 80
        ))
        eq(nrn[10], hit_all)

    def test_home_facing(self):
        home = self.home
        s1, s2, s3, s4, s5, t1, t2, u0, u1, u2, u3, u4, u5 = self.locations

        eq, er = self.expectEqual, self.expectRaises

        def gp(*s1s2):
            j1, j2 = next(zip(*s1s2))
            path = self.robot.models.get_path(j1, j2)
            eq(path, self.robot2.models.get_path(j1, j2))
            eq(path, self.robot3.models.get_path(j1, j2))
            return path

        eq(gp(home, s1)['facing'], PathFacing.FORWARD_UNI)
        er(KeyError, gp, home, s5)
        eq(gp(home, t1)['shape'], PathShape.TELEPORT)
        eq(gp(s1, home)['facing'], PathFacing.REVERSE_UNI)
        er(KeyError, gp, s5, home)
        eq(gp(t1, home)['shape'], PathShape.TELEPORT)

    def test_validate_constrained_path(self):
        home = self.home
        home_na = (home[0], Heading.NA)
        home_rotated = (home[0], 180.0)
        s1, s2, s3, s4, s5, t1, t2, u0, u1, u2, u3, u4, u5 = self.locations

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
        er(AssertionError, vcpr, s3, s4, [s3, s4])

        # ban rotation at home
        eq(vcp(home, home_rotated, [home]), (False, False, False))
        eq(vcpr(s1, home_rotated, [s1, home]), (False, False, False))
        eq(vcpr(home_rotated, home, [home]), (False, False, False))
        eq(vcp(home_rotated, s1, [home, s1]), (False, False, False))

        # same spot
        eq(vcp(home, home, [home]), (True, True, True))
        eq(vcpr(home, home_na, [home]), (True, True, True))
        eq(vcp(t1, t1, [t1]), (True, True, True))  # nrz
        eq(vcp(t2, t2, [t2]), (True, True, True))
        eq(vcpr(u5, u5, [u5]), (True, True, True))

        eq(vcp(t1, (t1[0], 0.0), [t1]), (False, False, False))  # nrz
        eq(vcpr(t1, (t1[0], 0.0), [t1, t2, t1]), (False, False, False))  # nrz
        eq(vcp(t1, (t1[0], 270.0), [t1]), (False, False, False))  # nrz
        eq(vcpr(t2, (t2[0], 0.0), [t2, t1, t2]), (True, False, False))
        eq(vcp(u4, (u4[0], 180.0), [u4]), (True, False, True))
        eq(vcpr(u4, (u4[0], Heading.NA), [u4]), (True, True, True))

        # traversals
        eq(vcp(home, s1, [home, s1]), (True, True, True))
        eq(vcp(t2, home, [t2, t1, home]), (True, True, True))
        eq(vcp(t2, s1, [t2, t1, home, s1]), (True, True, True))
        eq(vcp(s1, t2, [s1, home, t1, t2]), (True, False, False))
        eq(vcp((s4[0], 84.9), (s5[0], Heading.NA), [s4, s5]), (True, False, True))
        eq(vcp(s4, (u5[0], Heading.NA), [s4, s5, u5]), (True, True, True))
        eq(vcp(s5, s4, [s5, u5, u4, s4]), (True, True, True))  # with nonstop teleport

        for t in self.robot2.models.teleports:
            t['nonStopTransition'] = False
        try:
            reconstruct_line_graph(self.robot2.models)
            eq(vcp(s5, s4, [s5, u5, u4, s4]), (True, False, True))  # without nonstop teleport
        finally:
            for t in self.robot2.models.teleports:
                t['nonStopTransition'] = True
            reconstruct_line_graph(self.robot2.models)

        # reverse traversals
        eq(vcpr(home, s1, [home, s1]), (True, True, True))
        eq(vcpr(t2, home, [t2, t1, home]), (True, True, True))
        eq(vcpr(t2, s1, [t2, t1, home, s1]), (True, True, True))
        eq(vcpr(s1, t2, [s1, home, t1, t2]), (True, False, False))
        eq(vcpr(s4, (u5[0], Heading.NA), [s4, s5, u5]), (True, False, False))
        eq(vcpr(s5, s4, [s5, u5, u4, s4]), (True, False, False))

        # with payload (trolley), avoid no-rotate zones
        self.robot.base.set_dimension_profile(1)
        self.robot2.base.set_dimension_profile(1)
        self.robot3.base.set_dimension_profile(1)

        X = (9, 0)
        eq(vcp(t2, home, [t2, t1, home]), (True, True, True))
        eq(vcp((s3[0], 60), s4, [s3, X, s4]), (True, False, True))
        eq(vcp(s4, (u5[0], Heading.NA), [s4, s5, u5]), (True, False, True))

        eq(vcpr(t2, home, [t2, t1, home]), (True, True, True))
        eq(vcpr((s3[0], 60), s4, [s3, X, s4]), (False, False, False))
        eq(vcpr(s4, (u5[0], Heading.NA), [s4, s5, u5]), (True, False, False))

    def test_find_shortest_constrained_path(self):
        '''
        differential (unrestricted)
        '''
        X = 99999
        home = self.home
        home_na = (home[0], Heading.NA)
        home_rotated = (home[0], 180.0)
        s1, s2, s3, s4, s5, t1, t2, u0, u1, u2, u3, u4, u5 = self.locations

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
        er(AssertionError, fscp, s3, s4, reverse=True)

        # ban rotation at home
        eqF(fp(home, home_rotated), [])
        eqF(fpr(s1, home_rotated), [])
        eqF(fpr(home_rotated, home), [])
        eqF(fp(home_rotated, s1), [])

        # same spot
        eqT(fp(home, home), [home])
        eqT(fpr(home, home_na), [home])
        eqT(fp(s4, s4), [s4])
        eqT(fp(t1, t1), [t1])  # nrz
        eqT(fpr(t2, t2), [t2])

        eqT(fp((s4[0], 0.0), s4), [s4])
        eqT(fpr(s4, (s4[0], 0.0)), [s4])
        eqF(fp(t1, (t1[0], 270.0)), [])  # nrz
        eqT(fpr(t1, (t1[0], Heading.NA)), [t1])

        # traversals
        eqT(fp(home, s1), [home, s1])
        eqT(fp(s1, home), [s1, home])

        eqT(fp(home, s2), [home, s1, X, s2])
        eqT(fp((s2[0], 90.0), home), [s2, X, s1, home])

        eqT(fp(home, s4), [home, s1, X, s2, X, X, s3, X, s4])
        eqT(fp(s4, home), [s4, X, s3, X, X, s2, X, s1, home])

        eqT(fp(home, t1), [home, t1])
        eqT(fp(t1, home), [t1, home])

        eqT(fp(home, t2), [home, t1, t2])
        eqT(fp(t2, home), [t2, t1, home])

        eqT(fp(home, u1), [home, s1, X, s2, X, X, s3, X, s4, s5, u5, u4, X, u3, X, X, u2, X, u1])
        eqT(fp(u1, home), [u1, X, u2, X, X, u3, X, u4, s4, X, s3, X, X, s2, X, s1, home])

        eqT(fp(s1, u3), [s1, X, s2, X, X, s3, X, s4, s5, u5, u4, X, u3])
        eqT(fp((u3[0], 185.0), s1), [u3, X, u4, s4, X, s3, X, X, s2, X, s1])

        eqT(fp(s4, s5), [s4, s5])
        eqT(fp(s5, s4), [s5, s4])

        eqT(fp(t2, u0), [t2, t1, home, s1, X, s2, X, X, s3, X, s4, s5, u5, u4, X, u3, X, X, u2, X, u1, u0])
        eqT(fp(u0, t2), [u0, u1, X, u2, X, X, u3, X, u4, s4, X, s3, X, X, s2, X, s1, home, t1, t2])

        eqT(fp(u1, u2), [u1, X, u2])
        eqT(fp((u2[0], 179.0), u1), [u2, X, u1])

        eqT(fp(u4, u5), [u4, u5])
        eqT(fp(u5, u4), [u5, u4])

        # reverse traversals
        eqT(fpr(home, s1), [home, s1])
        eqT(fpr(s1, home), [s1, home])

        eqT(fpr(home, s2), [home, s1, X, s2])
        eqT(fpr((s2[0], 90.0), home), [s2, X, s1, home])

        eqT(fpr(home, s4), [home, s1, X, s2, X, X, s3, X, s4])
        eqT(fpr(s4, home), [s4, X, s3, X, X, s2, X, s1, home])

        eqT(fpr(home, t1), [home, t1])
        eqT(fpr(t1, home), [t1, home])

        eqT(fpr(home, t2), [home, t1, t2])
        eqT(fpr(t2, home), [t2, t1, home])

        eqT(fpr(home, u1), [home, s1, X, s2, X, X, s3, X, s4, s5, u5, u4, X, u3, X, X, u2, X, u1])
        eqT(fpr(u1, home), [u1, X, u2, X, X, u3, X, u4, s4, X, s3, X, X, s2, X, s1, home])

        eqT(fpr(s1, u3), [s1, X, s2, X, X, s3, X, s4, s5, u5, u4, X, u3])
        eqT(fpr((u3[0], 185.0), s1), [u3, X, u4, s4, X, s3, X, X, s2, X, s1])

        eqT(fpr(s4, s5), [s4, s5])
        eqT(fpr(s5, s4), [s5, s4])

        eqT(fpr(t2, u0), [t2, t1, home, s1, X, s2, X, X, s3, X, s4, s5, u5, u4, X, u3, X, X, u2, X, u1, u0])
        eqT(fpr(u0, t2), [u0, u1, X, u2, X, X, u3, X, u4, s4, X, s3, X, X, s2, X, s1, home, t1, t2])

        eqT(fpr(u1, u2), [u1, X, u2])
        eqT(fpr((u2[0], 179.0), u1), [u2, X, u1])

        eqT(fpr(u4, u5), [u4, u5])
        eqT(fpr(u5, u4), [u5, u4])

        # with payload (trolley), avoid no-rotate zones
        self.robot.base.set_dimension_profile(1)

        eqT(fp(home, t2), [home, t1, t2])
        eqT(fp(t2, home), [t2, t1, home])

        eqT(fp(s1, (s3[0], 90.0)), [s1, X, s2, X, X, s3])
        eqFT(fp((s3[0], 90.0), s1), [s3, X, s3, X, X, s2, X, s1])

        eqT(fp(s4, u3), [s4, s5, u5, u4, X, u3])
        eqT(fp((u3[0], 185.0), s4), [u3, X, u4, s4])

        eqT(fp(u1, u2), [u1, X, u2])
        eqT(fp((u2[0], 179.0), u1), [u2, X, u1])

        eqT(fpr(home, t2), [home, t1, t2])
        eqT(fpr(t2, home), [t2, t1, home])

        eqFT(fpr(s1, (s3[0], 90.0)), [s1, X, s2, X, X, s3, X, s3])
        eqT(fpr((s3[0], 90.0), s1), [s3, X, X, s2, X, s1])

        eqT(fpr(s4, u3), [s4, s5, u5, u4, X, u3])
        eqT(fpr((u3[0], 185.0), s4), [u3, X, u4, s4])

        eqT(fpr(u1, u2), [u1, X, u2])
        eqT(fpr((u2[0], 179.0), u1), [u2, X, u1])

    def test_find_shortest_constrained_path2(self, nonstop_teleports=True):
        '''
        trailer (no reverse and no u-turn)
        '''
        X = 99999
        home = self.home
        home_na = (home[0], Heading.NA)
        home_rotated = (home[0], 180.0)
        s1, s2, s3, s4, s5, t1, t2, u0, u1, u2, u3, u4, u5 = self.locations

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
        er(AssertionError, fscp, s3, s4, reverse=True)

        # ban rotation at home
        eqF(fp(home, home_rotated), [])
        eqF(fpr(s1, home_rotated), [])
        eqF(fpr(home_rotated, home), [])
        eqF(fp(home_rotated, s1), [])

        # same spot
        eqT(fp(home, home), [home])
        eqT(fpr(home, home_na), [home])
        eqT(fp(s4, s4), [s4])
        eqT(fp(t1, t1), [t1])  # nrz
        eqT(fpr(t2, t2), [t2])

        eqF(fp((s4[0], 0.0), s4), [])
        eqF(fpr(s4, (s4[0], 0.0)), [])
        eqF(fp(t1, (t1[0], 270.0)), [])  # nrz
        eqT(fpr(t1, (t1[0], Heading.NA)), [t1])

        # traversals
        eqT(fp(home, s1), [home, s1])
        eqF(fp(s1, home), [])

        eqT(fp(home, s2), [home, s1, X, s2])
        eqF(fp((s2[0], 90.0), home), [])

        if nonstop_teleports:
            eqFT(fp(home, s4), [home, s1, X, s2, X, X, s3, X, s4, s5, u5, u4, s4])
        else:
            eqF(fp(home, s4), [])
        eqF(fp(s4, home), [])

        eqT(fp(home, t1), [home, t1])
        eqT(fp(t1, home), [t1, home])

        eqF(fp(home, t2), [])
        eqT(fp(t2, home), [t2, t1, home])

        eqF(fp(home, u1), [])
        eqF(fp(u1, home), [])

        if nonstop_teleports:
            eqT(fp(s1, u3), [s1, X, s2, X, X, s3, X, s4, s5, u5, u4, X, u3])
        else:
            eqF(fp(s1, u3), [])
        eqF(fp((u3[0], 185.0), s1), [])

        eqF(fp(s4, s5), [])
        if nonstop_teleports:
            eqFT(fp(s5, s4), [s5, u5, u4, s4])
        else:
            eqF(fp(s5, s4), [])

        eqF(fp(t2, u0), [])
        eqF(fp(u0, t2), [])

        eqT(fp(u1, u2), [u1, X, u2])
        eqF(fp((u2[0], 179.0), u1), [])

        eqF(fp(u4, u5), [])
        eqF(fp(u5, u4), [])

        # reverse traversals
        eqT(fpr(home, s1), [home, s1])
        eqF(fpr(s1, home), [])

        eqF(fpr(home, s2), [])
        eqF(fpr((s2[0], 90.0), home), [])

        eqF(fpr(home, s4), [])
        eqF(fpr(s4, home), [])

        eqT(fpr(home, t1), [home, t1])
        eqT(fpr(t1, home), [t1, home])

        eqF(fpr(home, t2), [])
        eqT(fpr(t2, home), [t2, t1, home])

        eqF(fpr(home, u1), [])
        eqF(fpr(u1, home), [])

        eqF(fpr(s1, u3), [])
        eqF(fpr((u3[0], 185.0), s1), [])

        eqF(fpr(s4, s5), [])
        eqF(fpr(s5, s4), [])

        eqF(fpr(t2, u0), [])
        eqF(fpr(u0, t2), [])

        eqT(fpr(u1, u2), [u1, X, u2])
        eqF(fpr((u2[0], 179.0), u1), [])

        eqF(fpr(u4, u5), [])
        eqF(fpr(u5, u4), [])

        # with payload (trolley), avoid no-rotate zones
        self.robot2.base.set_dimension_profile(1)

        eqF(fp(home, t2), [])
        eqT(fp(t2, home), [t2, t1, home])

        eqT(fp(s1, (s3[0], 90.0)), [s1, X, s2, X, X, s3])
        eqF(fp((s3[0], 90.0), s1), [])

        eqF(fp(s4, u3), [])
        eqT(fp((u3[0], 185.0), s4), [u3, X, u4, s4])

        eqT(fp(u1, u2), [u1, X, u2])
        eqF(fp((u2[0], 179.0), u1), [])

        eqF(fpr(home, t2), [])
        eqT(fpr(t2, home), [t2, t1, home])

        eqF(fpr(s1, (s3[0], 90.0)), [])
        eqF(fpr((s3[0], 90.0), s1), [])

        eqF(fpr(s4, u3), [])
        eqF(fpr((u3[0], 185.0), s4), [])

        eqT(fpr(u1, u2), [u1, X, u2])
        eqF(fpr((u2[0], 179.0), u1), [])

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
        no reverse
        '''
        X = 99999
        home = self.home
        home_na = (home[0], Heading.NA)
        home_rotated = (home[0], 180.0)
        s1, s2, s3, s4, s5, t1, t2, u0, u1, u2, u3, u4, u5 = self.locations

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
        er(AssertionError, fscp, s3, s4, reverse=True)

        # ban rotation at home
        eqF(fp(home, home_rotated), [])
        eqF(fpr(s1, home_rotated), [])
        eqF(fpr(home_rotated, home), [])
        eqF(fp(home_rotated, s1), [])

        # same spot
        eqT(fp(home, home), [home])
        eqT(fpr(home, home_na), [home])
        eqT(fp(s4, s4), [s4])
        eqT(fp(t1, t1), [t1])  # nrz
        eqT(fpr(t2, t2), [t2])

        eqT(fp((s4[0], 0.0), s4), [s4])
        eqT(fpr(s4, (s4[0], 0.0)), [s4])
        eqF(fp(t1, (t1[0], 270.0)), [])  # nrz
        eqT(fpr(t1, (t1[0], Heading.NA)), [t1])

        # traversals
        eqT(fp(home, s1), [home, s1])
        eqF(fp(s1, home), [])

        eqT(fp(home, s2), [home, s1, X, s2])
        eqF(fp((s2[0], 90.0), home), [])

        eqT(fp(home, s4), [home, s1, X, s2, X, X, s3, X, s4])
        eqF(fp(s4, home), [])

        eqT(fp(home, t1), [home, t1])
        eqT(fp(t1, home), [t1, home])

        eqF(fp(home, t2), [])
        eqT(fp(t2, home), [t2, t1, home])

        eqF(fp(home, u1), [])
        eqF(fp(u1, home), [])

        eqT(fp(s1, u3), [s1, X, s2, X, X, s3, X, s4, s5, u5, u4, X, u3])
        eqT(fp((u3[0], 185.0), s1), [u3, X, u4, s4, X, s3, X, X, s2, X, s1])

        eqT(fp(s4, s5), [s4, s5])
        eqT(fp(s5, s4), [s5, s4])

        eqF(fp(t2, u0), [])
        eqF(fp(u0, t2), [])

        eqT(fp(u1, u2), [u1, X, u2])
        eqF(fp((u2[0], 179.0), u1), [])

        eqT(fp(u4, u5), [u4, u5])
        eqT(fp(u5, u4), [u5, u4])

        # reverse traversals
        eqT(fpr(home, s1), [home, s1])
        eqF(fpr(s1, home), [])

        eqF(fpr(home, s2), [])
        eqF(fpr((s2[0], 90.0), home), [])

        eqF(fpr(home, s4), [])
        eqF(fpr(s4, home), [])

        eqT(fpr(home, t1), [home, t1])
        eqT(fpr(t1, home), [t1, home])

        eqF(fpr(home, t2), [])
        eqT(fpr(t2, home), [t2, t1, home])

        eqF(fpr(home, u1), [])
        eqF(fpr(u1, home), [])

        eqF(fpr(s1, u3), [])
        eqF(fpr((u3[0], 185.0), s1), [])

        eqF(fpr(s4, s5), [])
        eqF(fpr(s5, s4), [])

        eqF(fpr(t2, u0), [])
        eqF(fpr(u0, t2), [])

        eqT(fpr(u1, u2), [u1, X, u2])
        eqF(fpr((u2[0], 179.0), u1), [])

        eqF(fpr(u4, u5), [])
        eqF(fpr(u5, u4), [])

        # with payload (trolley), avoid no-rotate zones
        self.robot3.base.set_dimension_profile(1)

        eqF(fp(home, t2), [])
        eqT(fp(t2, home), [t2, t1, home])

        eqT(fp(s1, (s3[0], 90.0)), [s1, X, s2, X, X, s3])
        eqFT(fp((s3[0], 90.0), s1), [s3, X, s3, X, X, s2, X, s1])

        eqT(fp(s4, u3), [s4, s5, u5, u4, X, u3])
        eqT(fp((u3[0], 185.0), s4), [u3, X, u4, s4])

        eqT(fp(u1, u2), [u1, X, u2])
        eqF(fp((u2[0], 179.0), u1), [])

        eqF(fpr(home, t2), [])
        eqT(fpr(t2, home), [t2, t1, home])

        eqF(fpr(s1, (s3[0], 90.0)), [])
        eqF(fpr((s3[0], 90.0), s1), [])

        eqF(fpr(s4, u3), [])
        eqF(fpr((u3[0], 185.0), s4), [])

        eqT(fpr(u1, u2), [u1, X, u2])
        eqF(fpr((u2[0], 179.0), u1), [])


if __name__ == '__main__':
    rospy.init_node('test_models_x', anonymous=True)
    rostest.rosrun('agv05_executor', 'test_models_x', __name__, sys.argv)
