#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from django.core.management import call_command
from django.test.utils import setup_databases
import django
import os
import rospy
import rostest
import sys

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "agv05_webserver.settings-test")
django.setup()
setup_databases(verbosity=2, interactive=False)

if True:  # prevent autopep8 messing up the import sequence
    from agv05_executor.robots.tracked_sim import Goal
    from agv05_executor.skill import UserError
    from agv05_skills.plan import NavigateTo, ResetAgvPosition, ReverseNavigateTo
    from agv05_webserver.system.models import Direction
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


class TestPlan(TestCase):
    granularity = 2  # 2 samples per second
    # speedup_factor = 100  # 100x speed-up; simulation rate = 100 x 2 = 200Hz
    speedup_factor = 0  # speed-up infinitely; run without sleep

    @classmethod
    def setUpClass(cls):
        super(TestPlan, cls).setUpClass()

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

    def test_navigate_to(self):
        '''
        differential (unrestricted)
        '''
        home = self.home
        s1, s2, s3, s4, s5, s6, s7, s8, s9, t1, t2 = self.locations

        robot = self.robot
        base = robot.base
        models = robot.models

        ul = NavigateTo.UNLIMITED_SPEED
        sl, md, mf = 0.4, 0.5, 0.6  # slow, medium, medium fast
        NS, L, R, NSB = Goal.MOTION_NONSTOP, Goal.MOTION_LEFT, Goal.MOTION_RIGHT, Goal.MOTION_NONSTOP_BEZIER
        eq, err = self.expectEqual, self.expectRaisesRegexp
        invalid_path_msg = 'Unable to find a valid path to destination.'

        base.set_initial_map_and_location(models.graph, home)
        base.granularity = self.granularity
        base.speedup_factor = self.speedup_factor

        def reset(station):
            if isinstance(station, tuple):
                base.set_initial_map_and_location(models.graph, station)
            else:
                ResetAgvPosition(robot, station=station).execute({})

        def to(station, **params):
            base.start_recording_goals()
            d = {
                'station': station,
                'align_station_type': 0,
                'next_motion': 0,
                'next_speed': 0,
            }
            d.update(params)
            NavigateTo(robot, **d).execute({})
            return base.stop_recording_goals()

        def rto(station, **params):
            base.start_recording_goals()
            d = {
                'station': station,
                'align_station_type': 1,
                'next_motion': 0,
                'next_speed': 0,
            }
            d.update(params)
            ReverseNavigateTo(robot, **d).execute({})
            return base.stop_recording_goals()

        def atob(a, b, **params):
            reset(a)
            return to(b, **params)

        def artob(a, b, **params):
            reset(a)
            return rto(b, **params)

        def mg(nav, **kwargs):  # make goal
            d = {
                'nav': nav,
                'enable_sensor': True,
            }
            d.update(**kwargs)
            return Goal(**d)

        # same spot
        eq(atob('Home', 'Home'), [])
        eq(artob('S4', 'S4'), [])

        # traversals
        eq(atob('Home', 'S1'), [  # Home -> S1
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
        ])
        eq(to('Home'), [  # S1 -> Home
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S2'), [  # Home -> S2
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=sl),
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT),
        ])
        eq(to('Home'), [  # S2 -> Home
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S3'), [  # Home -> S3
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])
        eq(to('Home'), [  # S3 -> Home
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=md, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S4'), [  # Home -> S4
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=sl),
            mg(Goal.NAV_FORWARD, speed=sl, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
        ])
        eq(to('Home'), [  # S4 -> Home
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NS, next_speed=md),
            mg(Goal.NAV_FORWARD_RIGHT, speed=md, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S5'), [  # Home -> S5
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul)
        ])
        eq(to('Home'), [  # S5 -> Home
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S7'), [  # Home -> S7
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, next_motion=R),  # teleport
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
        ])
        eq(atob((s7[0], Direction.WEST), 'Home'), [  # S7 -> Home
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S8'), [  # Home -> S8
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, next_motion=R),  # teleport
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
        ])
        eq(to('Home'), [  # S8 -> Home
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(atob('S1', 'S9'), [  # S1 -> S9
            mg(Goal.NAV_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul),
        ])
        eq(atob((s9[0], Direction.WEST), 'S1'), [  # S9 -> S1
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE_RIGHT, speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
        ])

        eq(atob('S2', 'S6'), [  # S2 -> S6
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
        ])
        eq(to('S2'), [  # S6 -> S2
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=sl),
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT),
        ])

        eq(atob('S3', 'S4'), [  # S3 -> S4
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=md, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=sl, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
        ])
        eq(to('S3'), [  # S4 -> S3
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
        ])

        eq(to('S5'), [  # S3 -> S5
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=md),
        ])
        eq(atob((s5[0], Direction.SOUTH), 'S3'), [  # S5 -> S3
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=md, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
        ])

        eq(to('S7'), [  # S3 -> S7
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
        ])
        eq(atob((s7[0], Direction.WEST), 'S3'), [  # S7 -> S3
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])

        eq(atob('T1', 'T2'), [  # T1 -> T2
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
        ])
        eq(to('T1'), [  # T2 -> T1
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
        ])

        # reverse traversals
        eq(artob('Home', 'S1'), [  # Home -> S1
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
        ])
        eq(rto('Home'), [  # S1 -> Home
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(rto('S2'), [  # Home -> S2
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl),
        ])
        eq(rto('Home'), [  # S2 -> Home
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(rto('S3', next_motion=NSB), [  # Home -> S3
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, next_motion=NSB),  # teleport
        ])
        eq(rto('Home'), [  # S3 -> Home
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_RIGHT, speed=md, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(rto('S4'), [  # Home -> S4
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
        ])
        eq(rto('Home'), [  # S4 -> Home
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(rto('S5'), [  # Home -> S5
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul),
        ])
        eq(artob((s5[0], Direction.SOUTH), 'Home'), [  # S5 -> Home
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(rto('S7'), [  # Home -> S7
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul),
        ])
        eq(rto('Home', next_motion=NS), [  # S7 -> Home
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_RIGHT, speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(rto('S8'), [  # Home -> S8
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul),
        ])
        eq(artob((s8[0], Direction.NORTH), 'Home', next_motion=L), [  # S8 -> Home
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(artob('S1', 'S9', next_motion=NSB, next_speed=0.3), [  # S1 -> S9
            mg(Goal.NAV_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NSB, next_speed=0.3),
        ])
        eq(artob((s9[0], Direction.WEST), 'S1'), [  # S9 -> S1
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE_RIGHT, speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
        ])

        eq(artob('S2', 'S6', next_motion=R), [  # S2 -> S6
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul, next_motion=R),
        ])
        eq(artob((s6[0], Direction.NORTH), 'S2'), [  # S6 -> S2
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl),
        ])

        eq(artob('S3', 'S4'), [  # S3 -> S4
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_RIGHT, speed=md, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
        ])
        eq(rto('S3'), [  # S4 -> S3
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
        ])

        eq(rto('S5'), [  # S3 -> S5
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_RIGHT, speed=md),
        ])
        eq(artob((s5[0], Direction.SOUTH), 'S3'), [  # S5 -> S3
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=md, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
        ])

        eq(rto('S7'), [  # S3 -> S7
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul),
        ])
        eq(rto('S3'), [  # S7 -> S3
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_RIGHT, speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])

        eq(artob('T1', 'T2'), [  # T1 -> T2
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
        ])
        eq(rto('T1'), [  # T2 -> T1
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul),
        ])

        # with payload (trolley), avoid no-rotate zones
        base.set_dimension_profile(1)

        eq(atob('Home', 'S2'), [  # Home -> S2
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=sl),
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT),
        ])
        eq(to('Home'), [  # S2 -> Home
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S7'), [  # Home -> S7
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, next_motion=R),  # teleport
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
        ])
        eq(atob((s7[0], Direction.WEST), 'Home'), [  # S7 -> Home
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(rto('S2'), [  # Home -> S2
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl),
        ])
        eq(rto('Home'), [  # S2 -> Home
            mg(Goal.NAV_UTURN_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(rto('S5'), [  # Home -> S5
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_ROTATE3Q_RIGHT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul),
        ])
        eq(artob((s5[0], Direction.SOUTH), 'Home'), [  # S5 -> Home
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

    def test_navigate_to2(self, nonstop_teleports=True):
        '''
        trailer (no reverse and no u-turn)
        '''
        home = self.home
        s1, s2, s3, s4, s5, s6, s7, s8, s9, t1, t2 = self.locations

        robot = self.robot2
        base = robot.base
        models = robot.models

        ul = NavigateTo.UNLIMITED_SPEED
        sl, md, mf = 0.4, 0.5, 0.6  # slow, medium, medium fast
        NS, L, R, NSB = Goal.MOTION_NONSTOP, Goal.MOTION_LEFT, Goal.MOTION_RIGHT, Goal.MOTION_NONSTOP_BEZIER
        eq, err = self.expectEqual, self.expectRaisesRegexp
        invalid_path_msg = 'Unable to find a valid path to destination.'

        base.set_initial_map_and_location(models.graph, home)
        base.granularity = self.granularity
        base.speedup_factor = self.speedup_factor

        def reset(station):
            if isinstance(station, tuple):
                base.set_initial_map_and_location(models.graph, station)
            else:
                ResetAgvPosition(robot, station=station).execute({})

        def to(station, **params):
            base.start_recording_goals()
            d = {
                'station': station,
                'align_station_type': 0,
                'next_motion': 0,
                'next_speed': 0,
            }
            d.update(params)
            NavigateTo(robot, **d).execute({})
            return base.stop_recording_goals()

        def rto(station, **params):
            base.start_recording_goals()
            d = {
                'station': station,
                'align_station_type': 1,
                'next_motion': 0,
                'next_speed': 0,
            }
            d.update(params)
            ReverseNavigateTo(robot, **d).execute({})
            return base.stop_recording_goals()

        def atob(a, b, **params):
            reset(a)
            return to(b, **params)

        def artob(a, b, **params):
            reset(a)
            return rto(b, **params)

        def mg(nav, **kwargs):  # make goal
            d = {
                'nav': nav,
                'enable_sensor': True,
            }
            d.update(**kwargs)
            return Goal(**d)

        nms_teleport = dict(next_motion=NS, next_speed=ul) if nonstop_teleports else {}
        nm_teleport = dict(next_motion=NS) if nonstop_teleports else {}

        # same spot
        eq(atob('Home', 'Home'), [])
        eq(artob('S4', 'S4'), [])

        # traversals
        eq(atob('Home', 'S1'), [  # Home -> S1
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
        ])
        eq(to('Home'), [  # S1 -> Home
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, next_motion=R),  # teleport
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        err(UserError, invalid_path_msg, to, 'S2')  # Home -> S2
        eq(atob('S2', 'Home'), [  # S2 -> Home
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S3'), [  # Home -> S3
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, **nms_teleport),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])
        eq(to('Home'), [  # S3 -> Home
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S4'), [  # Home -> S4
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, **nms_teleport),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, **nm_teleport),  # teleport
            mg(Goal.NAV_FORWARD, speed=0.4),  # teleport
        ])
        eq(to('Home'), [  # S4 -> Home
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S5'), [  # Home -> S5
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul)
        ])
        if nonstop_teleports:
            eq(to('Home'), [  # S5 -> Home
                mg(Goal.NAV_ROTATE_LEFT),
                mg(Goal.NAV_FORWARD_LEFT, speed=md, next_motion=R),
                mg(Goal.NAV_ROTATE_RIGHT),
                mg(Goal.NAV_FORWARD, speed=0.4, next_motion=NS),  # teleport
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
                mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
                mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
                mg(Goal.NAV_ROTATE_RIGHT),
                mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
            ])
        else:
            err(UserError, invalid_path_msg, to, 'Home')  # S5 -> Home

        eq(atob('Home', 'S7'), [  # Home -> S7
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, **nms_teleport),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, next_motion=R),  # teleport
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
        ])
        eq(atob((s7[0], Direction.WEST), 'Home'), [  # S7 -> Home
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S8'), [  # Home -> S8
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, **nms_teleport),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, next_motion=R),  # teleport
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
        ])
        eq(to('Home'), [  # S8 -> Home
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(atob('S1', 'S9'), [  # S1 -> S9
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, next_motion=R),  # teleport
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul),
        ])
        err(UserError, invalid_path_msg, atob, (s9[0], Direction.WEST), 'S1')  # S9 -> S1

        eq(atob('S2', 'S6'), [  # S2 -> S6
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
        ])
        err(UserError, invalid_path_msg, to, 'S2')  # S6 -> S2

        eq(atob('S3', 'S4'), [  # S3 -> S4
            mg(Goal.NAV_FORWARD, speed=0.4),  # teleport
        ])
        if nonstop_teleports:
            eq(to('S3'), [  # S4 -> S3
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NS, next_speed=md),
                mg(Goal.NAV_FORWARD_RIGHT, speed=md, next_motion=R),
                mg(Goal.NAV_ROTATE_RIGHT),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
                mg(Goal.NAV_ROTATE_RIGHT),
                mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
            ])
        else:
            eq(to('S3'), [  # S4 -> S3
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
                mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
                mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
                mg(Goal.NAV_ROTATE_RIGHT),
                mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD_RIGHT, speed=ul, **nms_teleport),
                mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
            ])

        eq(to('S5'), [  # S3 -> S5
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=md),
        ])
        if nonstop_teleports:
            eq(atob((s5[0], Direction.SOUTH), 'S3'), [  # S5 -> S3
                mg(Goal.NAV_ROTATE_LEFT),
                mg(Goal.NAV_FORWARD_LEFT, speed=md, next_motion=R),
                mg(Goal.NAV_ROTATE_RIGHT),
                mg(Goal.NAV_FORWARD, speed=0.4, next_motion=NS),  # teleport
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NS, next_speed=md),
                mg(Goal.NAV_FORWARD_RIGHT, speed=md, next_motion=R),
                mg(Goal.NAV_ROTATE_RIGHT),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
                mg(Goal.NAV_ROTATE_RIGHT),
                mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
            ])
        else:
            err(UserError, invalid_path_msg, atob, (s5[0], Direction.SOUTH), 'S3')  # S5 -> S3

        eq(atob('S3', 'S7'), [  # S3 -> S7
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
        ])
        eq(atob((s7[0], Direction.WEST), 'S3'), [  # S7 -> S3
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, **nms_teleport),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])

        eq(atob('T1', 'T2'), [  # T1 -> T2
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul),
        ])
        err(UserError, invalid_path_msg, to, 'T1')  # T2 -> T1

        # reverse traversals
        eq(artob('Home', 'S1'), [  # Home -> S1
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
        ])
        err(UserError, invalid_path_msg, rto, 'Home')  # S1 -> Home

        err(UserError, invalid_path_msg, artob, 'Home', 'S2')  # Home -> S2
        err(UserError, invalid_path_msg, artob, 'S2', 'Home')  # S2 -> Home

        eq(artob('Home', 'S3'), [  # Home -> S3
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, **nms_teleport),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])
        err(UserError, invalid_path_msg, rto, 'Home')  # S3 -> Home

        eq(artob('Home', 'S4'), [  # Home -> S4
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, **nms_teleport),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, **nm_teleport),  # teleport
            mg(Goal.NAV_FORWARD, speed=0.4),  # teleport
        ])
        err(UserError, invalid_path_msg, rto, 'Home')  # S4 -> Home

        err(UserError, invalid_path_msg, artob, 'Home', 'S5')  # Home -> S5
        err(UserError, invalid_path_msg, artob, (s5[0], Direction.SOUTH), 'Home')  # S5 -> Home

        err(UserError, invalid_path_msg, artob, 'Home', 'S7')  # Home -> S7
        err(UserError, invalid_path_msg, artob, (s7[0], Direction.WEST), 'Home')  # S7 -> Home

        err(UserError, invalid_path_msg, artob, 'Home', 'S8')  # Home -> S8
        eq(artob((s8[0], Direction.NORTH), 'Home'), [  # S8 -> Home
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        err(UserError, invalid_path_msg, artob, 'S1', 'S9')  # S1 -> S9
        err(UserError, invalid_path_msg, artob, (s9[0], Direction.WEST), 'S1')  # S9 -> S1

        err(UserError, invalid_path_msg, artob, 'S2', 'S6')  # S2 -> S6
        err(UserError, invalid_path_msg, artob, (s6[0], Direction.NORTH), 'S2')  # S6 -> S2

        eq(artob('S3', 'S4'), [  # S3 -> S4
            mg(Goal.NAV_FORWARD, speed=0.4),  # teleport
        ])
        err(UserError, invalid_path_msg, rto, 'S3')  # S4 -> S3

        err(UserError, invalid_path_msg, artob, 'S3', 'S5')  # S3 -> S5
        err(UserError, invalid_path_msg, artob, (s5[0], Direction.SOUTH), 'S3')  # S5 -> S3

        err(UserError, invalid_path_msg, artob, 'S3', 'S7')  # S3 -> S7
        err(UserError, invalid_path_msg, artob, (s7[0], Direction.WEST), 'S3')  # S7 -> S3

        err(UserError, invalid_path_msg, artob, 'T1', 'T2')  # T1 -> T2
        err(UserError, invalid_path_msg, artob, 'T2', 'T1')  # T2 -> T1

        # with payload (trolley), avoid no-rotate zones
        base.set_dimension_profile(1)

        err(UserError, invalid_path_msg, atob, 'Home', 'S2')  # Home -> S2
        if nonstop_teleports:
            eq(atob('S2', 'Home'), [  # S2 -> Home
                mg(Goal.NAV_FORWARD, speed=sl, next_motion=L),
                mg(Goal.NAV_ROTATE_LEFT),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
                mg(Goal.NAV_ROTATE_LEFT),
                mg(Goal.NAV_FORWARD_LEFT, speed=md, next_motion=R),
                mg(Goal.NAV_ROTATE_RIGHT),
                mg(Goal.NAV_FORWARD, speed=0.4, next_motion=NS),  # teleport
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
                mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
                mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
                mg(Goal.NAV_ROTATE_RIGHT),
                mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
            ])
        else:
            err(UserError, invalid_path_msg, atob, 'S2', 'Home')  # S2 -> Home

        eq(atob('Home', 'S7'), [  # Home -> S7
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, **nms_teleport),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, next_motion=R),  # teleport
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
        ])
        if nonstop_teleports:
            eq(atob((s7[0], Direction.WEST), 'Home'), [  # S7 -> Home
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
                mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul, next_motion=L),
                mg(Goal.NAV_ROTATE_LEFT),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
                mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NSB, next_speed=ul),
                mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul, next_motion=R),
                mg(Goal.NAV_ROTATE_RIGHT),
                mg(Goal.NAV_FORWARD, speed=0.4, next_motion=NS),  # teleport
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
                mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
                mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
                mg(Goal.NAV_FORWARD, speed=ul, next_motion=R),
                mg(Goal.NAV_ROTATE_RIGHT),
                mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
            ])
        else:
            err(UserError, invalid_path_msg, atob, (s7[0], Direction.WEST), 'Home')  # S7 -> Home

        err(UserError, invalid_path_msg, artob, 'Home', 'S2')  # Home -> S2
        err(UserError, invalid_path_msg, artob, 'S2', 'Home')  # S2 -> Home

        err(UserError, invalid_path_msg, artob, 'Home', 'S5')  # Home -> S5
        err(UserError, invalid_path_msg, artob, (s5[0], Direction.SOUTH), 'Home')  # S5 -> Home

    def test_navigate_to2_without_nonstop_teleports(self):
        for t in self.robot2.models.teleports:
            t['nonStopTransition'] = False
        try:
            reconstruct_line_graph(self.robot2.models)
            self.test_navigate_to2(nonstop_teleports=False)
        finally:
            for t in self.robot2.models.teleports:
                t['nonStopTransition'] = True
            reconstruct_line_graph(self.robot2.models)

    def test_navigate_to3(self):
        '''
        no right-turn
        '''
        home = self.home
        s1, s2, s3, s4, s5, s6, s7, s8, s9, t1, t2 = self.locations

        robot = self.robot3
        base = robot.base
        models = robot.models

        ul = NavigateTo.UNLIMITED_SPEED
        sl, md, mf = 0.4, 0.5, 0.6  # slow, medium, medium fast
        NS, L, R, NSB = Goal.MOTION_NONSTOP, Goal.MOTION_LEFT, Goal.MOTION_RIGHT, Goal.MOTION_NONSTOP_BEZIER
        eq, err = self.expectEqual, self.expectRaisesRegexp
        invalid_path_msg = 'Unable to find a valid path to destination.'

        base.set_initial_map_and_location(models.graph, home)
        base.granularity = self.granularity
        base.speedup_factor = self.speedup_factor

        def reset(station):
            if isinstance(station, tuple):
                base.set_initial_map_and_location(models.graph, station)
            else:
                ResetAgvPosition(robot, station=station).execute({})

        def to(station, **params):
            base.start_recording_goals()
            d = {
                'station': station,
                'align_station_type': 0,
                'next_motion': 0,
                'next_speed': 0,
            }
            d.update(params)
            NavigateTo(robot, **d).execute({})
            return base.stop_recording_goals()

        def rto(station, **params):
            base.start_recording_goals()
            d = {
                'station': station,
                'align_station_type': 1,
                'next_motion': 0,
                'next_speed': 0,
            }
            d.update(params)
            ReverseNavigateTo(robot, **d).execute({})
            return base.stop_recording_goals()

        def atob(a, b, **params):
            reset(a)
            return to(b, **params)

        def artob(a, b, **params):
            reset(a)
            return rto(b, **params)

        def mg(nav, **kwargs):  # make goal
            d = {
                'nav': nav,
                'enable_sensor': True,
            }
            d.update(**kwargs)
            return Goal(**d)

        # same spot
        eq(atob('Home', 'Home'), [])
        eq(artob('S4', 'S4'), [])

        # traversals
        eq(atob('Home', 'S1'), [  # Home -> S1
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
        ])
        eq(to('Home'), [  # S1 -> Home
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S2'), [  # Home -> S2
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=sl),
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT),
        ])
        eq(to('Home'), [  # S2 -> Home
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S3'), [  # Home -> S3
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])
        eq(to('Home'), [  # S3 -> Home
            mg(Goal.NAV_FORWARD, speed=0.4, next_motion=NS),  # teleport
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S4'), [  # Home -> S4
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, next_motion=NS),  # teleport
            mg(Goal.NAV_FORWARD, speed=0.4),  # teleport
        ])
        eq(to('Home'), [  # S4 -> Home
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S5'), [  # Home -> S5
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=sl),
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=sl, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul)
        ])
        eq(to('Home'), [  # S5 -> Home
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(atob('Home', 'S7'), [  # Home -> S7
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=sl),
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=sl, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
        ])
        eq(atob((s7[0], Direction.WEST), 'Home'), [  # S7 -> Home
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(to('S8'), [  # Home -> S8
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=sl),
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=sl, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
        ])
        eq(to('Home'), [  # S8 -> Home
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(atob('S1', 'S9'), [  # S1 -> S9
            mg(Goal.NAV_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul),
        ])
        eq(atob((s9[0], Direction.WEST), 'S1'), [  # S9 -> S1
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE_RIGHT, speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
        ])

        eq(atob('S2', 'S6'), [  # S2 -> S6
            mg(Goal.NAV_FORWARD, speed=sl, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
        ])
        eq(to('S2'), [  # S6 -> S2
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=sl),
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT),
        ])

        eq(atob('S3', 'S4', next_motion=R), [  # S3 -> S4
            mg(Goal.NAV_FORWARD, speed=0.4, next_motion=R),  # teleport
        ])
        eq(to('S3'), [  # S4 -> S3
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
        ])

        eq(to('S5'), [  # S3 -> S5
            mg(Goal.NAV_FORWARD, speed=0.4, next_motion=NS),  # teleport
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NS, next_speed=md),
            mg(Goal.NAV_FORWARD_RIGHT, speed=md),
        ])
        eq(atob((s5[0], Direction.SOUTH), 'S3'), [  # S5 -> S3
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])

        eq(atob('S3', 'S7'), [  # S3 -> S7
            mg(Goal.NAV_FORWARD, speed=0.4, next_motion=NS),  # teleport
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
        ])
        eq(atob((s7[0], Direction.WEST), 'S3'), [  # S7 -> S3
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])

        eq(atob('T1', 'T2'), [  # T1 -> T2
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
        ])
        eq(to('T1'), [  # T2 -> T1
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
        ])

        # reverse traversals
        eq(artob('Home', 'S1'), [  # Home -> S1
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
        ])
        eq(rto('Home'), [  # S1 -> Home
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(rto('S2'), [  # Home -> S2
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl),
        ])
        eq(rto('Home'), [  # S2 -> Home
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(artob('Home', 'S3'), [  # Home -> S3
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])
        eq(rto('Home'), [  # S3 -> Home
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(artob('Home', 'S4'), [  # Home -> S4
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, next_motion=NS),  # teleport
            mg(Goal.NAV_FORWARD, speed=0.4),  # teleport
        ])
        eq(rto('Home'), [  # S4 -> Home
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(rto('S5'), [  # Home -> S5
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul),
        ])
        eq(artob((s5[0], Direction.SOUTH), 'Home'), [  # S5 -> Home
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(rto('S7'), [  # Home -> S7
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul),
        ])
        eq(rto('Home'), [  # S7 -> Home
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_RIGHT, speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(rto('S8'), [  # Home -> S8
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul),
        ])
        eq(artob((s8[0], Direction.NORTH), 'Home'), [  # S8 -> Home
            mg(Goal.NAV_REVERSE, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        eq(artob('S1', 'S9'), [  # S1 -> S9
            mg(Goal.NAV_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_LEFT, speed=ul),
        ])
        eq(artob((s9[0], Direction.WEST), 'S1'), [  # S9 -> S1
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE_RIGHT, speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
        ])

        eq(artob('S2', 'S6'), [  # S2 -> S6
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul),
        ])
        eq(artob((s6[0], Direction.NORTH), 'S2'), [  # S6 -> S2
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl),
        ])

        eq(artob('S3', 'S4'), [  # S3 -> S4
            mg(Goal.NAV_FORWARD, speed=0.4),  # teleport
        ])
        eq(rto('S3'), [  # S4 -> S3
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=sl, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])

        eq(rto('S5'), [  # S3 -> S5
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_RIGHT, speed=md),
        ])
        eq(artob((s5[0], Direction.SOUTH), 'S3'), [  # S5 -> S3
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])

        eq(rto('S7'), [  # S3 -> S7
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_REVERSE, speed=ul),
        ])
        eq(rto('S3'), [  # S7 -> S3
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_REVERSE_RIGHT, speed=ul),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4),  # teleport
        ])

        eq(artob('T1', 'T2'), [  # T1 -> T2
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
        ])
        eq(rto('T1'), [  # T2 -> T1
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul),
        ])

        # with payload (trolley), avoid no-rotate zones
        base.set_dimension_profile(1)

        eq(atob('Home', 'S2'), [  # Home -> S2
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=sl),
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT),
        ])
        eq(to('Home'), [  # S2 -> Home
            mg(Goal.NAV_FORWARD, speed=sl),
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])

        err(UserError, invalid_path_msg, to, 'S7')  # Home -> S7
        eq(atob((s7[0], Direction.WEST), 'Home'), [  # S7 -> Home
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=NSB, next_speed=ul),
            mg(Goal.NAV_BEZIER_FORWARD_RIGHT, speed=ul),
            mg(Goal.NAV_UTURN_LEFT),
            mg(Goal.NAV_FORWARD_LEFT, speed=ul, enable_sensor=False),
        ])

        err(UserError, invalid_path_msg, rto, 'S2')  # Home -> S2
        err(UserError, invalid_path_msg, artob, 'S2', 'Home')  # S2 -> Home

        eq(artob('Home', 'S5'), [  # Home -> S5
            mg(Goal.NAV_FORWARD_RIGHT, speed=ul, next_motion=NS, next_speed=ul),
            mg(Goal.NAV_FORWARD_RIGHT, speed=0.4, next_motion=L),  # teleport
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_RIGHT, speed=md),
        ])
        eq(artob((s5[0], Direction.SOUTH), 'Home'), [  # S5 -> Home
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE_LEFT, speed=ul, enable_sensor=False),
        ])


if __name__ == '__main__':
    rospy.init_node('test_plan', anonymous=True)
    rostest.rosrun('agv05_skills', 'test_plan', TestPlan, sys.argv)
