#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from django.core.management import call_command
from django.test.utils import setup_databases
import django
import math
import os
import rospy
import rostest
import sys

os.environ['TRACKLESS'] = '1'
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "agv05_webserver.settings-test")
django.setup()
setup_databases(verbosity=2, interactive=False)

if True:  # prevent autopep8 messing up the import sequence
    from agv05_executor.robots.tracked_sim import Goal
    from agv05_executor.robots.trackless_sim import Goal as GoalX
    from agv05_executor.skill import UserError
    from agv05_msgs.msg import Path
    from agv05_skills.plan_x import NavigateToX, ResetAgvPositionX, ReverseNavigateToX
    from agv05_webserver.system.signals import signals_suppressed
    from agv05_webserver.systemx.models import Heading, PathShape
    from geometry_msgs.msg import Point32, Polygon, Pose2D

    # relative imports
    from fixtures_x import *
    from utils import *


# Implement equality functions for goal types.
def __Goal__eq__(a, b):
    for k in Goal.__slots__:
        if k == 'distance':
            if not __isclose(a.distance, b.distance):
                return False
        elif getattr(a, k) != getattr(b, k):
            return False
    return True


def __GoalX__eq__(a, b):
    for k in GoalX.__slots__:
        if k == 'next_distance':
            if not __isclose(a.next_distance, b.next_distance):
                return False
        elif getattr(a, k) != getattr(b, k):
            return False
    return True


def __isclose(a, b):
    return abs(a - b) < 1e-7


Goal.__eq__ = __Goal__eq__
GoalX.__eq__ = __GoalX__eq__


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


class TestPlanX(TestCase):
    granularity = 2  # 2 samples per second
    # speedup_factor = 100  # 100x speed-up; simulation rate = 100 x 2 = 200Hz
    speedup_factor = 0  # speed-up infinitely; run without sleep

    @classmethod
    def setUpClass(cls):
        super(TestPlanX, cls).setUpClass()

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
        super(TestPlanX, cls).tearDownClass()

    def tearDown(self):
        self.robot.base.set_dimension_profile(0)
        self.robot2.base.set_dimension_profile(0)
        self.robot3.base.set_dimension_profile(0)

    def test_navigate_to(self):
        '''
        differential (unrestricted)
        '''
        home = self.home
        s1, s2, s3, s4, s5, t1, t2, u0, u1, u2, u3, u4, u5 = self.locations

        robot = self.robot
        base = robot.base
        models = robot.models

        ih = models.invert_heading
        sh = models.compute_edge_start_heading
        eh = models.compute_edge_end_heading
        ish = (lambda e: ih(sh(e)))
        ieh = (lambda e: ih(eh(e)))
        gj = models.get_junction

        ul = NavigateToX.UNLIMITED_SPEED
        sl, md, mf = 0.4, 0.5, 0.6  # slow, medium, medium fast
        NS, L, R, NSB = GoalX.MOTION_NONSTOP, GoalX.MOTION_LEFT, GoalX.MOTION_RIGHT, GoalX.MOTION_NONSTOP_BEZIER
        eq, err = self.expectEqual, self.expectRaisesRegexp
        invalid_path_msg = 'Unable to find a valid path to destination.'

        base.set_initial_map_and_location(models.graph, home)
        base.granularity = self.granularity
        base.speedup_factor = self.speedup_factor

        def reset(station):
            if isinstance(station, tuple):
                base.set_initial_map_and_location(models.graph, station)
            else:
                ResetAgvPositionX(robot, station=station).execute({})

        def to(station, **params):
            base.start_recording_goals()
            d = {
                'station': station,
                'align_station_type': -1,
                'next_motion': 0,
                'next_speed': 0,
                'sense_line': 0,
            }
            d.update(params)
            NavigateToX(robot, **d).execute({})
            return base.stop_recording_goals()

        def rto(station, **params):
            base.start_recording_goals()
            d = {
                'station': station,
                'align_station_type': -1,
                'next_motion': 0,
                'next_speed': 0,
                'sense_line': 0,
            }
            d.update(params)
            ReverseNavigateToX(robot, **d).execute({})
            return base.stop_recording_goals()

        def atob(a, b, **params):
            reset(a)
            return to(b, **params)

        def artob(a, b, **params):
            reset(a)
            return rto(b, **params)

        def fp(a, b):
            paths = []
            plan = iter(models.find_shortest_constrained_path(a, b)[0])
            prev_j = next(plan)
            for j in plan:
                e = models.get_path(prev_j, j)
                paths.append((prev_j, j, e))
                prev_j = j
            return paths, poly(paths)

        def fpr(a, b):
            paths = []
            plan = iter(models.find_shortest_constrained_path(a, b, reverse=True)[0])
            prev_j = next(plan)
            for j in plan:
                e = models.get_path(prev_j, j)
                paths.append((prev_j, j, e))
                prev_j = j
            return paths, poly(paths)

        def poly(paths):
            return [Polygon(points=(
                [
                    Point32(**gj(e[0])),
                    Point32(**gj(e[1])),
                ]
                if e[2]['shape'] == PathShape.STRAIGHT else
                [
                    Point32(**gj(e[0])),
                    Point32(**e[2]['cp1']),
                    Point32(**e[2]['cp2']),
                    Point32(**gj(e[1])),
                ])
                if not models.is_teleport(e[2]) else []
            ) for e in paths]

        def sd(paths):  # sum of distances
            return sum(e[2]['distance'] for e in paths)

        def mg(nav, **kwargs):  # make goal
            d = {
                'nav': nav,
                'enable_sensor': True,
            }
            d.update(**kwargs)
            return Goal(**d)

        def mgx(nav, paths=None, theta=0, loc=None, **kwargs):  # make goalx
            d = {
                'nav': nav,
                'enable_sensor': True,
            }
            if paths:  # straight
                if not isinstance(paths, list):
                    paths = [paths]
                path = paths[0].points
                d['goal_tolerance'] = GoalX.TOLERANCE_GOAL
                d['path_start'] = Pose2D(x=path[0].x, y=path[0].y)
                d['path_end'] = Pose2D(x=path[-1].x, y=path[-1].y, theta=theta)
                if len(path) == 4:
                    d['path_cp1'] = Pose2D(x=path[1].x, y=path[1].y)
                    d['path_cp2'] = Pose2D(x=path[2].x, y=path[2].y)
                d['paths'] = Path(paths=paths)

            elif loc:  # rotate
                j, h = loc
                d['path_end'] = Pose2D(theta=math.radians(h), **gj(j))

            d.update(**kwargs)
            return GoalX(**d)

        # same spot
        eq(atob('Home', 'Home'), [])
        eq(artob('S4', 'S4'), [])

        # traversals
        paths, pp = fp(home, s1)
        eq(atob('Home', 'S1'), [  # Home -> S1
            mgx(GoalX.NAV_DYNAMIC_LINE_FORWARD, pp, -1, speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s1),
        ])
        paths, pp = fp(s1, home)
        eq(to('Home'), [  # S1 -> Home
            mgx(GoalX.NAV_DYNAMIC_LINE_REVERSE, pp, -1, speed=ul, enable_sensor=False),
        ])

        paths, pp = fp(home, s2)
        eq(to('S2'), [  # Home -> S2
            mgx(GoalX.NAV_DYNAMIC_LINE_FORWARD, pp[0], -1, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_FORWARD, pp[1], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[2][0], sh(paths[2]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[2], speed=ul),
        ])
        paths, pp = fp((s2[0], 90.0), home)
        eq(atob((s2[0], 90.0), 'Home'), [  # S2 -> Home
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[0], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_FORWARD, pp[1], speed=ul, next_motion=L),
            mgx(GoalX.NAV_DYNAMIC_LINE_REVERSE, pp[2], -1, speed=ul, enable_sensor=False),
        ])

        paths, pp = fp(home, s4)
        eq(to('S4'), [  # Home -> S4
            mgx(GoalX.NAV_DYNAMIC_LINE_FORWARD, pp[0], -1, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_FORWARD, pp[1], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[2][0], sh(paths[2]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[2:5], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[3:5])),
            mgx(GoalX.NAV_FORWARD, pp[3:5], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[4:5])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[4], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(eh(paths[4]) - sh(paths[5]))),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
            mgx(GoalX.NAV_FORWARD, pp[6:8], speed=mf, next_motion=NS, next_speed=ul, next_distance=sd(paths[7:8])),
            mgx(GoalX.NAV_FORWARD, pp[7], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s4),
        ])
        paths, pp = fp(s4, home)
        eq(to('Home'), [  # S4 -> Home
            mgx(GoalX.NAV_FORWARD, pp[0:2], speed=ul, next_motion=NS, next_speed=mf, next_distance=sd(paths[1:2])),
            mgx(GoalX.NAV_FORWARD, pp[1], speed=mf),
            mg(Goal.NAV_SEARCH_LINE_LEFT),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[3:6], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[4:6])),
            mgx(GoalX.NAV_FORWARD, pp[4:6], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[5:6])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[5], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[6][0], sh(paths[6]))),
            mgx(GoalX.NAV_FORWARD, pp[6], speed=ul, next_motion=L),
            mgx(GoalX.NAV_DYNAMIC_LINE_REVERSE, pp[7], -1, speed=ul, enable_sensor=False),
        ])

        eq(to('T1'), [])  # Home -> T1
        eq(to('Home'), [])  # T1 -> Home

        paths, pp = fp(home, t2)
        eq(to('T2'), [  # Home -> T2
            # mgx(),  # teleport
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[1], speed=sl),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=t2),
        ])
        paths, pp = fp(t2, home)
        eq(to('Home'), [  # T2 -> Home
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[0], speed=sl, next_motion=NS, next_speed=ul, next_distance=sd(paths[1:2])),
            # mgx(),  # teleport
        ])

        paths, pp = fp(home, u1)
        s5j = gj(s5[0])
        eq(to('U1'), [  # Home -> U1
            mgx(GoalX.NAV_DYNAMIC_LINE_FORWARD, pp[0], -1, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_FORWARD, pp[1], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[2][0], sh(paths[2]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[2:5], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[3:5])),
            mgx(GoalX.NAV_FORWARD, pp[3:5], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[4:5])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[4], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(eh(paths[4]) - sh(paths[5]))),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
            mgx(GoalX.NAV_FORWARD, pp[6:8], speed=mf, next_motion=NS, next_speed=ul, next_distance=sd(paths[7:8])),
            mgx(GoalX.NAV_FORWARD, pp[7], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[8][0], sh(paths[8]))),
            mgx(GoalX.NAV_FORWARD, pp[8], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=s5),
            mgx(GoalX.NAV_FORWARD, path_start=Pose2D(**s5j), path_end=Pose2D(x=s5j['x'], y=s5j['y'] + 2), speed=0.4),  # teleport
            mgx(GoalX.NAV_FORWARD, pp[10], speed=md, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[11][0], sh(paths[11]))),
            mgx(GoalX.NAV_FORWARD, pp[11:16], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[12:16])),
            mgx(GoalX.NAV_FORWARD, pp[12:16], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[13:16])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[13:16], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[14:16])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[14:16], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[15:16])),
            mgx(GoalX.NAV_FORWARD, pp[15], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[16][0], ish(paths[16]))),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[16], speed=ul, next_motion=L),
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(360 - ieh(paths[16]))),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=u1),
        ])
        paths, pp = fp(u1, home)
        eq(to('Home'), [  # U1 -> Home
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(90)),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[2][0], ish(paths[2]))),
            mgx(GoalX.NAV_REVERSE, pp[2], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[3][0], sh(paths[3]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[3:7], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[4:7])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[4:7], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[5:7])),
            mgx(GoalX.NAV_FORWARD, pp[5:7], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[6:7])),
            mgx(GoalX.NAV_FORWARD, pp[6], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=0.4),  # teleport
            mgx(GoalX.NAV_FORWARD, pp[8:10], speed=ul, next_motion=NS, next_speed=mf, next_distance=sd(paths[9:10])),
            mgx(GoalX.NAV_FORWARD, pp[9], speed=mf),
            mg(Goal.NAV_SEARCH_LINE_LEFT),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[11:14], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[12:14])),
            mgx(GoalX.NAV_FORWARD, pp[12:14], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[13:14])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[13], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[14][0], sh(paths[14]))),
            mgx(GoalX.NAV_FORWARD, pp[14], speed=ul, next_motion=L),
            mgx(GoalX.NAV_DYNAMIC_LINE_REVERSE, pp[15], -1, speed=ul, enable_sensor=False),
        ])

        paths, pp = fp(s1, u3)
        eq(atob('S1', 'U3'), [  # S1 -> U3
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_FORWARD, pp[0], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1:4], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[2:4])),
            mgx(GoalX.NAV_FORWARD, pp[2:4], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[3:4])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[3], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(eh(paths[3]) - sh(paths[4]))),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
            mgx(GoalX.NAV_FORWARD, pp[5:7], speed=mf, next_motion=NS, next_speed=ul, next_distance=sd(paths[6:7])),
            mgx(GoalX.NAV_FORWARD, pp[6], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[7][0], sh(paths[7]))),
            mgx(GoalX.NAV_FORWARD, pp[7], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=s5),
            mgx(GoalX.NAV_FORWARD, path_start=Pose2D(**s5j), path_end=Pose2D(x=s5j['x'], y=s5j['y'] + 2), speed=0.4),  # teleport
            mgx(GoalX.NAV_FORWARD, pp[9], speed=md, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[10][0], sh(paths[10]))),
            mgx(GoalX.NAV_FORWARD, pp[10:12], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[11:12])),
            mgx(GoalX.NAV_FORWARD, pp[11], speed=ul),
        ])
        paths, pp = fp((u3[0], 185.0), s1)
        eq(atob((u3[0], 185.0), 'S1'), [  # U3 -> S1
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_FORWARD, pp[0:2], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[1:2])),
            mgx(GoalX.NAV_FORWARD, pp[1], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=0.4),  # teleport
            mgx(GoalX.NAV_FORWARD, pp[3:5], speed=ul, next_motion=NS, next_speed=mf, next_distance=sd(paths[4:5])),
            mgx(GoalX.NAV_FORWARD, pp[4], speed=mf),
            mg(Goal.NAV_SEARCH_LINE_LEFT),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[6:9], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[7:9])),
            mgx(GoalX.NAV_FORWARD, pp[7:9], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[8:9])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[8], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[9][0], sh(paths[9]))),
            mgx(GoalX.NAV_FORWARD, pp[9], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s1),
        ])

        paths, pp = fp(s4, s5)
        eq(atob('S4', 'S5'), [  # S4 -> S5
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_FORWARD, pp[0], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=s5),
        ])
        paths, pp = fp(s5, s4)
        eq(to('S4'), [  # S5 -> S4
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_FORWARD, pp[0], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=s4),
        ])

        paths, pp = fp(t2, u0)
        eq(atob('T2', 'U0'), [  # T2 -> U0
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[0], speed=sl, next_motion=NS, next_speed=ul, next_distance=sd(paths[1:3])),
            # mgx(),  # teleport
            mgx(GoalX.NAV_DYNAMIC_LINE_FORWARD, pp[2], -1, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[3][0], sh(paths[3]))),
            mgx(GoalX.NAV_FORWARD, pp[3], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[4][0], sh(paths[4]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[4:7], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[5:7])),
            mgx(GoalX.NAV_FORWARD, pp[5:7], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[6:7])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[6], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(eh(paths[6]) - sh(paths[7]))),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
            mgx(GoalX.NAV_FORWARD, pp[8:10], speed=mf, next_motion=NS, next_speed=ul, next_distance=sd(paths[9:10])),
            mgx(GoalX.NAV_FORWARD, pp[9], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[10][0], sh(paths[10]))),
            mgx(GoalX.NAV_FORWARD, pp[10], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=s5),
            mgx(GoalX.NAV_FORWARD, path_start=Pose2D(**s5j), path_end=Pose2D(x=s5j['x'], y=s5j['y'] + 2), speed=0.4),  # teleport
            mgx(GoalX.NAV_FORWARD, pp[12], speed=md, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[13][0], sh(paths[13]))),
            mgx(GoalX.NAV_FORWARD, pp[13:18], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[14:18])),
            mgx(GoalX.NAV_FORWARD, pp[14:18], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[15:18])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[15:18], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[16:18])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[16:18], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[17:18])),
            mgx(GoalX.NAV_FORWARD, pp[17], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[18][0], ish(paths[18]))),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[18], speed=ul, next_motion=L),
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(360 - ieh(paths[18]))),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul),
        ])
        paths, pp = fp(u0, t2)
        eq(to('T2'), [  # U0 -> T2
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[2][0], sh(paths[2]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[2], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[3][0], ish(paths[3]))),
            mgx(GoalX.NAV_REVERSE, pp[3], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[4][0], sh(paths[4]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[4:8], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[5:8])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[5:8], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[6:8])),
            mgx(GoalX.NAV_FORWARD, pp[6:8], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[7:8])),
            mgx(GoalX.NAV_FORWARD, pp[7], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=0.4),  # teleport
            mgx(GoalX.NAV_FORWARD, pp[9:11], speed=ul, next_motion=NS, next_speed=mf, next_distance=sd(paths[10:11])),
            mgx(GoalX.NAV_FORWARD, pp[10], speed=mf),
            mg(Goal.NAV_SEARCH_LINE_LEFT),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[12:15], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[13:15])),
            mgx(GoalX.NAV_FORWARD, pp[13:15], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[14:15])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[14], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[15][0], sh(paths[15]))),
            mgx(GoalX.NAV_FORWARD, pp[15], speed=ul, next_motion=L),
            mgx(GoalX.NAV_DYNAMIC_LINE_REVERSE, pp[16], -1, speed=ul),
            # mgx(),  # teleport
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[18], speed=sl),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=t2),
        ])

        paths, pp = fp(u1, u2)
        eq(atob('U1', 'U2', align_station_type=0, next_motion=NS), [  # U1 -> U2
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1], speed=ul),
        ])
        paths, pp = fp((u2[0], 179.0), u1)
        eq(atob((u2[0], 179.0), 'U1', align_station_type=1), [  # U2 -> U1
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[0], speed=ul, next_motion=L),
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(360 - ieh(paths[0]))),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
        ])

        paths, pp = fp(u4, u5)
        eq(atob('U4', 'U5'), [  # U4 -> U5
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_FORWARD, pp[0], speed=md, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=u5),
        ])
        paths, pp = fp(u5, u4)
        eq(to('U4'), [  # U5 -> U4
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_FORWARD, pp[0], speed=md, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=u4),
        ])

        # reverse traversals
        paths, pp = fpr(home, s1)
        eq(artob('Home', 'S1', next_motion=NS), [  # Home -> S1
            mgx(GoalX.NAV_DYNAMIC_LINE_FORWARD, pp, -1, speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s1),
        ])
        paths, pp = fpr(s1, home)
        eq(rto('Home', next_motion=NSB), [  # S1 -> Home
            mgx(GoalX.NAV_DYNAMIC_LINE_REVERSE, pp, -1, speed=ul, enable_sensor=False),
        ])

        paths, pp = fpr(home, s2)
        eq(rto('S2'), [  # Home -> S2
            mgx(GoalX.NAV_DYNAMIC_LINE_FORWARD, pp[0], -1, speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[1][0], ish(paths[1]))),
            mgx(GoalX.NAV_REVERSE, pp[1], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[2][0], sh(paths[2]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[2], speed=ul),
        ])
        paths, pp = fpr((s2[0], 90.0), home)
        eq(artob((s2[0], 90.0), 'Home'), [  # S2 -> Home
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[0], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], ish(paths[1]))),
            mgx(GoalX.NAV_REVERSE, pp[1], speed=ul, next_motion=R),
            mgx(GoalX.NAV_DYNAMIC_LINE_REVERSE, pp[2], -1, speed=ul, enable_sensor=False),
        ])

        paths, pp = fpr(home, s4)
        eq(rto('S4'), [  # Home -> S4
            mgx(GoalX.NAV_DYNAMIC_LINE_FORWARD, pp[0], -1, speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[1][0], ish(paths[1]))),
            mgx(GoalX.NAV_REVERSE, pp[1], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[2][0], sh(paths[2]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[2], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[3][0], ish(paths[3]))),
            mgx(GoalX.NAV_REVERSE, pp[3:5], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[4:5])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[4], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(ieh(paths[4]) - ish(paths[5])), rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul),
            mgx(GoalX.NAV_REVERSE, pp[6:8], speed=mf, next_motion=NS, next_speed=ul, next_distance=sd(paths[7:8])),
            mgx(GoalX.NAV_REVERSE, pp[7], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s4)
        ])
        paths, pp = fpr(s4, home)
        eq(rto('Home'), [  # S4 -> Home
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[0][0], ish(paths[0]))),
            mgx(GoalX.NAV_REVERSE, pp[0:2], speed=ul, next_motion=NS, next_speed=mf, next_distance=sd(paths[1:2])),
            mgx(GoalX.NAV_REVERSE, pp[1], speed=mf),
            mg(Goal.NAV_SEARCH_LINE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[3:5], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[4:5])),
            mgx(GoalX.NAV_REVERSE, pp[4], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[5][0], sh(paths[5]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[5], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[6][0], ish(paths[6]))),
            mgx(GoalX.NAV_REVERSE, pp[6], speed=ul, next_motion=R),
            mgx(GoalX.NAV_DYNAMIC_LINE_REVERSE, pp[7], -1, speed=ul, enable_sensor=False),
        ])

        eq(rto('T1'), [])  # Home -> T1
        eq(rto('Home'), [])  # T1 -> Home

        paths, pp = fpr(home, t2)
        eq(rto('T2', next_motion=NSB, next_speed=0.2), [  # Home -> T2
            # mgx(),  # teleport
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[1], speed=sl, next_motion=NSB, next_speed=0.2),
        ])
        paths, pp = fpr(t2, home)
        eq(rto('Home', next_motion=NS), [  # T2 -> Home
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[0], speed=sl, next_motion=NS, next_speed=ul, next_distance=sd(paths[1:2])),
            # mgx(),  # teleport
        ])

        paths, pp = fpr(home, u1)
        eq(rto('U1', next_motion=NS), [  # Home -> U1
            mgx(GoalX.NAV_DYNAMIC_LINE_FORWARD, pp[0], -1, speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[1][0], ish(paths[1]))),
            mgx(GoalX.NAV_REVERSE, pp[1], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[2][0], sh(paths[2]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[2], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[3][0], ish(paths[3]))),
            mgx(GoalX.NAV_REVERSE, pp[3:5], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[4:5])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[4], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(ieh(paths[4]) - ish(paths[5])), rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul),
            mgx(GoalX.NAV_REVERSE, pp[6:8], speed=mf, next_motion=NS, next_speed=ul, next_distance=sd(paths[7:8])),
            mgx(GoalX.NAV_REVERSE, pp[7], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[8][0], ish(paths[8]))),
            mgx(GoalX.NAV_REVERSE, pp[8], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s5),
            mgx(GoalX.NAV_FORWARD, path_start=Pose2D(**s5j), path_end=Pose2D(x=s5j['x'], y=s5j['y'] + 2), speed=0.4),  # teleport
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[10][0], ish(paths[10]))),
            mgx(GoalX.NAV_REVERSE, pp[10], speed=md, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[11][0], ish(paths[11]))),
            mgx(GoalX.NAV_REVERSE, pp[11:15], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[12:15])),
            mgx(GoalX.NAV_REVERSE, pp[12:15], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[13:15])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[13:15], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[14:15])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[14], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[15][0], sh(paths[15]))),
            mgx(GoalX.NAV_FORWARD, pp[15], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[16][0], ish(paths[16]))),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[16], speed=ul, next_motion=L),
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(360 - ieh(paths[16]))),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=u1),
        ])
        paths, pp = fpr(u1, home)
        eq(rto('Home'), [  # U1 -> Home
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(90)),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[2][0], ish(paths[2]))),
            mgx(GoalX.NAV_REVERSE, pp[2:7], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[3:7])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[3:7], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[4:7])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[4:7], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[5:7])),
            mgx(GoalX.NAV_REVERSE, pp[5:7], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[6:7])),
            mgx(GoalX.NAV_REVERSE, pp[6], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(180), rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=0.4),  # teleport
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[8][0], ish(paths[8]))),
            mgx(GoalX.NAV_REVERSE, pp[8:10], speed=ul, next_motion=NS, next_speed=mf, next_distance=sd(paths[9:10])),
            mgx(GoalX.NAV_REVERSE, pp[9], speed=mf),
            mg(Goal.NAV_SEARCH_LINE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[11:13], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[12:13])),
            mgx(GoalX.NAV_REVERSE, pp[12], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[13][0], sh(paths[13]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[13], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[14][0], ish(paths[14]))),
            mgx(GoalX.NAV_REVERSE, pp[14], speed=ul, next_motion=R),
            mgx(GoalX.NAV_DYNAMIC_LINE_REVERSE, pp[15], -1, speed=ul, enable_sensor=False),
        ])

        paths, pp = fpr(s1, u3)
        eq(artob('S1', 'U3'), [  # S1 -> U3
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[0][0], ish(paths[0]))),
            mgx(GoalX.NAV_REVERSE, pp[0], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[2][0], ish(paths[2]))),
            mgx(GoalX.NAV_REVERSE, pp[2:4], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[3:4])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[3], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(ieh(paths[3]) - ish(paths[4])), rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul),
            mgx(GoalX.NAV_REVERSE, pp[5:7], speed=mf, next_motion=NS, next_speed=ul, next_distance=sd(paths[6:7])),
            mgx(GoalX.NAV_REVERSE, pp[6], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[7][0], ish(paths[7]))),
            mgx(GoalX.NAV_REVERSE, pp[7], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s5),
            mgx(GoalX.NAV_FORWARD, path_start=Pose2D(**s5j), path_end=Pose2D(x=s5j['x'], y=s5j['y'] + 2), speed=0.4),  # teleport
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[9][0], ish(paths[9]))),
            mgx(GoalX.NAV_REVERSE, pp[9], speed=md, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[10][0], ish(paths[10]))),
            mgx(GoalX.NAV_REVERSE, pp[10:12], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[11:12])),
            mgx(GoalX.NAV_REVERSE, pp[11], speed=ul),
        ])
        paths, pp = fpr((u3[0], 185.0), s1)
        eq(artob((u3[0], 185.0), 'S1'), [  # U3 -> S1
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[0][0], ish(paths[0]))),
            mgx(GoalX.NAV_REVERSE, pp[0:2], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[1:2])),
            mgx(GoalX.NAV_REVERSE, pp[1], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(180), rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=0.4),  # teleport
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[3][0], ish(paths[3]))),
            mgx(GoalX.NAV_REVERSE, pp[3:5], speed=ul, next_motion=NS, next_speed=mf, next_distance=sd(paths[4:5])),
            mgx(GoalX.NAV_REVERSE, pp[4], speed=mf),
            mg(Goal.NAV_SEARCH_LINE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[6:8], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[7:8])),
            mgx(GoalX.NAV_REVERSE, pp[7], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[8][0], sh(paths[8]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[8], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[9][0], ish(paths[9]))),
            mgx(GoalX.NAV_REVERSE, pp[9], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=s1),
        ])

        paths, pp = fpr(s4, s5)
        eq(artob('S4', 'S5'), [  # S4 -> S5
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[0][0], ish(paths[0]))),
            mgx(GoalX.NAV_REVERSE, pp[0], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s5),
        ])
        paths, pp = fpr(s5, s4)
        eq(rto('S4'), [  # S5 -> S4
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[0][0], ish(paths[0]))),
            mgx(GoalX.NAV_REVERSE, pp[0], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s4),
        ])

        paths, pp = fpr(t2, u0)
        eq(artob('T2', 'U0', next_motion=NS, next_speed=0.6), [  # T2 -> U0
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[0], speed=sl, next_motion=NS, next_speed=ul, next_distance=sd(paths[1:3])),
            # mgx(),  # teleport
            mgx(GoalX.NAV_DYNAMIC_LINE_FORWARD, pp[2], -1, speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[3][0], ish(paths[3]))),
            mgx(GoalX.NAV_REVERSE, pp[3], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[4][0], sh(paths[4]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[4], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[5][0], ish(paths[5]))),
            mgx(GoalX.NAV_REVERSE, pp[5:7], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[6:7])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[6], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(ieh(paths[6]) - ish(paths[7])), rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul),
            mgx(GoalX.NAV_REVERSE, pp[8:10], speed=mf, next_motion=NS, next_speed=ul, next_distance=sd(paths[9:10])),
            mgx(GoalX.NAV_REVERSE, pp[9], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[10][0], ish(paths[10]))),
            mgx(GoalX.NAV_REVERSE, pp[10], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s5),
            mgx(GoalX.NAV_FORWARD, path_start=Pose2D(**s5j), path_end=Pose2D(x=s5j['x'], y=s5j['y'] + 2), speed=0.4),  # teleport
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[12][0], ish(paths[12]))),
            mgx(GoalX.NAV_REVERSE, pp[12], speed=md, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[13][0], ish(paths[13]))),
            mgx(GoalX.NAV_REVERSE, pp[13:17], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[14:17])),
            mgx(GoalX.NAV_REVERSE, pp[14:17], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[15:17])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[15:17], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[16:17])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[16], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[17][0], sh(paths[17]))),
            mgx(GoalX.NAV_FORWARD, pp[17], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[18][0], ish(paths[18]))),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[18], speed=ul, next_motion=L),
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(360 - ieh(paths[18]))),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=NS, next_speed=0.6),
        ])
        paths, pp = fpr(u0, t2)
        eq(rto('T2', align_station_type=1), [  # U0 -> T2
            mg(Goal.NAV_UTURN_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=ul, next_motion=R),
            mg(Goal.NAV_ROTATE_RIGHT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[2][0], sh(paths[2]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[2], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[3][0], ish(paths[3]))),
            mgx(GoalX.NAV_REVERSE, pp[3:8], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[4:8])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[4:8], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[5:8])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[5:8], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[6:8])),
            mgx(GoalX.NAV_REVERSE, pp[6:8], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[7:8])),
            mgx(GoalX.NAV_REVERSE, pp[7], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(180), rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=0.4),  # teleport
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[9][0], ish(paths[9]))),
            mgx(GoalX.NAV_REVERSE, pp[9:11], speed=ul, next_motion=NS, next_speed=mf, next_distance=sd(paths[10:11])),
            mgx(GoalX.NAV_REVERSE, pp[10], speed=mf),
            mg(Goal.NAV_SEARCH_LINE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[12:14], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[13:14])),
            mgx(GoalX.NAV_REVERSE, pp[13], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[14][0], sh(paths[14]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[14], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[15][0], ish(paths[15]))),
            mgx(GoalX.NAV_REVERSE, pp[15], speed=ul, next_motion=R),
            mgx(GoalX.NAV_DYNAMIC_LINE_REVERSE, pp[16], -1, speed=ul),
            # mgx(),  # teleport
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[18], speed=sl),
            mg(Goal.NAV_SEARCH_LINE_LEFT, rotate_align_sensor=1),
        ])

        paths, pp = fpr(u1, u2)
        eq(artob('U1', 'U2', next_motion=R, next_speed=0.4), [  # U1 -> U2
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1], speed=ul, next_motion=R),
        ])
        paths, pp = fpr((u2[0], 179.0), u1)
        eq(artob((u2[0], 179.0), 'U1', align_station_type=0), [  # U2 -> U1
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[0], speed=ul, next_motion=L),
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(360 - ieh(paths[0]))),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
        ])

        paths, pp = fpr(u4, u5)
        eq(artob('U4', 'U5'), [  # U4 -> U5
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[0][0], ish(paths[0]))),
            mgx(GoalX.NAV_REVERSE, pp[0], speed=md, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=u5),
        ])
        paths, pp = fpr(u5, u4)
        eq(rto('U4'), [  # U5 -> U4
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[0][0], ish(paths[0]))),
            mgx(GoalX.NAV_REVERSE, pp[0], speed=md, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=u4),
        ])

        # with payload (trolley), avoid no-rotate zones
        base.set_dimension_profile(1)

        paths, pp = fp(home, t2)
        eq(atob('Home', 'T2'), [  # Home -> T2
            # mgx(),  # teleport
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[1], speed=sl),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=t2),
        ])
        paths, pp = fp(t2, home)
        eq(to('Home'), [  # T2 -> Home
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[0], speed=sl, next_motion=NS, next_speed=ul, next_distance=sd(paths[1:2])),
            # mgx(),  # teleport
        ])

        paths, pp = fp(s1, (s3[0], 90.0))
        eq(atob('S1', 'S3a'), [  # S1 -> S3
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_FORWARD, pp[0], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1:4], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[2:4])),
            mgx(GoalX.NAV_FORWARD, pp[2:4], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[3:4])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[3], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(eh(paths[3]) - sh(paths[4]))),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
        ])
        paths, pp = fp((s3[0], 90.0), s1)
        eq(to('S1'), [  # S3 -> S1
            mgx(GoalX.NAV_FORWARD, pp[0], speed=mf),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_FORWARD, pp[1], speed=mf),
            mg(Goal.NAV_SEARCH_LINE_LEFT),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[3:6], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[4:6])),
            mgx(GoalX.NAV_FORWARD, pp[4:6], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[5:6])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[5], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[6][0], sh(paths[6]))),
            mgx(GoalX.NAV_FORWARD, pp[6], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s1),
        ])

        paths, pp = fp(s4, u3)
        eq(atob('S4', 'U3'), [  # S4 -> U3
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_FORWARD, pp[0], speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s5),
            mgx(GoalX.NAV_FORWARD, path_start=Pose2D(**s5j), path_end=Pose2D(x=s5j['x'], y=s5j['y'] + 2), speed=0.4),  # teleport
            mgx(GoalX.NAV_FORWARD, pp[2], speed=md, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[3][0], sh(paths[3]))),
            mgx(GoalX.NAV_FORWARD, pp[3:5], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[4:5])),
            mgx(GoalX.NAV_FORWARD, pp[4], speed=ul),
        ])
        paths, pp = fp((u3[0], 185.0), s4)
        eq(atob((u3[0], 185.0), 'S4'), [  # U3 -> S4
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_FORWARD, pp[0:2], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[1:2])),
            mgx(GoalX.NAV_FORWARD, pp[1], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=0.4),  # teleport
        ])

        paths, pp = fp(u1, u2)
        eq(atob('U1', 'U2'), [  # U1 -> U2
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1], speed=ul),
        ])
        paths, pp = fp((u2[0], 179.0), u1)
        eq(atob((u2[0], 179.0), 'U1', align_station_type=0), [  # U2 -> U1
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[0], speed=ul, next_motion=L),
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(360 - ieh(paths[0]))),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
        ])

        paths, pp = fpr(home, t2)
        eq(artob('Home', 'T2', next_motion=NSB, next_speed=0.2), [  # Home -> T2
            # mgx(),  # teleport
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[1], speed=sl, next_motion=NSB, next_speed=0.2),
        ])
        paths, pp = fpr(t2, home)
        eq(rto('Home', next_motion=NS), [  # T2 -> Home
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[0], speed=sl, next_motion=NS, next_speed=ul, next_distance=sd(paths[1:2])),
            # mgx(),  # teleport
        ])

        paths, pp = fpr(s1, (s3[0], 90.0))
        eq(artob('S1', 'S3a'), [  # S1 -> S3
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[0][0], ish(paths[0]))),
            mgx(GoalX.NAV_REVERSE, pp[0], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1], speed=ul),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[2][0], ish(paths[2]))),
            mgx(GoalX.NAV_REVERSE, pp[2:4], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[3:4])),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[3], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(ieh(paths[3]) - ish(paths[4])), rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul),
            mgx(GoalX.NAV_REVERSE, pp[5], speed=mf),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[6][0], ish(paths[6]))),
            mgx(GoalX.NAV_REVERSE, pp[6], speed=mf),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(s3[0], 90.0)),
        ])
        paths, pp = fpr((s3[0], 90.0), s1)
        eq(rto('S1'), [  # S3 -> S1
            mg(Goal.NAV_SEARCH_LINE_LEFT, rotate_align_sensor=1),
            mg(Goal.NAV_BEZIER_REVERSE, speed=ul),
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[1:3], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[2:3])),
            mgx(GoalX.NAV_REVERSE, pp[2], speed=ul),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[3][0], sh(paths[3]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[3], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[4][0], ish(paths[4]))),
            mgx(GoalX.NAV_REVERSE, pp[4], speed=ul, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=s1),
        ])

        paths, pp = fpr(s4, u3)
        eq(artob('S4', 'U3'), [  # S4 -> U3
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[0][0], ish(paths[0]))),
            mgx(GoalX.NAV_REVERSE, pp[0], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s5),
            mgx(GoalX.NAV_FORWARD, path_start=Pose2D(**s5j), path_end=Pose2D(x=s5j['x'], y=s5j['y'] + 2), speed=0.4),  # teleport
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[2][0], ish(paths[2]))),
            mgx(GoalX.NAV_REVERSE, pp[2], speed=md, next_motion=R),
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[3][0], ish(paths[3]))),
            mgx(GoalX.NAV_REVERSE, pp[3:5], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[4:5])),
            mgx(GoalX.NAV_REVERSE, pp[4], speed=ul),
        ])
        paths, pp = fpr((u3[0], 185.0), s4)
        eq(artob((u3[0], 185.0), 'S4'), [  # U3 -> S4
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[0][0], ish(paths[0]))),
            mgx(GoalX.NAV_REVERSE, pp[0:2], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[1:2])),
            mgx(GoalX.NAV_REVERSE, pp[1], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(180), rotate_align_sensor=1),
            mg(Goal.NAV_REVERSE, speed=0.4),  # teleport
        ])

        paths, pp = fpr(u1, u2)
        eq(artob('U1', 'U2', next_motion=R, next_speed=0.4), [  # U1 -> U2
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1], speed=ul, next_motion=R),
        ])
        paths, pp = fpr((u2[0], 179.0), u1)
        eq(artob((u2[0], 179.0), 'U1', align_station_type=0), [  # U2 -> U1
            mgx(GoalX.NAV_BEZIER_REVERSE, pp[0], speed=ul, next_motion=L),
            mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(360 - ieh(paths[0]))),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mg(Goal.NAV_ROTATE_LEFT),
        ])

    def test_navigate_to2(self, nonstop_teleports=True):
        '''
        trailer (no reverse and no u-turn)
        '''
        home = self.home
        s1, s2, s3, s4, s5, t1, t2, u0, u1, u2, u3, u4, u5 = self.locations

        robot = self.robot2
        base = robot.base
        models = robot.models

        ih = models.invert_heading
        sh = models.compute_edge_start_heading
        eh = models.compute_edge_end_heading
        ish = (lambda e: ih(sh(e)))
        ieh = (lambda e: ih(eh(e)))
        gj = models.get_junction

        ul = NavigateToX.UNLIMITED_SPEED
        sl, md, mf = 0.4, 0.5, 0.6  # slow, medium, medium fast
        NS, L, R, NSB = GoalX.MOTION_NONSTOP, GoalX.MOTION_LEFT, GoalX.MOTION_RIGHT, GoalX.MOTION_NONSTOP_BEZIER
        eq, err = self.expectEqual, self.expectRaisesRegexp
        invalid_path_msg = 'Unable to find a valid path to destination.'
        reverse_not_allowed_msg = '"Reverse" motion is not allowed.'

        base.set_initial_map_and_location(models.graph, home)
        base.granularity = self.granularity
        base.speedup_factor = self.speedup_factor

        def reset(station):
            if isinstance(station, tuple):
                base.set_initial_map_and_location(models.graph, station)
            else:
                ResetAgvPositionX(robot, station=station).execute({})

        def to(station, **params):
            base.start_recording_goals()
            d = {
                'station': station,
                'align_station_type': -1,
                'next_motion': 0,
                'next_speed': 0,
                'sense_line': 0,
            }
            d.update(params)
            NavigateToX(robot, **d).execute({})
            return base.stop_recording_goals()

        def rto(station, **params):
            base.start_recording_goals()
            d = {
                'station': station,
                'align_station_type': -1,
                'next_motion': 0,
                'next_speed': 0,
                'sense_line': 0,
            }
            d.update(params)
            ReverseNavigateToX(robot, **d).execute({})
            return base.stop_recording_goals()

        def atob(a, b, **params):
            reset(a)
            return to(b, **params)

        def artob(a, b, **params):
            reset(a)
            return rto(b, **params)

        def fp(a, b):
            paths = []
            plan = iter(models.find_shortest_constrained_path(a, b)[0])
            prev_j = next(plan)
            for j in plan:
                e = models.get_path(prev_j, j)
                paths.append((prev_j, j, e))
                prev_j = j
            return paths, poly(paths)

        def fpr(a, b):
            paths = []
            plan = iter(models.find_shortest_constrained_path(a, b, reverse=True)[0])
            prev_j = next(plan)
            for j in plan:
                e = models.get_path(prev_j, j)
                paths.append((prev_j, j, e))
                prev_j = j
            return paths, poly(paths)

        def poly(paths):
            return [Polygon(points=(
                [
                    Point32(**gj(e[0])),
                    Point32(**gj(e[1])),
                ]
                if e[2]['shape'] == PathShape.STRAIGHT else
                [
                    Point32(**gj(e[0])),
                    Point32(**e[2]['cp1']),
                    Point32(**e[2]['cp2']),
                    Point32(**gj(e[1])),
                ])
                if not models.is_teleport(e[2]) else []
            ) for e in paths]

        def sd(paths):  # sum of distances
            return sum(e[2]['distance'] for e in paths)

        def mg(nav, **kwargs):  # make goal
            d = {
                'nav': nav,
                'enable_sensor': True,
            }
            d.update(**kwargs)
            return Goal(**d)

        def mgx(nav, paths=None, theta=0, loc=None, **kwargs):  # make goalx
            d = {
                'nav': nav,
                'enable_sensor': True,
            }
            if paths:  # straight
                if not isinstance(paths, list):
                    paths = [paths]
                path = paths[0].points
                d['goal_tolerance'] = GoalX.TOLERANCE_GOAL
                d['path_start'] = Pose2D(x=path[0].x, y=path[0].y)
                d['path_end'] = Pose2D(x=path[-1].x, y=path[-1].y, theta=theta)
                if len(path) == 4:
                    d['path_cp1'] = Pose2D(x=path[1].x, y=path[1].y)
                    d['path_cp2'] = Pose2D(x=path[2].x, y=path[2].y)
                d['paths'] = Path(paths=paths)

            elif loc:  # rotate
                j, h = loc
                d['path_end'] = Pose2D(theta=math.radians(h), **gj(j))

            d.update(**kwargs)
            return GoalX(**d)

        # same spot
        eq(atob('Home', 'Home'), [])
        eq(artob('S4', 'S4'), [])

        # traversals
        paths, pp = fp(home, s1)
        eq(atob('Home', 'S1'), [  # Home -> S1
            mgx(GoalX.NAV_FORWARD, pp, speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s1),
        ])
        err(UserError, invalid_path_msg, to, 'Home')  # S1 -> Home

        paths, pp = fp(home, s2)
        eq(atob('Home', 'S2'), [  # Home -> S2
            mgx(GoalX.NAV_FORWARD, pp[0], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_FORWARD, pp[1], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[2][0], sh(paths[2]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[2], speed=ul),
        ])
        err(UserError, invalid_path_msg, atob, (s2[0], 90.0), 'Home')  # S2 -> Home

        if nonstop_teleports:
            paths, pp = fp(home, s4)
            s5j = gj(s5[0])
            err(UserError, reverse_not_allowed_msg, atob, 'Home', 'S4')  # Home -> S4
            eq(base.stop_recording_goals(), [
                mgx(GoalX.NAV_FORWARD, pp[0], speed=ul, next_motion=L),
                mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
                mgx(GoalX.NAV_FORWARD, pp[1], speed=ul, next_motion=L),
                mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[2][0], sh(paths[2]))),
                mgx(GoalX.NAV_BEZIER_FORWARD, pp[2:5], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[3:5])),
                mgx(GoalX.NAV_FORWARD, pp[3:5], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[4:5])),
                mgx(GoalX.NAV_BEZIER_FORWARD, pp[4], speed=ul),
                mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(eh(paths[4]) - sh(paths[5]))),
                mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
                mgx(GoalX.NAV_FORWARD, pp[6:8], speed=mf, next_motion=NS, next_speed=ul, next_distance=sd(paths[7:8])),
                mgx(GoalX.NAV_FORWARD, pp[7], speed=ul, next_motion=L),
                mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[8][0], sh(paths[8]))),
                mgx(GoalX.NAV_FORWARD, pp[8], speed=ul, next_motion=R),
                mgx(GoalX.NAV_ROTATE_RIGHT, loc=s5),
                mgx(GoalX.NAV_FORWARD, path_start=Pose2D(**s5j), path_end=Pose2D(x=s5j['x'], y=s5j['y'] + 2), speed=0.4),  # teleport
                mgx(GoalX.NAV_FORWARD, pp[10], speed=md, next_motion=L),
                mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(90), rotate_align_sensor=1),
                # mg(Goal.NAV_REVERSE, speed=0.4),  # teleport: error: reverse not allowed
            ])
        else:
            err(UserError, invalid_path_msg, atob, 'Home', 'S4')  # Home -> S4
        err(UserError, invalid_path_msg, atob, 'S4', 'Home')  # S4 -> Home

        eq(atob('Home', 'T1'), [])  # Home -> T1
        eq(to('Home'), [])  # T1 -> Home

        err(UserError, invalid_path_msg, to, 'T2')  # Home -> T2
        paths, pp = fp(t2, home)
        nmsd_teleport_t1_home = dict(next_motion=NS, next_speed=ul, next_distance=sd(paths[1:2])) if nonstop_teleports else {}
        eq(atob('T2', 'Home'), [  # T2 -> Home
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[0], speed=sl, **nmsd_teleport_t1_home),
            # mgx(),  # teleport
        ])

        err(UserError, invalid_path_msg, to, 'U1')  # Home -> U1
        err(UserError, invalid_path_msg, atob, 'U1', 'Home')  # U1 -> Home

        if nonstop_teleports:
            paths, pp = fp(s1, u3)
            eq(atob('S1', 'U3'), [  # S1 -> U3
                mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[0][0], sh(paths[0]))),
                mgx(GoalX.NAV_FORWARD, pp[0], speed=ul, next_motion=L),
                mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
                mgx(GoalX.NAV_BEZIER_FORWARD, pp[1:4], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[2:4])),
                mgx(GoalX.NAV_FORWARD, pp[2:4], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[3:4])),
                mgx(GoalX.NAV_BEZIER_FORWARD, pp[3], speed=ul),
                mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(eh(paths[3]) - sh(paths[4]))),
                mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
                mgx(GoalX.NAV_FORWARD, pp[5:7], speed=mf, next_motion=NS, next_speed=ul, next_distance=sd(paths[6:7])),
                mgx(GoalX.NAV_FORWARD, pp[6], speed=ul, next_motion=L),
                mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[7][0], sh(paths[7]))),
                mgx(GoalX.NAV_FORWARD, pp[7], speed=ul, next_motion=R),
                mgx(GoalX.NAV_ROTATE_RIGHT, loc=s5),
                mgx(GoalX.NAV_FORWARD, path_start=Pose2D(**s5j), path_end=Pose2D(x=s5j['x'], y=s5j['y'] + 2), speed=0.4),  # teleport
                mgx(GoalX.NAV_FORWARD, pp[9], speed=md, next_motion=R),
                mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[10][0], sh(paths[10]))),
                mgx(GoalX.NAV_FORWARD, pp[10:12], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[11:12])),
                mgx(GoalX.NAV_FORWARD, pp[11], speed=ul),
            ])
        else:
            err(UserError, invalid_path_msg, atob, 'S1', 'U3')  # S1 -> U3
        err(UserError, invalid_path_msg, atob, (u3[0], 185.0), 'S1')  # U3 -> S1

        err(UserError, invalid_path_msg, atob, 'S4', 'S5')  # S4 -> S5
        if nonstop_teleports:
            paths, pp = fp(s5, s4)
            err(UserError, reverse_not_allowed_msg, atob, 'S5', 'S4')  # S5 -> S4
            eq(base.stop_recording_goals(), [
                mgx(GoalX.NAV_FORWARD, path_start=Pose2D(**s5j), path_end=Pose2D(x=s5j['x'], y=s5j['y'] + 2), speed=0.4),  # teleport
                mgx(GoalX.NAV_FORWARD, pp[1], speed=md, next_motion=L),
                mg(Goal.NAV_SEARCH_LINE_LEFT, distance=math.radians(90), rotate_align_sensor=1),
                # mg(Goal.NAV_REVERSE, speed=0.4),  # teleport: error: reverse not allowed
            ])
        else:
            err(UserError, invalid_path_msg, atob, 'S5', 'S4')  # S5 -> S4

        err(UserError, invalid_path_msg, atob, 'T2', 'U0')  # T2 -> U0
        err(UserError, invalid_path_msg, atob, 'U0', 'T2')  # U0 -> T2

        paths, pp = fp(u1, u2)
        eq(atob('U1', 'U2'), [  # U1 -> U2
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1], speed=ul),
        ])
        err(UserError, invalid_path_msg, atob, (u2[0], 179.0), 'U1')  # U2 -> U1

        err(UserError, invalid_path_msg, atob, 'U4', 'U5')  # U4 -> U5
        err(UserError, invalid_path_msg, atob, 'U5', 'U4')  # U5 -> U4

        # reverse traversals
        paths, pp = fpr(home, s1)
        eq(atob('Home', 'S1'), [  # Home -> S1
            mgx(GoalX.NAV_FORWARD, pp, speed=ul),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=s1),
        ])
        err(UserError, invalid_path_msg, rto, 'Home')  # S1 -> Home

        err(UserError, invalid_path_msg, artob, 'Home', 'S2')  # Home -> S2
        err(UserError, invalid_path_msg, artob, (s2[0], 90.0), 'Home')  # S2 -> Home

        err(UserError, invalid_path_msg, artob, 'Home', 'S4')  # Home -> S4
        err(UserError, invalid_path_msg, artob, 'S4', 'Home')  # S4 -> Home

        eq(artob('Home', 'T1'), [])  # Home -> T1
        eq(rto('Home'), [])  # T1 -> Home

        err(UserError, invalid_path_msg, rto, 'T2')  # Home -> T2
        paths, pp = fpr(t2, home)
        eq(artob('T2', 'Home'), [  # T2 -> Home
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[0], speed=sl, **nmsd_teleport_t1_home),
            # mgx(),  # teleport
        ])

        err(UserError, invalid_path_msg, rto, 'U1')  # Home -> U1
        err(UserError, invalid_path_msg, artob, 'U1', 'Home')  # U1 -> Home

        err(UserError, invalid_path_msg, artob, 'S1', 'U3')  # S1 -> U3
        err(UserError, invalid_path_msg, artob, (u3[0], 185.0), 'S1')  # U3 -> S1

        err(UserError, invalid_path_msg, artob, 'S4', 'S5')  # S4 -> S5
        err(UserError, invalid_path_msg, artob, 'S5', 'S4')  # S5 -> S4

        err(UserError, invalid_path_msg, artob, 'T2', 'U0')  # T2 -> U0
        err(UserError, invalid_path_msg, artob, 'U0', 'T2')  # U0 -> T2

        paths, pp = fpr(u1, u2)
        eq(artob('U1', 'U2'), [  # U1 -> U2
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1], speed=ul),
        ])
        err(UserError, invalid_path_msg, artob, (u2[0], 179.0), 'U1')  # U2 -> U1

        err(UserError, invalid_path_msg, artob, 'U4', 'U5')  # U4 -> U5
        err(UserError, invalid_path_msg, artob, 'U5', 'U4')  # U5 -> U4

        # with payload (trolley), avoid no-rotate zones
        base.set_dimension_profile(1)

        err(UserError, invalid_path_msg, atob, 'Home', 'T2')  # Home -> T2
        paths, pp = fp(t2, home)
        nmsd_teleport_t1_home = dict(next_motion=NS, next_speed=ul, next_distance=sd(paths[1:2])) if nonstop_teleports else {}
        eq(atob('T2', 'Home'), [  # T2 -> Home
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[0], speed=sl, **nmsd_teleport_t1_home),
            # mgx(),  # teleport
        ])

        paths, pp = fp(s1, (s3[0], 90.0))
        eq(atob('S1', 'S3a'), [  # S1 -> S3
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_FORWARD, pp[0], speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1:4], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[2:4])),
            mgx(GoalX.NAV_FORWARD, pp[2:4], speed=ul, next_motion=NSB, next_speed=ul, next_distance=sd(paths[3:4])),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[3], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_RIGHT, distance=math.radians(eh(paths[3]) - sh(paths[4]))),
            mg(Goal.NAV_BEZIER_FORWARD, speed=ul),
        ])
        err(UserError, invalid_path_msg, to, 'S1')  # S3 -> S1

        err(UserError, invalid_path_msg, atob, 'S4', 'U3')  # S4 -> U3
        paths, pp = fp((u3[0], 185.0), s4)
        err(UserError, reverse_not_allowed_msg, atob, (u3[0], 185.0), 'S4')  # U3 -> S4
        eq(base.stop_recording_goals(), [
            mgx(GoalX.NAV_ROTATE_RIGHT, loc=(paths[0][0], sh(paths[0]))),
            mgx(GoalX.NAV_FORWARD, pp[0:2], speed=ul, next_motion=NS, next_speed=ul, next_distance=sd(paths[1:2])),
            mgx(GoalX.NAV_FORWARD, pp[1], speed=ul),
            mg(Goal.NAV_SEARCH_LINE_LEFT, rotate_align_sensor=1),
            # mg(Goal.NAV_REVERSE, speed=0.4),  # teleport: error: reverse not allowed
        ])

        paths, pp = fp(u1, u2)
        eq(atob('U1', 'U2'), [  # U1 -> U2
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1], speed=ul),
        ])
        err(UserError, invalid_path_msg, atob, (u2[0], 179.0), 'U1')  # U2 -> U1

        err(UserError, invalid_path_msg, artob, 'Home', 'T2')  # Home -> T2
        paths, pp = fpr(t2, home)
        eq(artob('T2', 'Home'), [  # T2 -> Home
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[0], speed=sl, **nmsd_teleport_t1_home),
            # mgx(),  # teleport
        ])

        err(UserError, invalid_path_msg, artob, 'S1', 'S3a')  # S1 -> S3
        err(UserError, invalid_path_msg, artob, 'S3a', 'S1')  # S3 -> S1

        err(UserError, invalid_path_msg, artob, 'S4', 'U3')  # S4 -> U3
        err(UserError, invalid_path_msg, artob, (u3[0], 185.0), 'S4')  # U3 -> S4

        paths, pp = fpr(u1, u2)
        eq(artob('U1', 'U2'), [  # U1 -> U2
            mg(Goal.NAV_ROTATE_LEFT),
            mg(Goal.NAV_FORWARD, speed=ul, next_motion=L),
            mgx(GoalX.NAV_ROTATE_LEFT, loc=(paths[1][0], sh(paths[1]))),
            mgx(GoalX.NAV_BEZIER_FORWARD, pp[1], speed=ul),
        ])
        err(UserError, invalid_path_msg, artob, (u2[0], 179.0), 'U1')  # U2 -> U1

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


if __name__ == '__main__':
    rospy.init_node('test_plan_x', anonymous=True)
    rostest.rosrun('agv05_skills', 'test_plan_x', TestPlanX, sys.argv)
