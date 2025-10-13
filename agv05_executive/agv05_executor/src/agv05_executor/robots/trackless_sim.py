from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.systemx.models import PathShape
from geometry_msgs.msg import Pose2D
import agv05_msgs.msg
import math
import rospy
import tf

from .base import TracklessBase
from .robot import TracklessMixin
from .sim import Sim
from .tracked_sim import TrackedSimBase
from ..models.map_tracker_x import MapTrackerX

Goal = type(str('Goal'), (agv05_msgs.msg.NavxActionGoal, ), {})  # overcome fixed __slots__


class TracklessSimBase(TrackedSimBase, TracklessBase):
    manual_cmd_vel_topic = '/cmd_vel'
    action_server = '/navx_action'
    action_spec = agv05_msgs.msg.NavxActionAction
    map_tracker_class = MapTrackerX

    forward_str_fmt = '[(%.2f, %.2f) -> (%.2f, %.2f); NM=%d]'
    rotate_str_fmt = '[(%.2f, %.2f, %.1f deg)]'

    def __init__(self, robot):
        super(TracklessSimBase, self).__init__(robot)
        self.tf_broadcaster = tf.TransformBroadcaster()

    def dynamic_waypoint_forward(self, end, heading, constraints):
        rospy.loginfo('TracklessSimBase is dynamically moving forward to waypoint... ' + self.rotate_str_fmt,
            end['x'], end['y'], heading)
        params = {
            'nav': Goal.NAV_DYNAMIC_FORWARD,
            'goal_tolerance': Goal.TOLERANCE_GOAL,
            'path_end': Pose2D(theta=math.radians(heading), **end),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        # TODO: map_tracker

    def dynamic_waypoint_reverse(self, end, heading, constraints):
        rospy.loginfo('TracklessSimBase is dynamically moving reverse to waypoint... ' + self.rotate_str_fmt,
            end['x'], end['y'], heading)
        params = {
            'nav': Goal.NAV_DYNAMIC_REVERSE,
            'goal_tolerance': Goal.TOLERANCE_GOAL,
            'path_end': Pose2D(theta=math.radians(heading), **end),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        # TODO: map_tracker

    def dynamic_line_forward(self, start, end, constraints):
        rospy.loginfo('TracklessSimBase is dynamically moving forward... ' + self.forward_str_fmt,
            start['x'], start['y'], end['x'], end['y'], constraints['next_motion'])
        params = {
            'nav': Goal.NAV_DYNAMIC_LINE_FORWARD,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(x=end['x'], y=end['y'], theta=end.get('theta', -1.0)),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        self.map_tracker.forward(hint=end)

    def dynamic_line_reverse(self, start, end, constraints):
        rospy.loginfo('TracklessSimBase is dynamically moving reverse... ' + self.forward_str_fmt,
            start['x'], start['y'], end['x'], end['y'], constraints['next_motion'])
        params = {
            'nav': Goal.NAV_DYNAMIC_LINE_REVERSE,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(x=end['x'], y=end['y'], theta=end.get('theta', -1.0)),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        self.map_tracker.reverse(hint=end)

    def dynamic_bezier_forward(self, start, end, cp1, cp2, constraints):
        rospy.loginfo('TracklessSimBase is dynamically beziering forward... ' + self.forward_str_fmt,
            start['x'], start['y'], end['x'], end['y'], constraints['next_motion'])
        params = {
            'nav': Goal.NAV_DYNAMIC_BEZIER_FORWARD,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(x=end['x'], y=end['y'], theta=end.get('theta', -1.0)),
            'path_cp1': Pose2D(**cp1),
            'path_cp2': Pose2D(**cp2),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        self.map_tracker.forward(hint=end)

    def dynamic_bezier_reverse(self, start, end, cp1, cp2, constraints):
        rospy.loginfo('TracklessSimBase is dynamically beziering reverse... ' + self.forward_str_fmt,
            start['x'], start['y'], end['x'], end['y'], constraints['next_motion'])
        params = {
            'nav': Goal.NAV_DYNAMIC_BEZIER_REVERSE,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(x=end['x'], y=end['y'], theta=end.get('theta', -1.0)),
            'path_cp1': Pose2D(**cp1),
            'path_cp2': Pose2D(**cp2),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        self.map_tracker.reverse(hint=end)

    def move_forward(self, start, end, constraints):
        rospy.loginfo('TracklessSimBase is moving forward... ' + self.forward_str_fmt,
            start['x'], start['y'], end['x'], end['y'], constraints['next_motion'])
        params = {
            'nav': Goal.NAV_FORWARD,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(**end),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        self.map_tracker.forward(hint=end)

    def move_reverse(self, start, end, constraints):
        rospy.loginfo('TracklessSimBase is moving reverse... ' + self.forward_str_fmt,
            start['x'], start['y'], end['x'], end['y'], constraints['next_motion'])
        params = {
            'nav': Goal.NAV_REVERSE,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(**end),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        self.map_tracker.reverse(hint=end)

    def bezier_forward(self, start, end, cp1, cp2, constraints):
        rospy.loginfo('TracklessSimBase is beziering forward... ' + self.forward_str_fmt,
            start['x'], start['y'], end['x'], end['y'], constraints['next_motion'])
        params = {
            'nav': Goal.NAV_BEZIER_FORWARD,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(**end),
            'path_cp1': Pose2D(**cp1),
            'path_cp2': Pose2D(**cp2),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        self.map_tracker.forward(hint=end)

    def bezier_reverse(self, start, end, cp1, cp2, constraints):
        rospy.loginfo('TracklessSimBase is beziering reverse... ' + self.forward_str_fmt,
            start['x'], start['y'], end['x'], end['y'], constraints['next_motion'])
        params = {
            'nav': Goal.NAV_BEZIER_REVERSE,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(**end),
            'path_cp1': Pose2D(**cp1),
            'path_cp2': Pose2D(**cp2),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        self.map_tracker.reverse(hint=end)

    def forward_dock(self, marker_type, target, alignment_distance, constraints):
        rospy.loginfo('TracklessSimBase is docking forward... [%s, (%.2f, %.2f), %.2f]',
            marker_type, target['x'], target['y'], alignment_distance)
        params = {
            'nav': Goal.NAV_FORWARD_DOCK,
            'marker_type': marker_type,
            'path_end': Pose2D(**target),
            'path_cp2': Pose2D(x=alignment_distance),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        # TODO: map_tracker

    def reverse_dock(self, marker_type, target, alignment_distance, constraints):
        rospy.loginfo('TracklessSimBase is docking reverse... [%s, (%.2f, %.2f), %.2f]',
            marker_type, target['x'], target['y'], alignment_distance)
        params = {
            'nav': Goal.NAV_REVERSE_DOCK,
            'marker_type': marker_type,
            'path_end': Pose2D(**target),
            'path_cp2': Pose2D(x=alignment_distance),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        # TODO: map_tracker

    def omni_dock(self, marker_type, target, alignment_distance, constraints):
        rospy.loginfo('TracklessSimBase is docking omni... [%s, (%.2f, %.2f), %.2f]',
            marker_type, target['x'], target['y'], alignment_distance)
        params = {
            'nav': Goal.NAV_OMNI_DOCK,
            'marker_type': marker_type,
            'path_end': Pose2D(**target),
            'path_cp2': Pose2D(x=alignment_distance),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        # TODO: map_tracker

    def forward_undock(self, marker_type, target, alignment_distance, undock_end, constraints):
        rospy.loginfo('TracklessSimBase is undocking forward... [%s, (%.2f, %.2f), %.2f, (%.2f, %.2f)]',
            marker_type, target['x'], target['y'], alignment_distance, undock_end['x'], undock_end['y'])
        params = {
            'nav': Goal.NAV_FORWARD_UNDOCK,
            'marker_type': marker_type,
            'path_start': Pose2D(**undock_end),
            'path_end': Pose2D(**target),
            'path_cp2': Pose2D(x=alignment_distance),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        # TODO: map_tracker

    def reverse_undock(self, marker_type, target, alignment_distance, undock_end, constraints):
        rospy.loginfo('TracklessSimBase is undocking reverse... [%s, (%.2f, %.2f), %.2f, (%.2f, %.2f)]',
            marker_type, target['x'], target['y'], alignment_distance, undock_end['x'], undock_end['y'])
        params = {
            'nav': Goal.NAV_REVERSE_UNDOCK,
            'marker_type': marker_type,
            'path_start': Pose2D(**undock_end),
            'path_end': Pose2D(**target),
            'path_cp2': Pose2D(x=alignment_distance),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        # TODO: map_tracker

    def free_rotate_left(self, end, heading, constraints):
        rospy.loginfo('TracklessSimBase is free-rotating left... ' + self.rotate_str_fmt,
            end['x'], end['y'], heading)
        params = {
            'nav': Goal.NAV_ROTATE_LEFT,
            'path_end': Pose2D(theta=math.radians(heading), **end),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        self.map_tracker.rotate_left_to_heading(heading)

    def free_rotate_right(self, end, heading, constraints):
        rospy.loginfo('TracklessSimBase is free-rotating right... ' + self.rotate_str_fmt,
            end['x'], end['y'], heading)
        params = {
            'nav': Goal.NAV_ROTATE_RIGHT,
            'path_end': Pose2D(theta=math.radians(heading), **end),
        }
        params.update(constraints)
        self.make_and_send_goalx(**params)
        self.map_tracker.rotate_right_to_heading(heading)

    # override methods in ActionClientMixin
    def make_and_send_goalx(self, **kwargs):
        goal = Goal(**kwargs)
        if goal.speed == self.UNLIMITED_SPEED:
            goal._speed = self.STRAIGHT_DEFAULT_SPEED
        else:
            goal._speed = goal.speed
        if goal.nav in [Goal.NAV_ROTATE_LEFT, Goal.NAV_ROTATE_RIGHT]:
            goal._start_theta = self.get_pose().theta  # store start heading
        goal._elapsed = [0, 0]  # elapsed, total_elapsed

        self.goal = goal
        self.stopped = False
        self.paused = False
        self.robot.power.charging_status = False

        if self.recording_goals:
            goal.paths.paths = goal.paths.paths[:]
            self.goals.append(goal)

    def wait_for_result(self, timeout=rospy.Duration()):
        goal = self.goal
        if isinstance(goal, agv05_msgs.msg.NavActionGoal):  # tracked goal
            return super(TracklessSimBase, self).wait_for_result(timeout, goal)

        if self.speedup_factor:
            s = rospy.get_time()
            r = rospy.Rate(self.granularity * self.speedup_factor)
        else:
            p = 1.0 / self.granularity

        if timeout:
            timeout_time = timeout.to_sec()
        else:
            timeout_time = rospy.Time(2**32 - 1).to_sec()

        if goal:
            elapsed = goal._elapsed
            forward_flag = goal.next_motion in [Goal.MOTION_NONSTOP, Goal.MOTION_NONSTOP_BEZIER]

        while not rospy.is_shutdown() and goal and not self.stopped:
            if self.speedup_factor:
                r.sleep()
                now = rospy.get_time()
                elapsed[0] = (now - s) * self.speedup_factor
                s = now
            else:
                elapsed[0] = p

            timeout_time -= elapsed[0]
            if timeout_time <= 0:
                return False

            if self.stopped:
                break
            elif goal.nav in [Goal.NAV_FORWARD_DOCK, Goal.NAV_REVERSE_DOCK, Goal.NAV_OMNI_DOCK,
                    Goal.NAV_FORWARD_UNDOCK, Goal.NAV_REVERSE_UNDOCK,
                    Goal.NAV_DYNAMIC_FORWARD, Goal.NAV_DYNAMIC_REVERSE]:
                # run for a constant time of 1.5s
                if not (self.paused or self.stopped):
                    elapsed[1] += elapsed[0]
                    if elapsed[1] >= 1.5:
                        break
            elif goal.nav in [Goal.NAV_FORWARD, Goal.NAV_DYNAMIC_LINE_FORWARD]:
                if self._traverse_path(elapsed, goal._speed, forward_dir=True, forward_flag=forward_flag, bezier=False, goal=goal):
                    break
            elif goal.nav in [Goal.NAV_REVERSE, Goal.NAV_DYNAMIC_LINE_REVERSE]:
                if self._traverse_path(elapsed, goal._speed, forward_dir=False, forward_flag=forward_flag, bezier=False, goal=goal):
                    break
            elif goal.nav in [Goal.NAV_BEZIER_FORWARD, Goal.NAV_DYNAMIC_BEZIER_FORWARD]:
                if self._traverse_path(elapsed, goal._speed, forward_dir=True, forward_flag=forward_flag, bezier=True, goal=goal):
                    break
            elif goal.nav in [Goal.NAV_BEZIER_REVERSE, Goal.NAV_DYNAMIC_BEZIER_REVERSE]:
                if self._traverse_path(elapsed, goal._speed, forward_dir=False, forward_flag=forward_flag, bezier=True, goal=goal):
                    break
            elif goal.nav == Goal.NAV_ROTATE_LEFT:
                if self._rotate(elapsed, left_dir=True, goal=goal):
                    break
            elif goal.nav == Goal.NAV_ROTATE_RIGHT:
                if self._rotate(elapsed, left_dir=False, goal=goal):
                    break

        self.goal = None
        self._obstacle_blocked = False
        self.map_tracker.stop()
        return True

    # location tracking
    def reset_odom(self):
        pass

    def _send_transform(self):
        pose = self.get_pose()
        if not pose:
            return
        self.tf_broadcaster.sendTransform(
            (pose.x, pose.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, pose.theta),
            rospy.Time.now(), 'base', 'map')

    # simulation helper functions
    def _traverse_path(self, elapsed, speed, forward_dir, forward_flag, bezier=None, goal=None):
        if self.paused or self.stopped:
            # For DFleet sim agv, do not return if paused but
            # proceed to check for obstacle blocked.
            return False

        try:
            start, end, cp1, cp2, distance, shape = \
                self.__decode_goal_path(goal, bezier) if goal else self.__decode_path()
        except TypeError:
            # no path reference, run for a constant time of 1.5s
            if not (self.paused or self.stopped):
                elapsed[1] += elapsed[0]
                if elapsed[1] >= 1.5:
                    return True
            return False

        if self.paused or self.stopped:
            return False

        while elapsed[0] > 0:
            # prevent huge time gap between collision checks
            e = min(elapsed[0], 0.15)
            d = (elapsed[1] + e) * speed

            if shape == PathShape.STRAIGHT:
                pose = self._interpolate_line(start, end, min(d / distance, 1.0))
                if not forward_dir:
                    pose.theta = self._flip(pose.theta)

            elif shape == PathShape.BEZIER:
                pose = self._interpolate_bezier(start, end, cp1, cp2, min(d / distance, 1.0))
                if not forward_dir:
                    pose.theta = self._flip(pose.theta)

            else:
                assert False, 'Invalid path shape'

            # check collision here (only for DFleet sim agv)
            # do nothing (for standalone sim agv)

            elapsed[0] -= e
            elapsed[1] += e
            self.pose = pose
            self._send_transform()
            deceleration_time = 0 if forward_flag else min(speed / self.DECELERATION, 1.0)
            if d >= distance + deceleration_time * speed:
                return True
        return False

    def __decode_goal_path(self, goal, bezier):  # nothrow
        start = complex(goal.path_start.x, goal.path_start.y)
        end = complex(goal.path_end.x, goal.path_end.y)

        if bezier:
            shape = PathShape.BEZIER
            cp1 = complex(goal.path_cp1.x, goal.path_cp1.y)
            cp2 = complex(goal.path_cp2.x, goal.path_cp2.y)
            try:
                distance = goal._distance
            except AttributeError:
                # compute and cache distance
                distance = self.__compute_bezier_length(start, end, cp1, cp2)
                goal._distance = distance
        else:
            shape = PathShape.STRAIGHT
            distance = self.__compute_line_length(start, end)
            cp1 = None
            cp2 = None
        return start, end, cp1, cp2, distance, shape

    def __decode_path(self):  # throws TypeError when location or path is missing
        j1, h1 = self.map_tracker.prev_location  # throws TypeError
        j2, h2 = self.map_tracker.location  # throws TypeError
        if j1 == j2:
            raise TypeError()

        start = self.map_tracker.graph.node[j1]
        end = self.map_tracker.graph.node[j2]
        try:
            path = self.map_tracker.graph[j1][j2]
            if path['shape'] == PathShape.TELEPORT:
                raise KeyError()  # map tracker will not lookup teleport path
            cp1 = path['cp1']
            cp2 = path['cp2']
        except KeyError:
            # map tracker might lookup path traversed in the illegal direction
            path = self.map_tracker.graph[j2][j1]
            cp1 = path['cp2']
            cp2 = path['cp1']

        start = complex(start['x'], start['y'])
        end = complex(end['x'], end['y'])

        distance = path['distance']
        shape = path['shape']

        if shape == PathShape.BEZIER:
            cp1 = complex(cp1['x'], cp1['y'])
            cp2 = complex(cp2['x'], cp2['y'])
        else:
            cp1 = None
            cp2 = None
        return start, end, cp1, cp2, distance, shape

    def _rotate(self, elapsed, left_dir, goal=None):
        if self.paused or self.stopped:
            # For DFleet sim agv, do not return if paused but
            # proceed to check for obstacle blocked.
            return False

        try:
            end, start_theta, end_theta = \
                self.__decode_goal_rotation(goal) if goal else self.__decode_rotation()
        except TypeError:
            # no heading reference, run for a constant time of 1.5s
            if not (self.paused or self.stopped):
                elapsed[1] += elapsed[0]
                if elapsed[1] >= 1.5:
                    return True
            return False

        if self.paused or self.stopped:
            return False

        speed = self.TURN_DEFAULT_SPEED
        while elapsed[0] > 0:
            # prevent huge time gap between collision checks
            e = min(elapsed[0], 0.15)
            d = (elapsed[1] + e) * speed
            distance = (end_theta - start_theta) % (2 * math.pi * [-1, 1][left_dir])
            distance = abs(distance)

            if d >= distance:
                pose = Pose2D(end.real, end.imag, end_theta)
            elif left_dir:
                pose = Pose2D(end.real, end.imag, start_theta + d)
            else:
                pose = Pose2D(end.real, end.imag, start_theta - d)

            # check collision here (only for DFleet sim agv)
            # do nothing (for standalone sim agv)

            elapsed[0] -= e
            elapsed[1] += e
            self.pose = pose
            self._send_transform()
            deceleration_time = min(speed / self.DECELERATION, 1.0)
            if d >= distance + deceleration_time * speed:
                return True
        return False

    def __decode_goal_rotation(self, goal):  # nothrow
        end = complex(goal.path_end.x, goal.path_end.y)
        start_theta = goal._start_theta
        end_theta = goal.path_end.theta
        return end, start_theta, end_theta

    def __decode_rotation(self):  # throws TypeError when location is missing
        j1, h1 = self.map_tracker.prev_location
        j2, h2 = self.map_tracker.location
        if j1 != j2 or h1 == h2:
            raise TypeError()
        end = self.map_tracker.graph.node[j2]

        end = complex(end['x'], end['y'])
        start_theta = math.radians(h1)
        end_theta = math.radians(h2)
        return end, start_theta, end_theta

    # private (non-inheritable) functions, as indicated by double underscore (__)
    def __compute_line_length(self, start, end):
        return abs(end - start)

    def __compute_bezier_length(self, start, end, cp1, cp2):
        # derivative formula: see https://stackoverflow.com/a/31317254/2312564
        c1 = end - (3 * cp2) + (3 * cp1) - start
        c2 = (3 * cp2) - (6 * cp1) + (3 * start)
        c3 = (3 * cp1) - (3 * start)

        def derivative(t):
            return (3 * c1 * t * t) + (2 * c2 * t) + c3

        # approximation method
        steps = 100
        dt = 1.0 / steps
        length = 0
        for i in range(steps):
            t = (i + 0.5) * dt
            p = derivative(t)
            length += abs(p)
        length *= dt
        return length


class TracklessSim(TracklessMixin, Sim):
    name = 'trackless_sim'
    base_cls = TracklessSimBase

    def start(self, mode):
        if self.is_alive():
            rospy.logwarn('Robot is already started.')
            return

        super(TracklessSim, self).start(mode)

        if mode in (1, 3):
            self.start_amcl()
        elif mode == 2:
            pass

    def stop(self):
        if not self.is_alive():
            rospy.logwarn('Robot has not started yet.')
            return

        if self.get_mode() in (1, 3):
            try:
                self.stop_amcl()
            except Exception:
                pass
        elif self.get_mode() == 2:
            pass

        super(TracklessSim, self).stop()

    def start_amcl(self):
        pass

    def stop_amcl(self):
        pass

    def start_slam(self):
        pass

    def stop_slam(self):
        pass
