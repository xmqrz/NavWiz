from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import PathShape
from geometry_msgs.msg import Pose2D
import agv05_msgs.msg
import cmath
import math
import rospy

from .base import TrackedBase
from .robot import TrackedMixin
from .sim import Sim

Goal = type(str('Goal'), (agv05_msgs.msg.NavActionGoal, ), {})  # overcome fixed __slots__
Feedback = agv05_msgs.msg.NavActionFeedback
Result = agv05_msgs.msg.NavActionResult


class TrackedSimBase(TrackedBase):
    manual_cmd_vel_topic = '/cmd_vel'
    action_server = '/nav_action'
    action_spec = agv05_msgs.msg.NavActionAction

    UNLIMITED_SPEED = 10.0
    STRAIGHT_DEFAULT_SPEED = 0.7  # m/s
    TURN_DEFAULT_SPEED = 0.7  # rad/s
    DECELERATION = 1.0  # m/s2 and rad/s2

    granularity = 10  # 10 samples per sec
    speedup_factor = 1  # 1x speed-up; simulation rate = 1 x 10 = 10Hz
    recording_goals = False

    def __init__(self, robot):
        super(TrackedSimBase, self).__init__(robot)
        self.mileage = 1716.7
        self.goal = None
        self.stopped = False
        self.paused = False
        self._wait_traffic = False
        self._obstacle_blocked = False  # only used in DFleet sim agv

        self.pose = None
        self.__pose_pub = rospy.Publisher('/robot_pose', Pose2D, queue_size=1, latch=True)
        self.__update_status_timer = rospy.Timer(rospy.Duration(0.1), self._update_status)

    def forward(self, constraints, straight=True):
        rospy.loginfo('TrackedSimBase is forwarding... [NM=%d]', constraints['next_motion'])
        params = {
            'nav': Goal.NAV_FORWARD if straight else Goal.NAV_BEZIER_FORWARD,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.forward()

    def reverse(self, constraints, straight=True):
        rospy.loginfo('TrackedSimBase is reversing... [NM=%d]', constraints['next_motion'])
        params = {
            'nav': Goal.NAV_REVERSE if straight else Goal.NAV_BEZIER_REVERSE,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.reverse()

    def branch_forward_left(self, constraints, straight=True, track_dir=True):
        rospy.loginfo('TrackedSimBase is forwarding left... [NM=%d]', constraints['next_motion'])
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
        rospy.loginfo('TrackedSimBase is forwarding right... [NM=%d]', constraints['next_motion'])
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
        rospy.loginfo('TrackedSimBase is reversing left... [NM=%d]', constraints['next_motion'])
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
        rospy.loginfo('TrackedSimBase is reversing right... [NM=%d]', constraints['next_motion'])
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
        rospy.loginfo('TrackedSimBase is rotating left...')
        params = {
            'nav': Goal.NAV_ROTATE_LEFT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.rotate_left()

    def rotate_right(self, constraints):
        rospy.loginfo('TrackedSimBase is rotating right...')
        params = {
            'nav': Goal.NAV_ROTATE_RIGHT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.rotate_right()

    def uturn_left(self, constraints):
        rospy.loginfo('TrackedSimBase is u-turning left...')
        params = {
            'nav': Goal.NAV_UTURN_LEFT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.uturn_left()

    def uturn_right(self, constraints):
        rospy.loginfo('TrackedSimBase is u-turning right...')
        params = {
            'nav': Goal.NAV_UTURN_RIGHT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.uturn_right()

    def rotate3q_left(self, constraints):
        rospy.loginfo('TrackedSimBase is rotating three-quarter left...')
        params = {
            'nav': Goal.NAV_ROTATE3Q_LEFT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.rotate3q_left()

    def rotate3q_right(self, constraints):
        rospy.loginfo('TrackedSimBase is rotating three-quarter right...')
        params = {
            'nav': Goal.NAV_ROTATE3Q_RIGHT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.rotate3q_right()

    def search_line_left(self, constraints):
        rospy.loginfo('TrackedSimBase is searching line left...')
        params = {
            'nav': Goal.NAV_SEARCH_LINE_LEFT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def search_line_right(self, constraints):
        rospy.loginfo('TrackedSimBase is searching line right...')
        params = {
            'nav': Goal.NAV_SEARCH_LINE_RIGHT,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def wait_traffic(self, constraints={}):
        rospy.loginfo('TrackedSimBase is waiting for traffic...')
        params = {
            'nav': Goal.NAV_WAIT_TRAFFIC,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def free_motor(self, constraints={}):
        rospy.loginfo('TrackedSimBase is freeing motor...')
        params = {
            'nav': Goal.NAV_FREE_MOTOR,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.set_confused()

    def manual_control(self, constraints={}):
        rospy.loginfo('TrackedSimBase is handling manual control...')
        params = {
            'nav': Goal.NAV_MANUAL_CONTROL,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.set_confused()

    def line_calibrate(self, constraints={}):
        rospy.loginfo('TrackedSimBase is calibrating line...')
        params = {
            'nav': Goal.NAV_LINE_CALIBRATE,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.set_confused()

    def line_tune_pid(self, constraints={}):
        rospy.loginfo('TrackedSimBase is tuning line following PID...')
        params = {
            'nav': Goal.NAV_LINE_TUNE_PID,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.set_confused()

    def select_nav_profile(self, profile, constraints={}):
        rospy.loginfo('TrackedSimBase is switching nav profile...')
        if profile < 1 or profile > 5:
            return

        params = {
            'nav': Goal.NAV_SELECT_NAV_PROFILE1 + profile - 1,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def select_laser_profile(self, profile, constraints={}):
        rospy.loginfo('TrackedSimBase is switching laser profile...')
        if profile < 1 or profile > 10:
            return

        params = {
            'nav': Goal.NAV_SELECT_LASER_PROFILE1 + profile - 1,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def scan_laser_area(self, area, constraints={}):
        rospy.loginfo('TrackedSimBase is performing laser scanning...')
        if area < 1 or area > 31:
            return

        params = {
            'nav': Goal.NAV_SCAN_LASER_AREA1 + area - 1,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def stop(self, constraints={}, make_confused=True):
        rospy.loginfo('TrackedSimBase is stopping...')
        self.goal = None
        self.stopped = True
        self.paused = False
        self._wait_traffic = False
        self._obstacle_blocked = False
        self.map_tracker.stop()
        if make_confused:
            self.map_tracker.set_confused()

    def stop_wait_traffic(self, constraints={}):
        rospy.loginfo('TrackedSimBase is stopping wait traffic...')
        self.goal = None
        self.stopped = True
        self.paused = False
        self._wait_traffic = False
        self._obstacle_blocked = False

    def cancel_forward_flag(self):
        rospy.loginfo('TrackedSimBase is cancelling forward flag...')

    def pause(self):
        rospy.loginfo('TrackedSimBase is pausing...')
        self.paused = True

    def resume(self):
        rospy.loginfo('TrackedSimBase is resuming...')
        self.paused = False

    def safety_resume(self):
        rospy.loginfo('TrackedSimBase is resuming from safety...')

    def _update_status(self, timer_event):
        # prioritize other issue before pause state, according the actual agv05_nav
        if self._wait_traffic:
            self._handle_feedback(Feedback(status=Feedback.STATUS_WAIT_TRAFFIC))
        elif self._obstacle_blocked:
            self._handle_feedback(Feedback(status=Feedback.STATUS_OBSTACLE_BLOCKED))
        elif self.paused:
            self._handle_feedback(Feedback(status=Feedback.STATUS_PAUSED))

    # for testing use
    def start_recording_goals(self):
        self.goals = []
        self.recording_goals = True

    def stop_recording_goals(self):
        self.recording_goals = False
        goals = self.goals
        self.goals = []
        return goals

    # override methods in ActionClientMixin
    def make_and_send_goal(self, **kwargs):
        goal = Goal(**kwargs)
        if goal.speed == self.UNLIMITED_SPEED:
            goal._speed = self.STRAIGHT_DEFAULT_SPEED
        else:
            goal._speed = goal.speed
        goal._elapsed = [0, 0]  # elapsed, total_elapsed

        self.goal = goal
        self.stopped = False
        self.paused = False

        if self.recording_goals:
            self.goals.append(goal)

        if goal.nav == Goal.NAV_WAIT_TRAFFIC:
            self._wait_traffic = True
        elif not (Goal.NAV_SELECT_NAV_PROFILE1 <= goal.nav <= Goal.NAV_SCAN_LASER_AREA31):
            self.robot.power.charging_status = False

    def get_result(self):
        return Result(result=Result.RESULT_SUCCESS)

    def wait_for_result(self, timeout=rospy.Duration(), goal=None):
        if self.speedup_factor:
            s = rospy.get_time()
            r = rospy.Rate(self.granularity * self.speedup_factor)
        else:
            p = 1.0 / self.granularity

        if timeout:
            timeout_time = timeout.to_sec()
        else:
            timeout_time = rospy.Time(2**32 - 1).to_sec()

        goal = goal or self.goal
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
            elif Goal.NAV_SELECT_NAV_PROFILE1 <= goal.nav <= Goal.NAV_SCAN_LASER_AREA31:
                # complete immediately
                break
            elif goal.nav in [Goal.NAV_MANUAL_CONTROL, Goal.NAV_FREE_MOTOR, Goal.NAV_WAIT_TRAFFIC]:
                # run indefinitely until stopped
                continue
            elif goal.nav in [Goal.NAV_SEARCH_LINE_LEFT, Goal.NAV_SEARCH_LINE_RIGHT,
                              Goal.NAV_LINE_CALIBRATE, Goal.NAV_LINE_TUNE_PID]:
                # run for a constant time of 1.5s
                if not (self.paused or self.stopped):
                    elapsed[1] += elapsed[0]
                    if elapsed[1] >= 1.5:
                        break
            elif goal.nav in [Goal.NAV_FORWARD, Goal.NAV_FORWARD_LEFT, Goal.NAV_FORWARD_RIGHT,
                              Goal.NAV_BEZIER_FORWARD, Goal.NAV_BEZIER_FORWARD_LEFT, Goal.NAV_BEZIER_FORWARD_RIGHT]:
                if self._traverse_path(elapsed, goal._speed, forward_dir=True, forward_flag=forward_flag):
                    break
            elif goal.nav in [Goal.NAV_REVERSE, Goal.NAV_REVERSE_LEFT, Goal.NAV_REVERSE_RIGHT,
                              Goal.NAV_BEZIER_REVERSE, Goal.NAV_BEZIER_REVERSE_LEFT, Goal.NAV_BEZIER_REVERSE_RIGHT]:
                if self._traverse_path(elapsed, goal._speed, forward_dir=False, forward_flag=forward_flag):
                    break
            elif goal.nav in [Goal.NAV_ROTATE_LEFT, Goal.NAV_UTURN_LEFT, Goal.NAV_ROTATE3Q_LEFT]:
                if self._rotate(elapsed, left_dir=True):
                    break
            elif goal.nav in [Goal.NAV_ROTATE_RIGHT, Goal.NAV_UTURN_RIGHT, Goal.NAV_ROTATE3Q_RIGHT]:
                if self._rotate(elapsed, left_dir=False):
                    break

        self.goal = None
        self._obstacle_blocked = False
        self.map_tracker.stop()
        return True

    # location tracking
    def get_pose(self):
        return self.pose

    def reset_pose(self):
        self.pose = None

    def set_initial_pose(self, pose, precise=False):
        rospy.loginfo('TrackedSimBase is settings initial pose to [(%.2f, %.2f, %.1f deg)]',
            pose.x, pose.y, math.degrees(pose.theta))
        self.pose = pose
        self._send_transform()

    def set_initial_map_and_location(self, map, location):
        super(TrackedSimBase, self).set_initial_map_and_location(map, location)
        self.set_initial_pose(self.map_tracker.get_pose())

    def set_initial_map_and_location_tracker(self, map, location):
        super(TrackedSimBase, self).set_initial_map_and_location(map, location)

    def _send_transform(self):
        pose = self.get_pose()
        if not pose:
            return
        self.__pose_pub.publish(pose)

    # simulation helper functions
    def _traverse_path(self, elapsed, speed, forward_dir, forward_flag):
        if self.paused or self.stopped:
            # For DFleet sim agv, do not return if paused but
            # proceed to check for obstacle blocked.
            return False

        try:
            start, end, start_theta, end_theta, distance, shape = self.__decode_path()
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

            if d >= distance:
                pose = Pose2D(end.real, end.imag, end_theta)

            elif shape == PathShape.STRAIGHT:
                pose = self._interpolate_line(start, end, d / distance)
                if not forward_dir:
                    pose.theta = self._flip(pose.theta)

            elif shape == PathShape.S_CURVE:
                if forward_dir:
                    pose = self.__interpolate_scurve(start, end, start_theta, d, distance)
                else:
                    pose = self.__interpolate_scurve(start, end, self._flip(start_theta), d, distance)
                    pose.theta = self._flip(pose.theta)

            elif shape in [PathShape.BEND_LEFT, PathShape.BEND_RIGHT]:
                if forward_dir:
                    pose = self.__interpolate_bend(start, end, start_theta, end_theta, d, distance)
                else:
                    pose = self.__interpolate_bend(start, end, self._flip(start_theta),
                                                   self._flip(end_theta), d, distance)
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
        except KeyError:
            # map tracker might lookup path traversed in the illegal direction
            path = self.map_tracker.graph[j2][j1]

        start = complex(start['x'], start['y'])
        end = complex(end['x'], end['y'])
        start_theta = (h1 - 1) * math.pi / 2
        end_theta = (h2 - 1) * math.pi / 2

        distance = path['distance']
        shape = path['shape']
        return start, end, start_theta, end_theta, distance, shape

    def _rotate(self, elapsed, left_dir):
        if self.paused or self.stopped:
            # For DFleet sim agv, do not return if paused but
            # proceed to check for obstacle blocked.
            return False

        try:
            end, start_theta, end_theta = self.__decode_rotation()
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
            distance = (end_theta - start_theta) % (2 * math.pi * [1, -1][left_dir])
            distance = abs(distance)

            if d >= distance:
                pose = Pose2D(end.real, end.imag, end_theta)
            elif left_dir:
                pose = Pose2D(end.real, end.imag, start_theta - d)
            else:
                pose = Pose2D(end.real, end.imag, start_theta + d)

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

    def __decode_rotation(self):  # throws TypeError when location is missing
        j1, h1 = self.map_tracker.prev_location
        j2, h2 = self.map_tracker.location
        if j1 != j2 or h1 == h2:
            raise TypeError()
        end = self.map_tracker.graph.node[j2]

        end = complex(end['x'], end['y'])
        start_theta = (h1 - 1) * math.pi / 2
        end_theta = (h2 - 1) * math.pi / 2
        return end, start_theta, end_theta

    def _flip(self, theta):
        return (theta + math.pi) % (2 * math.pi)

    def _interpolate_line(self, start, end, t):
        diff = end - start
        p = start + diff * t
        return Pose2D(p.real, p.imag, cmath.phase(diff))

    def _interpolate_bezier(self, start, end, cp1, cp2, t):
        t2 = t * t
        t3 = t2 * t
        inv_t = 1.0 - t
        inv_t2 = inv_t * inv_t
        inv_t3 = inv_t2 * inv_t

        p = (inv_t3 * start
            + 3 * inv_t2 * t * cp1
            + 3 * inv_t * t2 * cp2
            + t3 * end)
        dp = (3 * inv_t2 * (cp1 - start)
            + 6 * inv_t * t * (cp2 - cp1)
            + 3 * t2 * (end - cp2))

        return Pose2D(p.real, p.imag, cmath.phase(dp))

    # private (non-inheritable) functions, as indicated by double underscore (__)
    def __interpolate_scurve(self, start, end, start_theta, d, distance):
        diff = end - start
        radius = min(abs(diff.real), abs(diff.imag)) / 4

        v = cmath.rect(1, start_theta)  # unit vector
        mid = self.__dot(v, diff) / 2
        d1 = mid - radius
        d2 = distance - d1

        # 3 sections: line, bezier, line
        if d <= d1:
            p = start + v * d
            return Pose2D(p.real, p.imag, start_theta)
        elif d >= d2:
            p = end + v * (d - distance)
            return Pose2D(p.real, p.imag, start_theta)
        else:
            p0 = start + v * (mid - radius * 1.0)
            c0 = start + v * (mid - radius * 0.5)
            p1 = end - v * (mid - radius * 1.0)
            c1 = end - v * (mid - radius * 0.5)
            t = (d - d1) / (d2 - d1)
            return self._interpolate_bezier(p0, p1, c0, c1, t)

    def __interpolate_bend(self, start, end, start_theta, end_theta, d, distance):
        diff = end - start
        radius = min(abs(diff.real), abs(diff.imag), 2.0) / 2

        v1 = cmath.rect(1, start_theta)  # unit vector
        v2 = cmath.rect(1, end_theta)
        d1 = self.__dot(v1, diff) - radius
        d2 = distance - self.__dot(v2, diff) + radius

        # 3 sections: line, arc, line
        if d <= d1:
            p = start + v1 * d
            return Pose2D(p.real, p.imag, start_theta)
        elif d >= d2:
            p = end + v2 * (d - distance)
            return Pose2D(p.real, p.imag, end_theta)
        else:
            th = (d - d1) / (d2 - d1) * math.pi / 2
            sin_th = math.sin(th)
            cos_th = math.cos(th)
            p = start + v1 * (d1 + radius * sin_th) + v2 * (radius * (1 - cos_th))
            return Pose2D(p.real, p.imag, start_theta + th * self.__cross(v1, v2))

    def __dot(self, v1, v2):  # dot product
        return v1.real * v2.real + v1.imag * v2.imag

    def __cross(self, v1, v2):  # cross product
        return v1.real * v2.imag - v1.imag * v2.real


class TrackedSim(TrackedMixin, Sim):
    name = 'tracked_sim'
    base_cls = TrackedSimBase
