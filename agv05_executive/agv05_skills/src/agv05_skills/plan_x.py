from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_msgs.msg import NavxActionGoal as Goal, NavActionResult as Result, Path
from agv05_webserver.system.models import PathBranch
from agv05_webserver.systemx.models import Heading, PathShape
from geometry_msgs.msg import Point32, Polygon
import math
import rospy

from .plan import NavigateTo, CheckAgvPosition, ResetAgvPosition, Motion


CheckAgvPositionX = CheckAgvPosition


class ResetAgvPositionX(ResetAgvPosition):
    HEADING_NA = Heading.NA
    HEADING_DEFAULT = Heading.MIN
    HEADING_WARNING = 'Resetting AGV position to a headingless station, yet the current heading of AGV is unknown.'

    class Meta(ResetAgvPosition.Meta):
        pass


class ResetAgvPositionTrackerX(ResetAgvPositionX):
    reset_tracker_only = True

    class Meta(ResetAgvPositionX.Meta):
        name = 'Reset AGV Position Tracker'

    def __str__(self):
        return 'Reset AGV position tracker to station "%s"' % self.station


class NavigateToX(NavigateTo):
    is_straight = staticmethod(lambda x: x['shape'] == PathShape.STRAIGHT)
    is_dynamic = (lambda self, x: x.get('dynamic') and 'dynamic' in self.robot.models.allowed_motions)
    is_reverse = (lambda self, x: (self.traverse_reversely, False, True)[x['facing']])
    is_nonstop = staticmethod(lambda x: x in [Goal.MOTION_NONSTOP, Goal.MOTION_NONSTOP_BEZIER])

    class Meta:
        name = 'Navigate To'
        params = [
            {
                'name': 'station',
                'type': 'Station',
                'description': 'Choose a destination station.',
            },
            {
                'name': 'align_station_type',
                'type': 'int',
                'description': 'Use front(0) or rear(1) line sensor, or trackless localization(-1) for station alignment.',
                'default': -1,
                'min': -1,
                'max': 1,
            },
            {
                'name': 'next_motion',
                'type': 'int',
                'description': 'Next motion after final junction (0=Idle, 1=Non-Stop, 2=Rotate Left, 3=Rotate Right, 4=Non-Stop Bezier)',
                'default': 0,
                'min': 0,
                'max': 4,
            },
            {
                'name': 'next_speed',
                'type': 'double',
                'description': 'Ending speed limit in m/s for non-stopping next_motion (0=speed)',
                'default': 0.0,
                'min': 0.0,
                'max': 3.0,
            },
            {
                'name': 'sense_line',
                'type': 'int',
                'description': ('Sense line options: 0 - Disable, '
                    '1 - Stop after reaching distance past line, 2 - Stop immediately when line is sensed, '
                    '3 - Stop after reaching distance past full line, 4 - Stop immediately when full line is sensed'),
                'default': 0,
                'min': 0,
                'max': 4,
            },
        ]
        outcomes = ['Done']
        mutexes = ['base', 'plan', 'traffic']

    def execute(self, ud):
        self._dynamic_heading = False  # used internally to track if the final path is dynamic
        return super(NavigateToX, self).execute(ud)

    def _traverse_plan(self):
        # Traverse dynamic plan
        assert len(self.plan) >= 2

        prev_p = self.plan[0]
        cur_p = self.plan[1]
        next_p = self.plan[2] if 2 < len(self.plan) else None

        cur_path = self.robot.models.get_path(prev_p, cur_p)
        next_path = self.robot.models.get_path(cur_p, next_p) if next_p is not None else None
        assert not self.is_teleport(cur_path)
        reverse = self.is_reverse(cur_path)
        next_motion = self.next_motions[1]

        # pre1. Execute transition for startAction if any
        transition_trigger_obj = self.transition_triggers[0]
        self._transition(transition_trigger_obj, next_motion, True)

        if self.preempt_requested():
            return 'Preempted'

        # 1. Rotate
        cur_start = self.robot.models.get_junction(prev_p)
        cur_start_heading = self.compute_edge_start_heading([prev_p, cur_p, cur_path])
        if self.is_dynamic(cur_path):
            # skip rotation in the beginning of dynamic path and update map_tracker manually
            self.robot.base.map_tracker._rotate_to_heading(self.invert_heading(cur_start_heading) if reverse else cur_start_heading)
        else:
            self._rotate_align(cur_start_heading, cur_start, reverse_next=reverse, tracked=cur_path['tracked'])

        if self.preempt_requested():
            return 'Preempted'

        # post1. Execute transition for endAction if any
        self._transition(transition_trigger_obj, next_motion, False)

        if self.preempt_requested():
            return 'Preempted'

        # pre2. Send update to server
        if self.robot.fms_manager:
            self._send_nav_request(transition=(prev_p, cur_p))

        # 2. Straight
        cur_end = self.robot.models.get_junction(cur_p)
        final = not next_path and self.plan_end == self.end
        enable_sensor = not (final and self.robot.config.disable_laser_at_home and cur_p == self.home[0])
        sense_line = self.sense_line if final else 0
        if self.preempt_requested() and self.is_nonstop(next_motion):
            next_motion = Goal.MOTION_IDLE  # cancel non-stop motion
        next_distance = 0
        next_speed = 0
        goal_tolerance = Goal.TOLERANCE_GOAL
        if self.is_nonstop(next_motion):
            next_speed = self.next_speed if next_path is None else self._get_path_speed(next_path)
            if self.is_dynamic(cur_path):
                next_distance = self.next_distance[0]
                goal_tolerance = Goal.TOLERANCE_MID
            elif self.robot.config.forward_speed_limit_by_next_path_length and next_path is not None:
                next_distance = next_path['distance']
            elif self.next_distance[1] < Goal.NEXT_DISTANCE_MAX:
                next_distance = self.next_distance[1]
            if next_path is not None and not self.is_teleport(next_path):
                if not self.is_straight(next_path):
                    next_motion = Goal.MOTION_NONSTOP_BEZIER
        self._move_straight(cur_path, cur_start, cur_end,
            next_motion, next_speed, next_distance,
            goal_tolerance, sense_line, enable_sensor, reverse)
        self._plan_pop()
        self._paths_plan_pop()

        if self.preempt_requested():
            return 'Preempted'
        return True

    def _rotate_to_swapping_heading(self):
        if self.swapping_end == self.plan_end:
            return
        self.swapping_end = self.plan_end
        return self._rotate_to_station_heading(self.plan_end, -1, False)

    def _rotate_to_station_heading(self, location, align_station_type, next_motion=None):
        # Align to station heading
        transition_trigger_obj = None
        rotated = False
        if location[1] != Heading.NA:
            j = self.robot.models.get_junction(location[0])
            current_heading = self.robot.base.get_location()[1]
            next_heading = location[1]
            transition_triggers = self.robot.models.transition_triggers.get(location[0], [])
            for transition in transition_triggers:
                if (self.is_heading_equal(current_heading, transition['previous_heading']) and
                        self.is_heading_equal(next_heading, transition['next_heading']) and
                        self._is_transition_applicable(transition, location[0], False, True)):
                    transition_trigger_obj = transition
                    break

            # Execute transition for startAction if any
            self._transition(transition_trigger_obj, Goal.MOTION_IDLE, True)

            if align_station_type < 0:
                if next_motion is None:  # indicates final destination
                    next_motion = self.next_motions[0]
                    align_at_destination = self.robot.config.always_rotate_align_at_destination
                else:
                    align_at_destination = False

                skip_rotate_align = (
                    self.robot.base.map_tracker.is_tracked_heading() or
                    not (align_at_destination or self._dynamic_heading) or
                    self.is_nonstop(next_motion))
                rotated = self._rotate_align(location[1], j, force_align=not skip_rotate_align)
                self._dynamic_heading = False
            else:
                rotated = self._rotate_align(location[1], j, tracked=True, force_align_sensor=align_station_type)

            # Execute transition for endAction if any
            self._transition(transition_trigger_obj, Goal.MOTION_IDLE, False)

        if self.preempt_requested():
            return 'Preempted'
        return rotated

    def _handle_teleport(self):
        # Perform execution of teleport's pre-action and action.
        if not self.teleport_plan:
            return

        count = len(self.plan)
        assert count >= self.teleport_plan[0]
        if count > self.teleport_plan[0] + self.MAX_TELEPORT_PRE_STEPS:
            return

        steps = count - self.teleport_plan[0]
        prev_p = self.plan[steps]
        cur_p = self.plan[steps + 1]

        cur_path = self.robot.models.get_path(prev_p, cur_p)
        teleport = self.robot.models.teleports[cur_path['teleport']]

        if steps > 0:
            # Execute teleport's pre-action.
            next_motion = self.next_motions[steps + 1]
            self._teleport_pre(teleport, next_motion, steps)
            return

        # Execute teleport's action.
        # 1. Rotate
        teleport_start = self.robot.models.get_location_from_station(teleport['start'])
        self._rotate_to_station_heading(teleport_start, teleport['alignStationType'], teleport['nonStopTransition'])

        if self.preempt_requested():
            return 'Preempted'

        # pre2. Send update to server
        if self.robot.fms_manager:
            self._send_nav_request(transition=(prev_p, cur_p))

        # 2. Teleport
        next_motion = self.next_motions[1]
        if self.preempt_requested() and self.is_nonstop(next_motion):
            next_motion = Goal.MOTION_IDLE  # cancel non-stop motion
        if self._teleport(teleport, next_motion) != 'Preempted':
            if teleport['autoResetAgvPosition'] == 1:
                ResetAgvPositionX(self.robot, station=teleport['end'], skip_navigation_request=True).execute({})
            elif teleport['autoResetAgvPosition'] == 2:
                ResetAgvPositionTrackerX(self.robot, station=teleport['end'], skip_navigation_request=True).execute({})
            elif self.robot.base.get_location()[0] != cur_p:
                self.fail('Teleport action must move the AGV to the end station.')
        self._plan_pop()
        self.teleport_plan.pop(0)

        if self.preempt_requested():
            return 'Preempted'
        return True

    def _pre_handle_plan(self, location):
        # Extract all upcoming plan paths
        count = len(self.plan)
        self.next_motions = [Goal.MOTION_IDLE] * count
        self.next_distance = [Goal.NEXT_DISTANCE_MAX] * count
        self.transition_triggers = [None] * count  # Transition trigger at last junction is not pre-processed
        self.paths_plan = [[]]
        self.teleport_plan = []
        current_transition_start_heading = None
        current_transition_end_heading = None
        next_transition_start_heading = location[1]

        for i in range(count - 1):
            prev_p = self.plan[i]
            cur_p = self.plan[i + 1]
            next_p = self.plan[i + 2] if i + 2 < len(self.plan) else None

            cur_path = self.robot.models.get_path(prev_p, cur_p)
            next_path = self.robot.models.get_path(cur_p, next_p) if next_p is not None else None
            e1 = (prev_p, cur_p, cur_path)
            e2 = (cur_p, next_p, next_path)

            next_motion = self._get_next_motion(e1, e2, self.transitions[i + 1])
            self.next_motions[i + 1] = next_motion
            self.next_distance[i] = cur_path['distance']

            if self.is_teleport(cur_path):
                teleport = self.robot.models.teleports[cur_path['teleport']]
                next_transition_start_heading = self.robot.models.get_location_from_station(teleport['end'])[1]
                self.teleport_plan.append(count - i)  # 1-indexed in reverse direction
                if len(self.paths_plan[-1]):
                    self.paths_plan.append([])
                continue

            # current heading transition (heading is referring to robot heading instead of path heading)
            current_transition_start_heading = next_transition_start_heading
            current_transition_end_heading = self.compute_edge_start_heading(e1)
            next_transition_start_heading = self.compute_edge_end_heading(e1)

            reverse = self.is_reverse(cur_path)
            if reverse:
                current_transition_end_heading = self.invert_heading(current_transition_end_heading)
                next_transition_start_heading = self.invert_heading(next_transition_start_heading)

            # Look for transition trigger at current junction (prev_p)
            transition_trigger_obj = None
            transition_triggers = self.robot.models.transition_triggers.get(prev_p, [])
            cur_location = (prev_p, current_transition_start_heading)

            for transition in transition_triggers:
                if (self.is_heading_equal(current_transition_start_heading, transition['previous_heading']) and
                        self.is_heading_equal(current_transition_end_heading, transition['next_heading']) and
                        self._is_transition_applicable(transition, cur_location, reverse, False)):
                    transition_trigger_obj = transition
                    break

            if transition_trigger_obj:
                self.transition_triggers[i] = transition_trigger_obj
                if transition_trigger_obj['cancelNonStopTransition'] and self.is_nonstop(self.next_motions[i]):
                    self.next_motions[i] = Goal.MOTION_IDLE  # cancel non-stop motion
                    if len(self.paths_plan[-1]):
                        self.paths_plan.append([])

            # continuous non-stopping paths
            start = self.robot.models.get_junction(prev_p)
            end = self.robot.models.get_junction(cur_p)

            path = Polygon(points=[Point32(**start)])
            if cur_path['shape'] == PathShape.BEZIER:
                path.points.append(Point32(**cur_path['cp1']))
                path.points.append(Point32(**cur_path['cp2']))
            path.points.append(Point32(**end))

            self.paths_plan[-1].append(path)
            if not self.is_nonstop(next_motion):
                self.paths_plan.append([])

        if not len(self.paths_plan[-1]):
            self.paths_plan.pop()

        for i in range(count - 2, -1, -1):
            if self.is_nonstop(self.next_motions[i + 1]):
                self.next_distance[i] += self.next_distance[i + 1]
                if self.next_distance[i] > Goal.NEXT_DISTANCE_MAX:
                    self.next_distance[i] = Goal.NEXT_DISTANCE_MAX

    def _plan_pop(self):
        with self._cv:
            if self.robot.fms_manager:
                if self.accepted_error:
                    self.fail('Unable to find a valid path to destination.')
                signature = (self.plan, self.transitions, self.plan_end)
                self.plan = self.accepted_plan
                self.transitions = self.accepted_transitions
                self.plan_end = self.accepted_plan_end
                if signature != (self.plan, self.transitions, self.plan_end):
                    self._pre_handle_plan(self.robot.base.get_location())

            self.next_motions.pop(0)
            self.next_distance.pop(0)
            self.transition_triggers.pop(0)
            self.transitions.pop(0)
            return self.plan.pop(0)

    def _paths_plan_pop(self):
        if len(self.paths_plan[0]) > 1:
            self.paths_plan[0].pop(0)
        else:
            self.paths_plan.pop(0)

    def _rotate_align(self, path_dir, coord, reverse_next=False, tracked=False, force_align_sensor=None, force_align=False):
        assert path_dir >= 0 and path_dir < Heading.NA
        location = self.robot.base.get_location()
        if reverse_next:
            path_dir = self.invert_heading(path_dir)

        # Trackless to Track transition
        if tracked and not self.robot.base.map_tracker.is_tracked_heading():
            force_align = True

        if not force_align and self.is_heading_equal(path_dir, location[1]):
            return

        if force_align_sensor is None:
            force_align_sensor = int(reverse_next)

        for i in range(tracked and self.robot.config.search_line_trial_max):
            self._rotate_force_align(path_dir, coord, location, True, force_align_sensor)
            if self.preempt_requested() or self.robot.base.get_result().result == Result.RESULT_SUCCESS:
                return True
            self.robot.base.map_tracker.set_confused()

        self._rotate_force_align(path_dir, coord, location, False, force_align_sensor)
        return True

    def _rotate_force_align(self, path_dir, coord, location, tracked, force_align_sensor):
        pose = self.robot.base.get_pose()
        tracked_transition = tracked and self.robot.base.map_tracker.is_tracked_heading()
        if pose and not tracked_transition:
            diff_dir = (path_dir - math.degrees(pose.theta)) % 360
        else:
            diff_dir = (path_dir - location[1]) % 360

        while self.is_paused():
            self.robot.panel.toggle_led_paused(True)
            rospy.sleep(0.2)
            if self.preempt_requested():
                return

        transition = self.transitions[0]
        rotate_constraints = {
            'enable_sensor': True,
        }
        if self.is_heading_equal(diff_dir, 180) or diff_dir < 180:
            left_dir = transition not in (Motion.R2, Motion.R3)
        else:
            left_dir = transition in (Motion.L2, Motion.L3)

        if tracked:
            rotate_constraints['rotate_align_sensor'] = force_align_sensor
            if self.is_heading_equal(diff_dir, 90) and tracked_transition:
                fn = self.robot.base.rotate_left if left_dir else self.robot.base.rotate3q_right
                fn(rotate_constraints)
            elif self.is_heading_equal(diff_dir, 180) and tracked_transition:
                fn = self.robot.base.uturn_left if left_dir else self.robot.base.uturn_right
                fn(rotate_constraints)
            elif self.is_heading_equal(diff_dir, 270) and tracked_transition:
                fn = self.robot.base.rotate3q_left if left_dir else self.robot.base.rotate_right
                fn(rotate_constraints)
            elif left_dir:
                rotate_constraints['distance'] = math.radians(diff_dir)
                self.robot.base.search_line_left(rotate_constraints)
                # update map_tracker manually because search_line does not
                self.robot.base.map_tracker.rotate_left_to_heading(path_dir)
            else:
                rotate_constraints['distance'] = math.radians(360 - diff_dir)
                self.robot.base.search_line_right(rotate_constraints)
                # update map_tracker manually because search_line does not
                self.robot.base.map_tracker.rotate_right_to_heading(path_dir)
        else:
            if left_dir:
                self.robot.base.free_rotate_left(coord, path_dir, rotate_constraints)
            else:
                self.robot.base.free_rotate_right(coord, path_dir, rotate_constraints)
        if self.is_paused():
            rospy.sleep(0.2)
            self.robot.base.pause()
        self.wait_for_result()

    def _move_straight(self, path, start, end, next_motion, next_speed, next_distance,
                       goal_tolerance, sense_line, enable_sensor, reverse):
        forward_constraints = {
            'speed': self._get_path_speed(path),
            'next_motion': next_motion,
            'next_speed': next_speed,
            'enable_sensor': enable_sensor,
        }
        straight_path = self.is_straight(path)

        while self.is_paused():
            self.robot.panel.toggle_led_paused(True)
            rospy.sleep(0.2)
            if self.preempt_requested():
                return

        if path['tracked']:
            if self.robot.config.straight_miss_junction_pre_check:
                pose = self.robot.base.get_pose()
                if pose:
                    dx = end['x'] - pose.x
                    dy = end['y'] - pose.y
                    forward_constraints.update({
                        'next_distance': -math.hypot(dx, dy),
                    })
            branch = path['branch']
            rv_branch = path['rv_branch']
            if not reverse:
                if branch == PathBranch.RIGHT:
                    self.robot.base.branch_forward_right(forward_constraints, straight=straight_path)
                elif branch == PathBranch.LEFT:
                    self.robot.base.branch_forward_left(forward_constraints, straight=straight_path)
                elif rv_branch == PathBranch.RIGHT:  # consider reverse direction branch for smooth transversal
                    self.robot.base.branch_forward_left(forward_constraints, straight=straight_path, track_dir=False)
                elif rv_branch == PathBranch.LEFT:
                    self.robot.base.branch_forward_right(forward_constraints, straight=straight_path, track_dir=False)
                else:
                    self.robot.base.forward(forward_constraints, straight=straight_path)
            else:
                if branch == PathBranch.RIGHT:
                    self.robot.base.branch_reverse_right(forward_constraints, straight=straight_path)
                elif branch == PathBranch.LEFT:
                    self.robot.base.branch_reverse_left(forward_constraints, straight=straight_path)
                elif rv_branch == PathBranch.RIGHT:  # consider reverse direction branch for smooth transversal
                    self.robot.base.branch_reverse_left(forward_constraints, straight=straight_path, track_dir=False)
                elif rv_branch == PathBranch.LEFT:
                    self.robot.base.branch_reverse_right(forward_constraints, straight=straight_path, track_dir=False)
                else:
                    self.robot.base.reverse(forward_constraints, straight=straight_path)
        else:
            forward_constraints.update({
                'paths': Path(paths=self.paths_plan[0]),
                'next_distance': next_distance,
                'goal_tolerance': goal_tolerance,
                'sense_line': sense_line,
            })
            if not reverse:
                if straight_path:
                    fn = self.robot.base.dynamic_line_forward if self.is_dynamic(path) else self.robot.base.move_forward
                    fn(start, end, forward_constraints)
                else:
                    fn = self.robot.base.dynamic_bezier_forward if self.is_dynamic(path) else self.robot.base.bezier_forward
                    fn(start, end, path['cp1'], path['cp2'], forward_constraints)
            else:
                if straight_path:
                    fn = self.robot.base.dynamic_line_reverse if self.is_dynamic(path) else self.robot.base.move_reverse
                    fn(start, end, forward_constraints)
                else:
                    fn = self.robot.base.dynamic_bezier_reverse if self.is_dynamic(path) else self.robot.base.bezier_reverse
                    fn(start, end, path['cp1'], path['cp2'], forward_constraints)

        self._dynamic_heading = self.is_dynamic(path)
        if self.is_paused():
            rospy.sleep(0.2)
            self.robot.base.pause()
        self.wait_for_result()

    def _get_next_motion(self, e1, e2, transition):
        if e1[1] == self.end[0] and self.check_transition(e1, self.end[1], reverse=self.traverse_reversely, zero=True):
            # patch: FMS plan to swapping station while crossing final station
            e2 = (-1, -1, None)

        cancel_nonstop = False  # based on e1
        if self.is_teleport(e1[2]):
            teleport = self.robot.models.teleports[e1[2]['teleport']]
            if (teleport['autoResetAgvPosition'] == 1 or
                    teleport['autoResetAgvPosition'] == 2 and self.map_idx(e1[0]) != self.map_idx(e1[1])):
                cancel_nonstop = True
        elif self.is_dynamic(e1[2]):
            if not (e2[2] and self.is_dynamic(e2[2])):
                cancel_nonstop = True

        if e2[2]:
            if (self.is_dynamic(e1[2]) and self.is_dynamic(e2[2]) and
                    self.is_reverse(e1[2]) == self.is_reverse(e2[2])):
                return Goal.MOTION_NONSTOP

            next_motion = self._to_motion(transition)
            if not self.is_nonstop(next_motion):
                pass
            elif cancel_nonstop:
                next_motion = Goal.MOTION_IDLE
            elif self.is_teleport(e2[2]):
                teleport = self.robot.models.teleports[e2[2]['teleport']]
                if (not teleport['nonStopTransition'] or (teleport['alignStationType'] >= 0 and
                        not self.is_teleport(e1[2]) and not e1[2]['tracked'])):
                    next_motion = Goal.MOTION_IDLE  # cancel non-stop motion
            elif not self.is_teleport(e1[2]):
                if (e1[2]['tracked'] != e2[2]['tracked'] or
                        self.is_dynamic(e1[2]) != self.is_dynamic(e2[2]) or
                        self.is_reverse(e1[2]) != self.is_reverse(e2[2])):
                    next_motion = Goal.MOTION_IDLE  # cancel non-stop motion

        elif e1[1] == self.home[0]:
            # disable next motion: final dock or waiting for traffic controller
            next_motion = Goal.MOTION_IDLE

        elif e1[1] == self.end[0] and self.plan_end == self.end:
            # last path to station
            next_motion = self._to_motion(transition)
            if not self.is_nonstop(next_motion):
                pass
            elif cancel_nonstop:
                next_motion = Goal.MOTION_IDLE
            else:
                next_motion = self.next_motion  # follow next_motion input parameter
                if (self.is_nonstop(next_motion) and self.align_station_type >= 0 and
                        not self.is_teleport(e1[2]) and not e1[2]['tracked']):
                    next_motion = Goal.MOTION_IDLE  # cancel non-stop motion

        else:
            # waiting for traffic controller
            next_motion = self._to_motion(transition)
            if self.is_nonstop(next_motion):
                next_motion = Goal.MOTION_IDLE

        return next_motion


class ReverseNavigateToX(NavigateToX):
    traverse_reversely = True

    class Meta(NavigateToX.Meta):
        name = 'Reverse Navigate To'

    def __str__(self):
        return 'Reverse navigating to station "%s"' % self.station
