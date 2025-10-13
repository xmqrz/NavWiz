from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.models.map_tracker import Motion
from agv05_executor.skill import Skill
from agv05_executor.state_machine import StateConstructError, state_factory
from agv05_msgs.msg import NavActionGoal as Goal
from agv05_webserver.system.models import Direction, PathBranch, PathShape
from six.moves import zip
import rospy
import six
import threading
import time

from .mixin import CancelForwardFlagOnPreempt


class CheckAgvPosition(Skill):

    class Meta:
        name = 'Check AGV Position'
        params = [
            {
                'name': 'station',
                'type': 'Station',
                'description': 'Check if agv is currently located at this station.',
            },
        ]
        outcomes = ['Yes', 'No']
        mutexes = []

    def __str__(self):
        return 'Checking whether AGV is at station "%s".' % self.station

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        j, h = self.robot.models.get_location_from_station(self.station)
        return 'Yes' if self.check_position(j, h) else 'No'

    def check_position(self, junction, heading):
        agv_location = self.robot.base.get_location()
        if not isinstance(agv_location, tuple):
            return False
        if agv_location[0] != junction:
            return False
        if not self.robot.models.check_transition(agv_location[1], heading, zero=True):
            return False
        return True


class CheckAgvPositionPrefix(CheckAgvPosition):

    class Meta(CheckAgvPosition.Meta):
        name = 'Check AGV Position Prefix'
        params = [
            {
                'name': 'station_prefix',
                'type': 'str',
                'description': 'Check if agv is currently located at any station with this prefix.',
            },
        ]

    def __str__(self):
        return 'Checking whether AGV is at station with prefix "%s".' % self.station_prefix

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        for station in self.robot.models.stations_assoc:
            if station.startswith(self.station_prefix):
                j, h = self.robot.models.stations_assoc[station]
                if self.check_position(j, h):
                    return 'Yes'
        return 'No'


class CheckAgvPositionSuffix(CheckAgvPosition):

    class Meta(CheckAgvPosition.Meta):
        name = 'Check AGV Position Suffix'
        params = [
            {
                'name': 'station_suffix',
                'type': 'str',
                'description': 'Check if agv is currently located at any station with this suffix.',
            },
        ]

    def __str__(self):
        return 'Checking whether AGV is at station with suffix "%s".' % self.station_suffix

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        for station in self.robot.models.stations_assoc:
            if station.endswith(self.station_suffix):
                j, h = self.robot.models.stations_assoc[station]
                if self.check_position(j, h):
                    return 'Yes'
        return 'No'


class IsAgvHomed(CheckAgvPosition):

    class Meta:
        name = 'Is AGV Homed'
        params = []
        outcomes = ['Yes', 'No']
        mutexes = []

    def __str__(self):
        return 'Checking whether AGV is homed.'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        j, h = self.robot.models.get_agv_home_location()
        return 'Yes' if self.check_position(j, h) else 'No'


class CheckAgvPositionRfid(CheckAgvPosition):

    class Meta:
        name = 'Check AGV Position RFID'
        params = [
            {
                'name': 'rfid_string',
                'type': 'str',
                'description': 'Check if agv is currently located at the junction matching this rfid.',
            },
        ]
        outcomes = ['Yes', 'No']
        mutexes = []

    def __str__(self):
        return 'Checking whether AGV is at the junction matching rfid "%s".' % self.rfid_string

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        location = self.robot.models.get_location_from_rfid(self.rfid_string)
        if location and self.check_position(*location):
            return 'Yes'

        location = self.robot.models.get_location_from_station(self.rfid_string)
        if location and self.check_position(*location):
            return 'Yes'

        return 'No'


class ResetAgvPosition(Skill):
    reset_tracker_only = False
    skip_navigation_request = False

    HEADING_NA = Direction.NA
    HEADING_DEFAULT = Direction.NORTH
    HEADING_WARNING = 'Resetting AGV position to a directionless station, yet the current direction of AGV is unknown.'

    class Meta:
        name = 'Reset AGV Position'
        params = [
            {
                'name': 'station',
                'type': 'Station',
                'description': 'Reset position to this station.',
            },
        ]
        outcomes = ['Done']
        mutexes = ['base']

    def __str__(self):
        return 'Reset AGV position to station "%s"' % self.station

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        j, h = self.robot.models.get_location_from_station(self.station)
        self.set_position(j, h)

        # set_position for trackless might take 2-3 seconds, therefore check for preempt again.
        if self.preempt_requested():
            return 'Preempted'
        return 'Done'

    def set_position(self, junction, heading, flip_heading=False):
        if heading == self.HEADING_NA:
            agv_location = self.robot.base.get_location()
            if isinstance(agv_location, tuple):
                heading = agv_location[1]
            else:
                rospy.logwarn(self.HEADING_WARNING)
                heading = self.HEADING_DEFAULT
        elif flip_heading:
            heading = (heading + self.HEADING_NA / 2) % self.HEADING_NA

        reset_tracker_only = self.reset_tracker_only
        if reset_tracker_only:
            map_idx = self.robot.base.map_tracker.map_idx
            agv_location = self.robot.base.get_location()
            if not isinstance(agv_location, tuple):
                rospy.logwarn('Resetting AGV position in addition to tracker as the current location of AGV is unknown.')
                reset_tracker_only = False
            elif map_idx(agv_location[0]) != map_idx(junction):
                rospy.logwarn('Resetting AGV position in addition to tracker as the map has changed.')
                reset_tracker_only = False

        location = (junction, heading)
        if reset_tracker_only:
            self.robot.base.set_initial_map_and_location_tracker(self.robot.models.graph, location)
        else:
            self.robot.base.set_initial_map_and_location(self.robot.models.graph, location)

        if self.robot.fms_manager and not self.skip_navigation_request and not getattr(self.robot, 'inner_sm', None):
            self.robot.fms_manager.send_occupation_request_async()


class NavigateTo(CancelForwardFlagOnPreempt, Skill):
    traverse_reversely = False
    is_straight = staticmethod(lambda x: x['shape'] == PathShape.STRAIGHT)
    is_reverse = (lambda self, x: (self.traverse_reversely, False, True)[x['facing']])
    is_nonstop = staticmethod(lambda x: x in [Goal.MOTION_NONSTOP, Goal.MOTION_NONSTOP_BEZIER])

    MAX_TELEPORT_PRE_STEPS = 10
    UNLIMITED_SPEED = 10.0

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
                'description': 'Use front(0) or rear(1) line sensor for station alignment.',
                'default': 0,
                'min': 0,
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
        ]
        outcomes = ['Done']
        mutexes = ['base', 'plan', 'traffic']

    def __init__(self, *args, **kwargs):
        super(NavigateTo, self).__init__(*args, **kwargs)
        self.robot.base.map_tracker.patch_models(self)
        self._lock = threading.Lock()

    def __str__(self):
        return 'Navigating to station "%s"' % self.station

    def request_preempt(self):
        super(NavigateTo, self).request_preempt()
        with self._lock:
            if self.teleport_pre_sm:
                self.teleport_pre_sm.request_preempt()

    def execute(self, ud):
        rospy.loginfo('%s', self)

        with self._lock:
            self.teleport_pre_sm = None
            self.teleport_pre_thread = None

        self._cv = threading.Condition()
        self.plan = []
        self.transitions = []
        self.plan_end = None

        if self.robot.fms_manager:
            self.swapping_end = None
            self.accepted_error = False
            self.accepted_plan = []
            self.accepted_transitions = []
            self.accepted_plan_end = ()
            self.accepted_reply = 0
            self.seen_reply = 0
            self.robot.fms_manager.set_navigation_approval_cb(self.handle_navigation_approval)

        outcome = self._execute0()

        # Send final update to server
        if self.robot.fms_manager:
            self.robot.fms_manager.set_navigation_approval_cb(None)
            if not self.preempt_requested():
                self.robot.fms_manager.send_occupation_request_async()

        # Preempt and wait for teleport_pre_thread.
        with self._lock:
            if self.teleport_pre_sm:
                self.teleport_pre_sm.request_preempt()
                self.teleport_pre_sm = None

            if self.teleport_pre_thread:
                self.teleport_pre_thread.join()
                self.teleport_pre_thread = None

        return outcome

    def _execute0(self):
        if self.preempt_requested():
            return 'Preempted'

        # Prepare start, end and home location
        self.start = self.robot.base.get_location()
        self.end = self.robot.models.get_location_from_station(self.station)
        self.home = self.robot.models.get_agv_home_location()

        if not self.start:
            self.fail('AGV does not know its own location.')
        if not self.end:
            self.fail('Invalid destination station.')

        # Execute dynamic plan
        self._plan_path(self.start)
        if self.robot.fms_manager:
            self.__nav_request_location = (None, None, None)
            self._send_nav_request(self.start)
        waiting_traffic = False

        while not (self.preempt_requested() or len(self.plan) == 1 and self.plan_end == self.end):
            if len(self.plan) <= 1:
                if not self.robot.fms_manager:
                    break
                if self.plan and self.plan_end != self.end:
                    if self._rotate_to_swapping_heading():
                        continue
                with self._cv:
                    if self.accepted_error:
                        if waiting_traffic:
                            self._stop_wait_traffic()
                            waiting_traffic = False
                        self.fail('Unable to find a valid path to destination.')
                    signature = (self.plan, self.transitions, self.plan_end)
                    if signature == (self.accepted_plan, self.accepted_transitions, self.accepted_plan_end):
                        self._cv.wait(1)
                    if signature != (self.accepted_plan, self.accepted_transitions, self.accepted_plan_end):
                        self.plan = self.accepted_plan
                        self.transitions = self.accepted_transitions
                        self.plan_end = self.accepted_plan_end
                        self._pre_handle_plan(self.robot.base.get_location())
                        continue
                if not waiting_traffic:
                    self.robot.base.wait_traffic()
                    waiting_traffic = True
            else:
                if waiting_traffic:
                    self._stop_wait_traffic()
                    waiting_traffic = False
                self._handle_teleport() or self._traverse_plan()

            if self.robot.fms_manager:
                self._send_nav_request(self.robot.base.get_location())

        if waiting_traffic:
            self._stop_wait_traffic()
            waiting_traffic = False

        if self.preempt_requested():
            return 'Preempted'

        # Execute final rotate
        if self.end[0] != self.home[0]:
            self._rotate_to_destination_heading()

        if self.preempt_requested():
            return 'Preempted'
        return 'Done'

    def _is_transition_applicable(self, transition, cur_location, reverse, is_destination):
        if is_destination:
            return 4 in transition['applicableMotion']
        elif cur_location == self.start:
            if reverse:
                return 1 in transition['applicableMotion']
            return 0 in transition['applicableMotion']
        else:
            if reverse:
                return 3 in transition['applicableMotion']
            return 2 in transition['applicableMotion']

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
        self._rotate_align(cur_path['direction'], reverse_next=reverse)

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
        final = not next_path and self.plan_end == self.end
        enable_sensor = not (final and self.robot.config.disable_laser_at_home and cur_p == self.home[0])
        if self.preempt_requested() and self.is_nonstop(next_motion):
            next_motion = Goal.MOTION_IDLE  # cancel non-stop motion
        next_speed = 0
        if self.is_nonstop(next_motion):
            next_speed = self.next_speed if next_path is None else self._get_path_speed(next_path)
            if next_path is not None and not self.is_teleport(next_path):
                if not self.is_straight(next_path):
                    next_motion = Goal.MOTION_NONSTOP_BEZIER
        self._move_straight(cur_path, next_motion, next_speed, enable_sensor, reverse)
        self._plan_pop()

        if self.preempt_requested():
            return 'Preempted'
        return True

    def _rotate_to_destination_heading(self):
        return self._rotate_to_station_heading(self.end, self.align_station_type)

    def _rotate_to_swapping_heading(self):
        if self.swapping_end == self.plan_end:
            return
        self.swapping_end = self.plan_end
        return self._rotate_to_station_heading(self.plan_end, int(self.traverse_reversely))

    def _rotate_to_station_heading(self, location, align_station_type):
        transition_trigger_obj = None
        current_heading = self.robot.base.get_location()[1]
        next_heading = location[1]
        transition_triggers = self.robot.models.transition_triggers.get(location[0], [])
        transition_trigger_obj = None
        for transition in transition_triggers:
            if current_heading != transition['previous_heading'] or next_heading != transition['next_heading']:
                continue
            if self._is_transition_applicable(transition, location[0], False, True):
                transition_trigger_obj = transition
                break

        # Execute transition for startAction if any
        self._transition(transition_trigger_obj, Goal.MOTION_IDLE, True)

        # Align to station heading
        rotated = False
        if location[1] != Direction.NA:
            rotated = self._rotate_align(location[1], force_align_sensor=align_station_type)

        if self.preempt_requested():
            return 'Preempted'

        # Execute transition for endAction if any
        self._transition(transition_trigger_obj, Goal.MOTION_IDLE, False)

        return rotated

    def _handle_teleport(self):
        # Perform execution of teleport's pre-action and action.
        if not self.teleport_plan:
            return

        count = len(self.plan)
        assert (count >= self.teleport_plan[0])
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
        self._rotate_to_station_heading(teleport_start, teleport['alignStationType'])

        if self.preempt_requested():
            return 'Preempted'

        # pre2. Send update to server
        if self.robot.fms_manager:
            self._send_nav_request(transition=(prev_p, cur_p))

        # 2. Teleport
        next_motion = self.next_motions[1]
        if self.preempt_requested() and self.is_nonstop(next_motion):
            next_motion = Goal.MOTION_IDLE  # cancel non-stop motion

        rfid_location = 1 if self.traverse_reversely else 0
        if teleport['validateRfid']:
            self.robot.rfid._reset_rfid(rfid_location)

        if self._teleport(teleport, next_motion) != 'Preempted':
            if teleport['autoResetAgvPosition']:
                ResetAgvPosition(self.robot, station=teleport['end'], skip_navigation_request=True).execute({})
            elif self.robot.base.get_location()[0] != cur_p:
                self.fail('Teleport action must move the AGV to the end station.')
        self._plan_pop()
        self.teleport_plan.pop(0)

        if self.preempt_requested():
            return 'Preempted'

        if teleport['validateRfid']:
            self._validate_rfid(rfid_location)

        return True

    def handle_navigation_approval(self, msg):
        # sanitize msg
        try:
            timestamp = float(msg['timestamp'])
            approved = bool(msg['approved'])
            signature = (int(msg['dp']), tuple(msg['start']), tuple(msg['end']), bool(msg['reverse']))
            error = msg.get('error', False)
            if approved:
                plan = msg['plan']
                plan_transitions = msg['plan_transitions']
                plan_end = tuple(msg['plan_end'])
                assert isinstance(plan, list)
                assert isinstance(plan_transitions, list)
        except Exception:
            return

        # discard out-of-order (older) navigation approval
        if timestamp <= self.seen_reply:
            return
        self.seen_reply = timestamp

        with self._cv:
            if signature != (
                    self.robot.base.get_dimension_profile(),
                    self.start, self.end, self.traverse_reversely):
                return
            if error:
                self.accepted_error = error
                return
            if not approved or not plan:
                return

            if len(self.plan) >= 2:
                cur_path = tuple(self.plan[:2])
                try:
                    idx = list(zip(plan, plan[1:])).index(cur_path)
                except Exception:
                    idx = -1
            else:
                location = self.robot.base.get_location()
                try:
                    idx = plan.index(location[0])
                except Exception:
                    idx = -1

            if idx >= 0:
                self.accepted_plan = plan[idx:]
                if len(self.plan) >= 3:
                    if len(self.accepted_plan) < 3 or self.accepted_plan[2] != self.plan[2]:
                        self.robot.base.cancel_forward_flag()
                self.accepted_transitions = plan_transitions[idx:]
                self.accepted_plan_end = plan_end
                self.accepted_reply = timestamp
                self._send_nav_request(ack_only=True)

            self._cv.notifyAll()

    def _send_nav_request(self, location=None, transition=None, ack_only=False):
        """
        Provide `transition` (j1, j2) instead of `location` if AGV has departed its location.
        """
        if location:
            if self.__nav_request_location[1] == location[0]:
                location = self.__nav_request_location[:1] + location
            else:
                location = (None,) + location
        elif transition:
            location = transition + (None,)
        else:
            location = self.__nav_request_location
        self.__nav_request_location = location

        self.robot.fms_manager.send_navigation_request_async({
            'timestamp': time.time(),
            'ack_only': ack_only,
            'dp': self.robot.base.get_dimension_profile(),
            'start': self.start,
            'end': self.end,
            'reverse': self.traverse_reversely,
            'location': location,
            'accepted_reply': self.accepted_reply,
            'seen_reply': self.seen_reply,
        })

    def _plan_path(self, location):
        plan, transitions = self.robot.models.find_shortest_constrained_path(location, self.end, self.traverse_reversely)
        if not plan:
            self.fail('Unable to find a valid path to destination.')
        if self.robot.fms_manager:
            return

        self.plan = plan
        self.transitions = transitions
        self.plan_end = self.end
        self._pre_handle_plan(location)

    def _pre_handle_plan(self, location):
        # Extract all upcoming plan paths
        count = len(self.plan)
        self.next_motions = [Goal.MOTION_IDLE] * count
        self.transition_triggers = [None] * count  # Transition trigger at last junction is not pre-processed
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

            if self.is_teleport(cur_path):
                teleport = self.robot.models.teleports[cur_path['teleport']]
                next_transition_start_heading = self.robot.models.get_location_from_station(teleport['end'])[1]
                self.teleport_plan.append(count - i)  # 1-indexed in reverse direction
                continue

            # Current heading transition (heading is referring to robot heading instead of path heading)
            current_transition_start_heading = next_transition_start_heading
            current_transition_end_heading = cur_path['direction']
            next_transition_start_heading = (cur_path['direction'] + cur_path['transform']) % 4

            reverse = self.is_reverse(cur_path)
            if reverse:
                current_transition_end_heading = (current_transition_end_heading + 2) % 4
                next_transition_start_heading = (next_transition_start_heading + 2) % 4

            # Look for transition trigger at current junction (prev_p)
            transition_trigger_obj = None
            transition_triggers = self.robot.models.transition_triggers.get(prev_p, [])
            cur_location = (prev_p, current_transition_start_heading)

            for transition in transition_triggers:
                if current_transition_start_heading != transition['previous_heading'] or current_transition_end_heading != transition['next_heading']:
                    continue
                if self._is_transition_applicable(transition, cur_location, reverse, False):
                    transition_trigger_obj = transition
                    break

            if transition_trigger_obj:
                self.transition_triggers[i] = transition_trigger_obj
                if transition_trigger_obj['cancelNonStopTransition'] and self.is_nonstop(self.next_motions[i]):
                    self.next_motions[i] = Goal.MOTION_IDLE  # cancel non-stop motion

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
            self.transition_triggers.pop(0)
            self.transitions.pop(0)
            return self.plan.pop(0)

    def _teleport_pre(self, teleport, next_motion, steps):
        def _execute():
            rospy.loginfo('[Teleport Pre-action (%d)] Executing...', steps)
            sm_outcome = self.teleport_pre_sm.execute({})
            rospy.loginfo('[Teleport Pre-action (%d)] Outcome: %s', steps, sm_outcome)

            with self._lock:
                self.teleport_pre_sm = None
                self.teleport_pre_thread = None

        with self._lock:
            if self.teleport_pre_sm or self.teleport_pre_thread:
                return

            self.teleport_pre_sm = self._build_teleport_sm(teleport, next_motion, steps)
            if not self.teleport_pre_sm:
                return

            self.teleport_pre_thread = threading.Thread(target=_execute)
            self.teleport_pre_thread.start()

    def _teleport(self, teleport, next_motion):
        logged_once = False
        while not self.preempt_requested():
            with self._lock:
                if not self.teleport_pre_thread or not self.teleport_pre_thread.is_alive():
                    break
            if not logged_once:
                logged_once = True
                rospy.loginfo('[Teleport Action] Waiting pre-action to complete...')
            rospy.sleep(1.0)

        while self.is_paused():
            self.robot.panel.toggle_led_paused(True)
            rospy.sleep(0.2)
            if self.preempt_requested():
                return 'Preempted'

        self.teleport_sm = self._build_teleport_sm(teleport, next_motion)
        if not self.teleport_sm:
            return
        self.robot.inner_sm = self.teleport_sm

        if self.preempt_requested():
            self.teleport_sm = None
            self.robot.inner_sm = None
            return 'Preempted'

        rospy.loginfo('[Teleport Action] Executing...')
        sm_outcome = self.teleport_sm.execute({})
        rospy.loginfo('[Teleport Action] Outcome: %s', sm_outcome)
        self.teleport_sm = None
        self.robot.inner_sm = None

        if sm_outcome in ['Preempted', 'Aborted']:
            return 'Preempted'

    def _build_teleport_sm(self, teleport, next_motion, steps=0):
        action = teleport['preAction'] if steps else teleport['action']
        if not action:
            return

        params = {
            '${start}': teleport['start'],
            '${end}': teleport['end'],
            '${auto_reset_agv_position}': teleport['autoResetAgvPosition'],
            '${next_motion}': next_motion,
            '${pre_steps}': steps,
            '${distance_cost}': teleport['distance'],
        }

        tag = 'Teleport Pre-action' if steps else 'Teleport Action'
        try:
            with self.robot.models.read_lock:
                return state_factory(self.robot, action['skillId'], action['params'], params)
        except StateConstructError as ex:
            rospy.logerr('[%s] StateConstructError: %s' % (tag, ex))
            raise
        except Exception as ex:
            rospy.logerr('[%s] Uncaught exception: %s' % (tag, ex))
            raise

    def _transition(self, transition, next_motion, is_start):
        if transition is None:
            return

        self.transition_sm = self._build_transition_sm(transition, next_motion, is_start)
        if not self.transition_sm:
            return
        self.robot.inner_sm = self.transition_sm

        if self.preempt_requested():
            self.transition_sm = None
            self.robot.inner_sm = None
            return 'Preempted'

        rospy.loginfo('[Transition Action] Executing...')
        sm_outcome = self.transition_sm.execute({})
        rospy.loginfo('[Transition Action] Outcome: %s', sm_outcome)
        self.transition_sm = None
        self.robot.inner_sm = None

        if sm_outcome in ['Preempted', 'Aborted']:
            return 'Preempted'

        if self.robot.base.get_location()[0] != self.plan[0]:
            self.fail('Transition action must not change the AGV\'s location.')

    def _build_transition_sm(self, transition, next_motion, is_start):
        action = transition['startAction'] if is_start else transition['endAction']
        if not action:
            return

        params = {
            '${start}': transition['start'],
            '${end}': transition['end'],
            '${next_motion}': next_motion,
        }

        tag = 'Transition Start Action' if is_start else 'Transition End Action'
        try:
            with self.robot.models.read_lock:
                return state_factory(self.robot, action['skillId'], action['params'], params)
        except StateConstructError as ex:
            rospy.logerr('[%s] StateConstructError: %s' % (tag, ex))
            raise
        except Exception as ex:
            rospy.logerr('[%s] Uncaught exception: %s' % (tag, ex))
            raise

    def _rotate_align(self, path_dir, reverse_next=False, force_align_sensor=None):
        while self.is_paused():
            self.robot.panel.toggle_led_paused(True)
            rospy.sleep(0.2)
            if self.preempt_requested():
                return

        assert path_dir >= 0 and path_dir < Direction.NA
        location = self.robot.base.get_location()
        diff_dir = (path_dir - location[1] + (2 if reverse_next else 0)) % 4
        if diff_dir == 0:
            return

        transition = self.transitions[0]
        if diff_dir == 1:
            fn = self.robot.base.rotate_right if transition != Motion.L3 else self.robot.base.rotate3q_left
        elif diff_dir == 2:
            fn = self.robot.base.uturn_left if transition != Motion.R2 else self.robot.base.uturn_right
        elif diff_dir == 3:
            fn = self.robot.base.rotate_left if transition != Motion.R3 else self.robot.base.rotate3q_right

        rotate_constraints = {
            'enable_sensor': True,
            'rotate_align_sensor': force_align_sensor if force_align_sensor is not None else int(reverse_next),
        }
        fn(rotate_constraints)

        if self.is_paused():
            rospy.sleep(0.2)
            self.robot.base.pause()
        self.wait_for_result()
        return True

    def _move_straight(self, path, next_motion, next_speed, enable_sensor, reverse):
        rfid_location = 1 if reverse else 0
        self.robot.rfid._reset_rfid(rfid_location)

        forward_constraints = {
            'speed': self._get_path_speed(path),
            'next_motion': next_motion,
            'next_speed': next_speed,
            'enable_sensor': enable_sensor,
        }
        straight = self.is_straight(path)
        branch = path['branch']
        rv_branch = path['rv_branch']

        while self.is_paused():
            self.robot.panel.toggle_led_paused(True)
            rospy.sleep(0.2)
            if self.preempt_requested():
                return

        if not reverse:
            if branch == PathBranch.RIGHT:
                self.robot.base.branch_forward_right(forward_constraints, straight=straight)
            elif branch == PathBranch.LEFT:
                self.robot.base.branch_forward_left(forward_constraints, straight=straight)
            elif rv_branch == PathBranch.RIGHT:  # consider reverse direction branch for smooth transversal
                self.robot.base.branch_forward_left(forward_constraints, straight=straight, track_dir=False)
            elif rv_branch == PathBranch.LEFT:
                self.robot.base.branch_forward_right(forward_constraints, straight=straight, track_dir=False)
            else:
                self.robot.base.forward(forward_constraints, straight=straight)
        else:
            if branch == PathBranch.RIGHT:
                self.robot.base.branch_reverse_right(forward_constraints, straight=straight)
            elif branch == PathBranch.LEFT:
                self.robot.base.branch_reverse_left(forward_constraints, straight=straight)
            elif rv_branch == PathBranch.RIGHT:  # consider reverse direction branch for smooth transversal
                self.robot.base.branch_reverse_left(forward_constraints, straight=straight, track_dir=False)
            elif rv_branch == PathBranch.LEFT:
                self.robot.base.branch_reverse_right(forward_constraints, straight=straight, track_dir=False)
            else:
                self.robot.base.reverse(forward_constraints, straight=straight)

        if self.is_paused():
            rospy.sleep(0.2)
            self.robot.base.pause()
        self.wait_for_result()

        if self.preempt_requested():
            return 'Preempted'
        self._validate_rfid(rfid_location)

    def _stop_wait_traffic(self):
        self.robot.base.stop_wait_traffic()
        while not self.robot.base.wait_for_result(rospy.Duration(2)):
            self.robot.base.stop_wait_traffic()

    def _get_next_motion(self, e1, e2, transition):
        if e1[1] == self.end[0] and self.check_transition(e1, self.end[1], reverse=self.traverse_reversely, zero=True):
            # patch: FMS plan to swapping station while crossing final station
            e2 = (-1, -1, None)

        if e2[2]:
            next_motion = self._to_motion(transition)
            if not self.is_nonstop(next_motion):
                pass
            elif self.is_teleport(e2[2]):
                teleport = self.robot.models.teleports[e2[2]['teleport']]
                if not teleport['nonStopTransition']:
                    next_motion = Goal.MOTION_IDLE  # cancel non-stop motion
            elif not self.is_teleport(e1[2]):
                if self.is_reverse(e1[2]) != self.is_reverse(e2[2]):
                    next_motion = Goal.MOTION_IDLE

        elif e1[1] == self.home[0]:
            # disable next motion: final dock or waiting for traffic controller
            next_motion = Goal.MOTION_IDLE

        elif e1[1] == self.end[0] and self.plan_end == self.end:
            # last path to station
            next_motion = self._to_motion(transition)
            if self.is_nonstop(next_motion):
                next_motion = self.next_motion  # follow next_motion input parameter

        else:
            # waiting for traffic controller
            next_motion = self._to_motion(transition)
            if self.is_nonstop(next_motion):
                next_motion = Goal.MOTION_IDLE

        return next_motion

    def _to_motion(self, transition):
        # N.A, ZERO, L0, L1, L2, L3, R0, R1, R2, R3
        return (0, Goal.MOTION_NONSTOP, 0, Goal.MOTION_LEFT, 0, 0, 0, Goal.MOTION_RIGHT, 0, 0)[transition]

    def _get_path_speed(self, path):
        if self.is_teleport(path):
            return self.UNLIMITED_SPEED

        raw = path['speed']
        if isinstance(raw, six.string_types) and raw.startswith('${'):
            raw = self.robot.models.map_params_assoc[raw]['default']

        speed = float(raw)
        return speed if speed > 0 else self.UNLIMITED_SPEED

    def _validate_rfid(self, rfid_location):
        j, h = self.robot.base.get_location()

        rfid_string = ''
        flip_heading = False

        if rfid_location == 0:
            rfid_string = self.robot.rfid.front_rfid
            flip_heading = False
        else:
            rfid_string = self.robot.rfid.rear_rfid
            flip_heading = True

        if rfid_string:
            rfid = self.robot.models.get_location_from_rfid(rfid_string)

            if flip_heading:
                h = (h + Direction.NA / 2) % Direction.NA

            if rfid and j == rfid[0] and h == rfid[1]:
                return True

        elif (j, Direction.NA) not in self.robot.models.rfid_assoc.values():  # if j dont have rfid
            return True

        self.robot.base.stop(make_confused=False)
        self.wait_for_result()

        rfid_in_map = ''
        for r, l in self.robot.models.rfid_assoc.items():
            if j == l[0]:
                rfid_in_map = r
                break
        if rfid_in_map and h >= 0 and h < Direction.NA:
            rfid_in_map += ':' + ['N', 'E', 'S', 'W'][h]
        else:
            rfid_in_map = '(empty)'

        self.fail('RFID differs from AGV location. Read %s, supposed to be %s' % (
            rfid_string if rfid_string else '(empty)', rfid_in_map))


class ReverseNavigateTo(NavigateTo):
    traverse_reversely = True

    class Meta(NavigateTo.Meta):
        name = 'Reverse Navigate To'
        params = [dict(d, default=1) if d['name'] == 'align_station_type' else d for d in NavigateTo.Meta.params]

    def __str__(self):
        return 'Reverse navigating to station "%s"' % self.station


class SelectDimensionProfile(Skill):

    class Meta:
        name = 'Select Dimension Profile'
        params = [
            {
                'name': 'profile',
                'type': 'int',
                'description': 'Dimension profile for traffic collision checking (0=no payload, 1-5=payload 1-5)',
                'default': 0,
                'min': 0,
                'max': 5,
            },
        ]
        outcomes = ['Done']
        mutexes = ['traffic']

    def __str__(self):
        return 'Select dimension profile "%s"' % self.profile

    def execute(self, ud):
        rospy.loginfo('%s', self)
        self.robot.base.set_dimension_profile(self.profile)
        if self.robot.fms_manager and not getattr(self.robot, 'inner_sm', None):
            if self.robot.fms_manager.has_occupation_request():
                self.robot.fms_manager.send_occupation_request_async()
        return 'Done'
