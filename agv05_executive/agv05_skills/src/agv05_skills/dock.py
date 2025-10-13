from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import agv05_msgs.msg
import math
import rospy

from .mixin import CancelForwardFlagOnPreempt

Result = agv05_msgs.msg.NavxActionResult


class DockMixin(CancelForwardFlagOnPreempt, Skill):
    reverse = False
    base_method = ''
    base_motion = set()

    class Meta:
        name = 'Dock Mixin (trackless) (Extend Me Please~)'
        params = [
            {
                'name': 'marker_type',
                'type': 'str',
                'description': 'Marker type and sensor id, eg: reflector__laser1, vmarker__laser3',
            },
            {
                'name': 'target_x',
                'type': 'double',
                'description': 'X coordinate of the target (in meters) in the marker frame',
                'default': 1.0,
                'min': -5.0,
                'max': 5.0,
            },
            {
                'name': 'target_y',
                'type': 'double',
                'description': 'Y coordinate of the target (in meters) in the marker frame',
                'default': 0.0,
                'min': -5.0,
                'max': 5.0,
            },
            {
                'name': 'target_heading',
                'type': 'double',
                'description': 'Target heading (in degrees) in the marker frame',
                'default': 0.0,
                'min': 0.0,
                'max': 359.9,
            },
            {
                'name': 'alignment_distance',
                'type': 'double',
                'description': 'Distance (in meters) to keep a straight movement for alignment before docking to the target',
                'default': 0.5,
                'min': 0.0,
                'max': 5.0,
            },
            {
                'name': 'speed',
                'type': 'double',
                'description': 'Speed in m/s',
                'default': 0.2,
                'min': 0.01,
                'max': 3.0,
            },
            {
                'name': 'enable_sensor',
                'type': 'bool',
                'description': 'Enable obstacle sensor',
                'default': True,
            },
            {
                'name': 'next_motion',
                'type': 'int',
                'description': 'Next motion (0=Idle, 1=Non-Stop, 2=Rotate Left, 3=Rotate Right, 4=Non-Stop Bezier)',
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
            {
                'name': 'io_trigger_type',
                'type': 'int',
                'description': 'IO trigger type (0=disable, 1=level LO, 2=level HI, 3=edge LO-to-HI, 4=edge HI-to-LO)',
                'default': 0,
                'min': 0,
                'max': 4,
            },
            {
                'name': 'io_trigger_port',
                'type': 'int',
                'description': 'IO port to trigger the stop of movement (1-8)',
                'default': 1,
                'min': 1,
                'max': 8,
            },
            {
                'name': 'io_trigger_pin',
                'type': 'int',
                'description': 'IO pin to trigger the stop of movement (0-15)',
                'default': 0,
                'min': 0,
                'max': 15,
            },
            {
                'name': 'io_trigger_distance',
                'type': 'double',
                'description': 'IO trigger distance (m)',
                'default': 0,
                'min': 0,
                'max': 1,
            },
            {
                'name': 'error_io_trigger_type',
                'type': 'int',
                'description': 'Error IO trigger type (0=disable, 1=level LO, 2=level HI, 3=edge LO-to-HI, 4=edge HI-to-LO)',
                'default': 0,
                'min': 0,
                'max': 4,
            },
            {
                'name': 'error_io_trigger_port',
                'type': 'int',
                'description': 'IO port to trigger error (1-8)',
                'default': 1,
                'min': 1,
                'max': 8,
            },
            {
                'name': 'error_io_trigger_pin',
                'type': 'int',
                'description': 'IO pin to trigger error (0-15)',
                'default': 0,
                'min': 0,
                'max': 15,
            },
        ]
        outcomes = ['IO Detected', 'Target Reached', 'Marker Not Detected', 'Error Triggered']
        mutexes = ['base']

    def __str__(self):
        return self.Meta.name

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if not self.base_motion.issubset(self.robot.models.allowed_motions):
            for motion in self.base_motion:
                if motion not in self.robot.models.allowed_motions:
                    self.fail('"%s" motion is not allowed.' % motion.replace('_', ' ').title())

        target = self._compute_target()
        getattr(self.robot.base, self.base_method)(self.marker_type, *target, constraints={
            'speed': self.speed,
            'enable_sensor': self.enable_sensor,
            'next_motion': self.next_motion,
            'next_speed': self.next_speed,
            'sense_line': self.sense_line,
            'io_trigger_type': self.io_trigger_type,
            'io_trigger_port': self.io_trigger_port,
            'io_trigger_pin': self.io_trigger_pin,
            'next_distance': self.io_trigger_distance,
            'error_io_trigger_type': self.error_io_trigger_type,
            'error_io_trigger_port': self.error_io_trigger_port,
            'error_io_trigger_pin': self.error_io_trigger_pin,
        })
        self.wait_for_result()
        result = self.robot.base.get_result()
        if self.preempt_requested():
            return 'Preempted'
        elif result.result == Result.RESULT_SUCCESS:
            return 'Target Reached'
        elif result.result == Result.RESULT_IO_DETECTED:
            return 'IO Detected'
        elif result.result == Result.RESULT_ERROR_TRIGGERED:
            return 'Error Triggered'
        else:
            return 'Marker Not Detected'

    def _compute_target(self):
        target = {
            'x': self.target_x,
            'y': self.target_y,
            'theta': math.radians(self.target_heading),
        }
        return (target, self.alignment_distance)


class ForwardDock(DockMixin):
    base_method = 'forward_dock'
    base_motion = {'forward'}

    class Meta(DockMixin.Meta):
        name = 'Forward Dock (trackless)'

    def __str__(self):
        return 'Forward docking (trackless)'


class ReverseDock(DockMixin):
    reverse = True
    base_method = 'reverse_dock'
    base_motion = {'reverse'}

    class Meta(DockMixin.Meta):
        name = 'Reverse Dock (trackless)'

    def __str__(self):
        return 'Reverse docking (trackless)'


class OmniDock(DockMixin):
    base_method = 'omni_dock'
    base_motion = {'forward', 'reverse', 'rotate_left', 'rotate_right'}

    # provide defaults for the unused parameters
    alignment_distance = 0
    next_motion = 0
    next_speed = 0
    sense_line = 0
    io_trigger_type = 0
    io_trigger_port = 1
    io_trigger_pin = 0
    io_trigger_distance = 0
    error_io_trigger_type = 0
    error_io_trigger_port = 1
    error_io_trigger_pin = 0

    class Meta(DockMixin.Meta):
        name = 'Omni Dock (trackless)'
        params = DockMixin.Meta.params[:4] + DockMixin.Meta.params[5:7]
        outcomes = DockMixin.Meta.outcomes[1:3]

    def __str__(self):
        return 'Omni docking (trackless)'


class UndockMixin(DockMixin):

    # provide defaults for the unused parameters
    sense_line = 0
    io_trigger_type = 0
    io_trigger_port = 1
    io_trigger_pin = 0
    io_trigger_distance = 0
    error_io_trigger_type = 0
    error_io_trigger_port = 1
    error_io_trigger_pin = 0

    class Meta(DockMixin.Meta):
        params = DockMixin.Meta.params[:4] + [
            {
                'name': 'alignment_distance',
                'type': 'double',
                'description': 'Distance (in meters) to keep a straight movement during the initial undocking from the target',
                'default': 0.5,
                'min': 0.0,
                'max': 5.0,
            },
            {
                'name': 'end_x',
                'type': 'double',
                'description': 'X coordinate of the endpoint (in meters) in the marker frame',
                'default': 3.0,
                'min': -10.0,
                'max': 10.0,
            },
            {
                'name': 'end_y',
                'type': 'double',
                'description': 'Y coordinate of the endpoint (in meters) in the marker frame',
                'default': 0.0,
                'min': -10.0,
                'max': 10.0,
            },
            {
                'name': 'end_heading',
                'type': 'double',
                'description': 'Endpoint heading (in degrees) in the marker frame',
                'default': 0.0,
                'min': 0.0,
                'max': 359.9,
            },
        ] + DockMixin.Meta.params[5:9]

        outcomes = DockMixin.Meta.outcomes[1:3]

    def _compute_target(self):
        target = {
            'x': self.target_x,
            'y': self.target_y,
            'theta': math.radians(self.target_heading),
        }
        end = {
            'x': self.end_x,
            'y': self.end_y,
            'theta': math.radians(self.end_heading),
        }
        return (target, self.alignment_distance, end)


class ForwardUndock(UndockMixin):
    base_method = 'forward_undock'
    base_motion = {'forward'}

    class Meta(UndockMixin.Meta):
        name = 'Forward Undock (trackless)'

    def __str__(self):
        return 'Forward undocking (trackless)'


class ReverseUndock(UndockMixin):
    reverse = True
    base_method = 'reverse_undock'
    base_motion = {'reverse'}

    class Meta(UndockMixin.Meta):
        name = 'Reverse Undock (trackless)'

    def __str__(self):
        return 'Reverse undocking (trackless)'
