from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import agv05_msgs.msg
import math
import rospy

from .mixin import CancelForwardFlagOnPreempt

Result = agv05_msgs.msg.NavActionResult


class ForwardMixin(CancelForwardFlagOnPreempt, Skill):
    reverse = False
    base_method = ''
    base_motion = ''

    class Meta:
        name = 'Forward Mixin (Extend Me Please~)'
        params = [
            {
                'name': 'speed',
                'type': 'double',
                'description': 'Speed in m/s',
                'default': 0.4,
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
        ]
        outcomes = ['Done']
        mutexes = ['base']

    def __str__(self):
        return self.Meta.name

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.base_motion not in self.robot.models.allowed_motions:
            self.fail('"%s" motion is not allowed.' %
                self.base_motion.replace('_', ' ').title())
        self.robot.rfid._reset_rfid(1 if self.reverse else 0)
        getattr(self.robot.base, self.base_method)(self._get_constraints())
        self.wait_for_result()
        return self._get_outcome(self.robot.base.get_result())

    def _get_constraints(self):
        return {
            'line_follow_type': 0,
            'speed': self.speed,
            'enable_sensor': self.enable_sensor,
            'next_motion': self.next_motion,
            'next_speed': self.next_speed,
        }

    def _get_outcome(self, result):
        return 'Preempted' if self.preempt_requested() else 'Done'


class ReverseMixin(ForwardMixin):
    reverse = True


class Forward(ForwardMixin):
    base_method = 'forward'
    base_motion = 'forward'

    class Meta(ForwardMixin.Meta):
        name = 'Forward'

    def __str__(self):
        return 'Forwarding'


class Reverse(ReverseMixin):
    base_method = 'reverse'
    base_motion = 'reverse'

    class Meta(ReverseMixin.Meta):
        name = 'Reverse'

    def __str__(self):
        return 'Reversing'


class BranchForwardLeft(ForwardMixin):
    base_method = 'branch_forward_left'
    base_motion = 'forward'

    class Meta(ForwardMixin.Meta):
        name = 'Branch Forward Left'

    def __str__(self):
        return 'Forwarding left'


class BranchForwardRight(ForwardMixin):
    base_method = 'branch_forward_right'
    base_motion = 'forward'

    class Meta(ForwardMixin.Meta):
        name = 'Branch Forward Right'

    def __str__(self):
        return 'Forwarding right'


class BranchReverseLeft(ReverseMixin):
    base_method = 'branch_reverse_left'
    base_motion = 'reverse'

    class Meta(ReverseMixin.Meta):
        name = 'Branch Reverse Left'

    def __str__(self):
        return 'Reversing left'


class BranchReverseRight(ReverseMixin):
    base_method = 'branch_reverse_right'
    base_motion = 'reverse'

    class Meta(ReverseMixin.Meta):
        name = 'Branch Reverse Right'

    def __str__(self):
        return 'Reversing right'


class ForwardByDistance(ForwardMixin):
    base_method = 'forward'
    base_motion = 'forward'

    class Meta(ForwardMixin.Meta):
        name = 'Forward (by distance)'
        params = ForwardMixin.Meta.params + [
            {
                'name': 'distance',
                'type': 'double',
                'description': 'Distance in m',
                'min': 0.01,
                'max': 100.0,
            },
            {
                'name': 'line_follow_type',
                'type': 'int',
                'description': 'Type of line follow feedback (0=Track, 1=Odom-1D, 2=Odom-2D)',
                'default': 0,
                'min': 0,
                'max': 2,
            },
            {
                'name': 'after_junction',
                'type': 'bool',
                'description': 'Measure distance only after Track junction',
                'default': False,
            },
        ]

    def __str__(self):
        return 'Forwarding (by distance)'

    def _get_constraints(self):
        return {
            'line_follow_type': self.line_follow_type,
            'speed': self.speed,
            'enable_sensor': self.enable_sensor,
            'next_motion': self.next_motion,
            'next_speed': self.next_speed,
            'distance': -self.distance if self.after_junction and self.line_follow_type in [0] else self.distance,
        }


class ReverseByDistance(ForwardByDistance):
    reverse = True
    base_method = 'reverse'
    base_motion = 'reverse'

    class Meta(ForwardByDistance.Meta):
        name = 'Reverse (by distance)'

    def __str__(self):
        return 'Reversing (by distance)'


class ForwardWaitIo(ForwardMixin):
    base_method = 'forward'
    base_motion = 'forward'

    class Meta(ForwardMixin.Meta):
        name = 'Forward (wait IO)'
        params = ForwardMixin.Meta.params + [
            {
                'name': 'io_trigger_type',
                'type': 'int',
                'description': 'IO trigger type (0=disable, 1=level LO, 2=level HI, 3=edge LO-to-HI, 4=edge HI-to-LO)',
                'default': 2,
                'min': 0,
                'max': 4,
                'version': 1,
            },
            {
                'name': 'io_trigger_port',
                'type': 'int',
                'description': 'IO port to trigger the stop of movement (1-8)',
                'default': 1,
                'min': 1,
                'max': 8,
                'version': 1,
            },
            {
                'name': 'io_trigger_pin',
                'type': 'int',
                'description': 'IO pin to trigger the stop of movement (0-15)',
                'default': 0,
                'min': 0,
                'max': 15,
                'version': 1,
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
        outcomes = ['IO Detected', 'Junction Detected', 'Error Triggered']

    def __str__(self):
        return 'Forwarding (wait IO)'

    def _get_constraints(self):
        return {
            'line_follow_type': 0,
            'speed': self.speed,
            'enable_sensor': self.enable_sensor,
            'next_motion': self.next_motion,
            'next_speed': self.next_speed,
            'io_trigger_type': self.io_trigger_type,
            'io_trigger_port': self.io_trigger_port,
            'io_trigger_pin': self.io_trigger_pin,
            'next_distance': self.io_trigger_distance,
            'error_io_trigger_type': self.error_io_trigger_type,
            'error_io_trigger_port': self.error_io_trigger_port,
            'error_io_trigger_pin': self.error_io_trigger_pin,
        }

    def _get_outcome(self, result):
        if self.preempt_requested():
            return 'Preempted'
        elif result.result == Result.RESULT_SUCCESS:
            return 'Junction Detected'
        elif result.result == Result.RESULT_IO_DETECTED:
            return 'IO Detected'
        else:
            return 'Error Triggered'


class ReverseWaitIo(ForwardWaitIo):
    reverse = True
    base_method = 'reverse'
    base_motion = 'reverse'

    class Meta(ForwardWaitIo.Meta):
        name = 'Reverse (wait IO)'

    def __str__(self):
        return 'Reversing (wait IO)'


class ForwardWaitIoByDistance(ForwardByDistance, ForwardWaitIo, ForwardMixin):

    class Meta(ForwardByDistance.Meta, ForwardWaitIo.Meta, ForwardMixin.Meta):
        name = 'Forward (wait IO) (by distance)'
        params = ForwardByDistance.Meta.params + ForwardWaitIo.Meta.params[len(ForwardMixin.Meta.params):]
        outcomes = ['IO Detected', 'Done', 'Error Triggered']

    def __str__(self):
        return 'Forwarding (wait IO) (by distance)'

    def _get_constraints(self):
        return {
            'line_follow_type': self.line_follow_type,
            'speed': self.speed,
            'enable_sensor': self.enable_sensor,
            'next_motion': self.next_motion,
            'next_speed': self.next_speed,
            'distance': -self.distance if self.after_junction and self.line_follow_type in [0] else self.distance,
            'io_trigger_type': self.io_trigger_type,
            'io_trigger_port': self.io_trigger_port,
            'io_trigger_pin': self.io_trigger_pin,
            'next_distance': self.io_trigger_distance,
            'error_io_trigger_type': self.error_io_trigger_type,
            'error_io_trigger_port': self.error_io_trigger_port,
            'error_io_trigger_pin': self.error_io_trigger_pin,
        }

    def _get_outcome(self, result):
        if self.preempt_requested():
            return 'Preempted'
        elif result.result == Result.RESULT_SUCCESS:
            return 'Done'
        elif result.result == Result.RESULT_IO_DETECTED:
            return 'IO Detected'
        else:
            return 'Error Triggered'


class ReverseWaitIoByDistance(ForwardWaitIoByDistance):
    reverse = True
    base_method = 'reverse'
    base_motion = 'reverse'

    class Meta(ForwardWaitIoByDistance.Meta):
        name = 'Reverse (wait IO) (by distance)'

    def __str__(self):
        return 'Reversing (wait IO) (by distance)'


class RotateMixin(Skill):
    base_method = ''
    base_motion = ''

    class Meta:
        name = 'Rotate Mixin (Extend Me Please~)'
        params = [
            {
                'name': 'enable_sensor',
                'type': 'bool',
                'description': 'Enable obstacle sensor',
                'default': True,
            },
            {
                'name': 'alignment_sensor',
                'type': 'int',
                'description': 'Alignment Sensor, front (0), rear (1)',
                'default': 0,
                'min': 0,
                'max': 1,
            },
        ]
        outcomes = ['Done']
        mutexes = ['base']

    def __str__(self):
        return self.Meta.name

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.base_motion not in self.robot.models.allowed_motions:
            self.fail('"%s" motion is not allowed.' %
                self.base_motion.replace('_', ' ').title())
        getattr(self.robot.base, self.base_method)(self._get_constraints())
        self.robot.base.wait_for_result()
        return 'Preempted' if self.preempt_requested() else 'Done'

    def _get_constraints(self):
        return {
            'line_follow_type': 0,
            'enable_sensor': self.enable_sensor,
            'distance': 0,
            'rotate_align_sensor': self.alignment_sensor,
        }


class RotateByAngle(RotateMixin):

    class Meta(RotateMixin.Meta):
        name = 'Rotate (by angle)'
        params = RotateMixin.Meta.params[:1] + [
            {
                'name': 'angle',
                'type': 'int',
                'description': 'Angle (in degrees) to rotate',
                'min': 10,
                'max': 350,
            },
            {
                'name': 'clockwise',
                'type': 'bool',
                'description': 'Direction of rotation',
                'default': False,
            },
        ]

    def __str__(self):
        return self.Meta.name

    def execute(self, ud):
        self.base_method = 'search_line_right' if self.clockwise else 'search_line_left'
        self.base_motion = 'rotate_right' if self.clockwise else 'rotate_left'
        return super(RotateByAngle, self).execute(ud)

    def _get_constraints(self):
        angle = math.radians(self.angle)
        return {
            'line_follow_type': 2,
            'enable_sensor': self.enable_sensor,
            'distance': angle,
        }


class RotateLeft(RotateMixin):
    base_method = 'rotate_left'
    base_motion = 'rotate_left'

    class Meta(RotateMixin.Meta):
        name = 'Rotate Left'

    def __str__(self):
        return 'Rotating left'


class RotateRight(RotateMixin):
    base_method = 'rotate_right'
    base_motion = 'rotate_right'

    class Meta(RotateMixin.Meta):
        name = 'Rotate Right'

    def __str__(self):
        return 'Rotating right'


class UturnLeft(RotateMixin):
    base_method = 'uturn_left'
    base_motion = 'uturn_left'

    class Meta(RotateMixin.Meta):
        name = 'U-turn Left'

    def __str__(self):
        return 'U-turning left'


class UturnRight(RotateMixin):
    base_method = 'uturn_right'
    base_motion = 'uturn_right'

    class Meta(RotateMixin.Meta):
        name = 'U-turn Right'

    def __str__(self):
        return 'U-turning right'


class Rotate3qLeft(RotateMixin):
    base_method = 'rotate3q_left'
    base_motion = 'uturn_left'

    class Meta(RotateMixin.Meta):
        name = 'Rotate 3-Q Left'

    def __str__(self):
        return 'Rotating three-quarter left'


class Rotate3qRight(RotateMixin):
    base_method = 'rotate3q_right'
    base_motion = 'uturn_right'

    class Meta(RotateMixin.Meta):
        name = 'Rotate 3-Q Right'

    def __str__(self):
        return 'Rotating three-quarter right'


class SearchLineLeft(RotateMixin):
    base_method = 'search_line_left'
    base_motion = 'rotate_left'

    class Meta(RotateMixin.Meta):
        name = 'Search Line Left'
        params = RotateMixin.Meta.params + [
            {
                'name': 'angle',
                'type': 'int',
                'description': 'Angle (in degrees) to line',
                'default': 30,
                'min': 0,
                'max': 350,
            },
        ]
        outcomes = ['LineFound', 'LineNotFound']

    def __str__(self):
        return 'Searching line left'

    def execute(self, ud):
        RotateMixin.execute(self, ud)
        result = self.robot.base.get_result()
        if self.preempt_requested():
            return 'Preempted'
        elif result.result == Result.RESULT_SUCCESS:
            return 'LineFound'
        else:
            return 'LineNotFound'

    def _get_constraints(self):
        angle = math.radians(self.angle)
        return {
            'line_follow_type': 0,
            'enable_sensor': self.enable_sensor,
            'distance': angle,
            'rotate_align_sensor': self.alignment_sensor,
        }


class SearchLineRight(SearchLineLeft):
    base_method = 'search_line_right'
    base_motion = 'rotate_right'

    class Meta(SearchLineLeft.Meta):
        name = 'Search Line Right'

    def __str__(self):
        return 'Searching line right'


class SelectNavProfile(Skill):

    class Meta:
        name = "Select Navigation Profile"
        params = [
            {
                'name': 'profile',
                'type': 'int',
                'description': 'Profile for navigation (1-5)',
                'default': 1,
                'min': '1',
                'max': '5',
            },
        ]
        outcomes = ['Done']
        mutexes = ['base']

    def __str__(self):
        return "Select navigation profile"

    def execute(self, ud):
        rospy.loginfo('%s', self)
        self.robot.base.select_nav_profile(self.profile)
        self.robot.base.wait_for_result()
        return 'Preempted' if self.preempt_requested() else 'Done'


class SelectLaserProfile(Skill):

    class Meta:
        name = "Select Laser Profile"
        params = [
            {
                'name': 'laser_profile',
                'type': 'int',
                'description': 'Laser profile for navigation (1-10)',
                'default': 1,
                'min': '1',
                'max': '10',
            },
        ]
        outcomes = ['Done']
        mutexes = ['base']

    def __str__(self):
        return "Select laser profile"

    def execute(self, ud):
        rospy.loginfo('%s', self)
        self.robot.base.select_laser_profile(self.laser_profile)
        self.robot.base.wait_for_result()
        return 'Preempted' if self.preempt_requested() else 'Done'


class ScanLaserArea(Skill):

    class Meta:
        name = "Scan Laser Area"
        params = [
            {
                'name': 'laser_area',
                'type': 'int',
                'description': 'Area for Laser Scanning (1-31)',
                'default': 1,
                'min': '1',
                'max': '31',
            },
        ]
        outcomes = ['ObjectDetected', 'ObjectNotDetected']
        mutexes = ['base']

    def __str__(self):
        return "Scan laser area"

    def execute(self, ud):
        rospy.loginfo('%s', self)
        self.robot.base.scan_laser_area(self.laser_area)
        self.robot.base.wait_for_result()
        result = self.robot.base.get_result()
        if self.preempt_requested():
            return 'Preempted'
        elif result.result == Result.RESULT_SUCCESS:
            return 'ObjectDetected'
        else:
            return 'ObjectNotDetected'


ScanLaserProfile = ScanLaserArea
