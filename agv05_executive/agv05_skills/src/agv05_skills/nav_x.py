from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import agv05_msgs.msg
import math
import rospy

from .mixin import CancelForwardFlagOnPreempt

Result = agv05_msgs.msg.NavxActionResult


class ForwardXMixin(CancelForwardFlagOnPreempt, Skill):
    reverse = False
    base_method = ''
    base_motion = ''

    class Meta:
        name = 'Forward Mixin (trackless) (Extend Me Please~)'
        params = [
            {
                'name': 'distance',
                'type': 'double',
                'description': 'Distance (in meters) to move forward',
                'min': 0.1,
                'max': 100.0,
            },
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
        mutexes = ['base']

    def __str__(self):
        return self.Meta.name

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.base_motion not in self.robot.models.allowed_motions:
            self.fail('"%s" motion is not allowed.' %
                self.base_motion.replace('_', ' ').title())

        pose = self.robot.base.get_pose()
        if not pose:
            self.fail('Robot position is unknown.')
        path = self._compute_path(pose)

        mt_pose = self.robot.base.map_tracker.get_pose()
        if mt_pose:
            mt_path = self._compute_path(mt_pose)
            next_location = self.robot.base.map_tracker.peek_next_location(forward=not self.reverse, hint=mt_path[1])
            if next_location:
                end = self.robot.models.get_junction(next_location[0])
                mt_path[1]['x'] = end['x']
                mt_path[1]['y'] = end['y']
                path = mt_path

        getattr(self.robot.base, self.base_method)(*path, constraints=self._get_constraints())
        self.wait_for_result()
        return self._get_outcome(self.robot.base.get_result())

    def _compute_path(self, pose):
        raise NotImplementedError()

    def _get_constraints(self):
        return {
            'speed': self.speed,
            'enable_sensor': self.enable_sensor,
            'next_motion': self.next_motion,
            'next_speed': self.next_speed,
            'sense_line': self.sense_line,
        }

    def _get_outcome(self, result):
        return 'Preempted' if self.preempt_requested() else 'Done'


class ReverseXMixin(ForwardXMixin):
    reverse = True


class ForwardX(ForwardXMixin):
    base_method = 'move_forward'
    base_motion = 'forward'

    class Meta(ForwardXMixin.Meta):
        name = 'Forward (trackless)'

    def __str__(self):
        return 'Forwarding (trackless)'

    def _compute_path(self, pose):
        start = {'x': pose.x, 'y': pose.y}
        end = {
            'x': pose.x + self.distance * math.cos(pose.theta),
            'y': pose.y + self.distance * math.sin(pose.theta),
        }
        return (start, end)


class ReverseX(ReverseXMixin):
    base_method = 'move_reverse'
    base_motion = 'reverse'

    class Meta(ReverseXMixin.Meta):
        name = 'Reverse (trackless)'
        params = [
            {
                'name': 'distance',
                'type': 'double',
                'description': 'Distance (in meters) to move reverse',
                'min': 0.1,
                'max': 100.0,
            },
        ] + ReverseXMixin.Meta.params[1:]

    def __str__(self):
        return 'Reversing (trackless)'

    def _compute_path(self, pose):
        start = {'x': pose.x, 'y': pose.y}
        end = {
            'x': pose.x - self.distance * math.cos(pose.theta),
            'y': pose.y - self.distance * math.sin(pose.theta),
        }
        return (start, end)


class BezierForwardX(ForwardXMixin):
    base_method = 'bezier_forward'
    base_motion = 'forward'

    class Meta(ForwardXMixin.Meta):
        name = 'Bezier Forward (trackless)'
        params = [
            {
                'name': 'end_x',
                'type': 'double',
                'description': 'X coordinate of endpoint (in meters) from current position',
                'min': 0.1,
                'max': 100.0,
            },
            {
                'name': 'end_y',
                'type': 'double',
                'description': 'Y coordinate of endpoint (in meters) from current position',
                'min': -100.0,
                'max': 100.0,
            },
            {
                'name': 'cp1_x',
                'type': 'double',
                'description': 'X coordinate of control point 1 (in meters) from current position',
                'min': 0.1,
                'max': 100.0,
            },
            {
                'name': 'cp1_y',
                'type': 'double',
                'description': 'Y coordinate of control point 1 (in meters) from current position',
                'min': -100.0,
                'max': 100.0,
            },
            {
                'name': 'cp2_x',
                'type': 'double',
                'description': 'X coordinate of control point 2 (in meters) from current position',
                'min': 0.1,
                'max': 100.0,
            },
            {
                'name': 'cp2_y',
                'type': 'double',
                'description': 'Y coordinate of control point 2 (in meters) from current position',
                'min': -100.0,
                'max': 100.0,
            },
        ] + ForwardXMixin.Meta.params[1:]

    def __str__(self):
        return 'Bezier-Forwarding (trackless)'

    def _compute_path(self, pose):
        sin_th = math.sin(pose.theta)
        cos_th = math.cos(pose.theta)

        start = {'x': pose.x, 'y': pose.y}
        end = {
            'x': pose.x + cos_th * self.end_x - sin_th * self.end_y,
            'y': pose.y + sin_th * self.end_x + cos_th * self.end_y,
        }
        cp1 = {
            'x': pose.x + cos_th * self.cp1_x - sin_th * self.cp1_y,
            'y': pose.y + sin_th * self.cp1_x + cos_th * self.cp1_y,
        }
        cp2 = {
            'x': pose.x + cos_th * self.cp2_x - sin_th * self.cp2_y,
            'y': pose.y + sin_th * self.cp2_x + cos_th * self.cp2_y,
        }
        return (start, end, cp1, cp2)


class BezierReverseX(BezierForwardX):
    reverse = True
    base_method = 'bezier_reverse'
    base_motion = 'reverse'

    class Meta(ReverseXMixin.Meta):
        name = 'Bezier Reverse (trackless)'
        params = [
            {
                'name': 'end_x',
                'type': 'double',
                'description': 'X coordinate of endpoint (in meters) from current position',
                'min': -100.0,
                'max': -0.1,
            },
            {
                'name': 'end_y',
                'type': 'double',
                'description': 'Y coordinate of endpoint (in meters) from current position',
                'min': -100.0,
                'max': 100.0,
            },
            {
                'name': 'cp1_x',
                'type': 'double',
                'description': 'X coordinate of control point 1 (in meters) from current position',
                'min': -100.0,
                'max': -0.1,
            },
            {
                'name': 'cp1_y',
                'type': 'double',
                'description': 'Y coordinate of control point 1 (in meters) from current position',
                'min': -100.0,
                'max': 100.0,
            },
            {
                'name': 'cp2_x',
                'type': 'double',
                'description': 'X coordinate of control point 2 (in meters) from current position',
                'min': -100.0,
                'max': -0.1,
            },
            {
                'name': 'cp2_y',
                'type': 'double',
                'description': 'Y coordinate of control point 2 (in meters) from current position',
                'min': -100.0,
                'max': 100.0,
            },
        ] + ReverseXMixin.Meta.params[1:]

    def __str__(self):
        return 'Bezier-Reversing (trackless)'


class ForwardXWaitIo(ForwardX):

    class Meta(ForwardX.Meta):
        name = 'Forward (wait IO) (trackless)'
        params = ForwardX.Meta.params + [
            {
                'name': 'io_trigger_type',
                'type': 'int',
                'description': 'IO trigger type (0=disable, 1=level LO, 2=level HI, 3=edge LO-to-HI, 4=edge HI-to-LO)',
                'default': 2,
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
        outcomes = ['IO Detected', 'Done', 'Error Triggered']

    def __str__(self):
        return 'Forwarding (wait IO) (trackless)'

    def _get_constraints(self):
        return {
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


class ReverseXWaitIo(ReverseX, ForwardXWaitIo):

    class Meta(ForwardXWaitIo.Meta):
        name = 'Reverse (wait IO) (trackless)'
        params = [
            {
                'name': 'distance',
                'type': 'double',
                'description': 'Distance (in meters) to move reverse',
                'min': 0.1,
                'max': 100.0,
            },
        ] + ForwardXWaitIo.Meta.params[1:]

    def __str__(self):
        return 'Reversing (wait IO) (trackless)'


class RotateXMixin(Skill):
    base_method = ''
    base_motion = ''

    class Meta:
        name = 'Rotate Mixin (trackless) (ExtendMePlease~)'
        params = [
            {
                'name': 'angle',
                'type': 'int',
                'description': 'Angle (in degrees) to rotate counter-clockwise.',
                'min': 10,
                'max': 350,
            },
            {
                'name': 'enable_sensor',
                'type': 'bool',
                'description': 'Enable obstacle sensor',
                'default': True,
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

        pose = self.robot.base.get_pose()
        if not pose:
            self.fail('Robot position is unknown')

        mt_pose = self.robot.base.map_tracker.get_pose()
        if mt_pose:
            pose = mt_pose

        end = {'x': pose.x, 'y': pose.y}
        heading = self._compute_heading(pose)

        getattr(self.robot.base, self.base_method)(end, heading, {
            'enable_sensor': self.enable_sensor,
        })
        self.robot.base.wait_for_result()
        return 'Preempted' if self.preempt_requested() else 'Done'

    def _compute_heading(self, pose):
        raise NotImplementedError()


class RotateLeftX(RotateXMixin):
    base_method = 'free_rotate_left'
    base_motion = 'rotate_left'

    class Meta(RotateXMixin.Meta):
        name = 'Rotate Left (trackless)'

    def __str__(self):
        return 'Rotating left (trackless)'

    def _compute_heading(self, pose):
        heading = math.degrees(pose.theta) + self.angle
        if heading >= 360:
            heading -= 360
        return heading


class RotateRightX(RotateXMixin):
    base_method = 'free_rotate_right'
    base_motion = 'rotate_right'

    class Meta(RotateXMixin.Meta):
        name = 'Rotate Right (trackless)'
        params = [
            {
                'name': 'angle',
                'type': 'int',
                'description': 'Angle (in degrees) to rotate clockwise.',
                'min': 10,
                'max': 350,
            },
        ] + RotateXMixin.Meta.params[1:]

    def __str__(self):
        return 'Rotating right (trackless)'

    def _compute_heading(self, pose):
        heading = math.degrees(pose.theta) - self.angle
        if heading < 0:
            heading += 360
        return heading


class RotateAbsoluteX(Skill):

    class Meta:
        name = 'Rotate to Heading (trackless)'
        params = [
            {
                'name': 'heading',
                'type': 'int',
                'description': 'Target heading (in degrees), with respect to map',
                'min': 0,
                'max': 359,
            },
            {
                'name': 'enable_sensor',
                'type': 'bool',
                'description': 'Enable obstacle sensor',
                'default': True,
            },
        ]
        outcomes = ['Done']
        mutexes = ['base']

    def __str__(self):
        return self.Meta.name

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if not {'rotate_left', 'rotate_right'}.issubset(self.robot.models.allowed_motions):
            self.fail('"rotate_left" or "rotate_right" motion is not allowed.')

        pose = self.robot.base.get_pose()
        if not pose:
            self.fail('Robot position is unknown')

        end = {'x': pose.x, 'y': pose.y}
        diff = (self.heading - math.degrees(pose.theta)) % 360
        base_method = self.robot.base.free_rotate_right if diff > 180 else self.robot.base.free_rotate_left

        base_method(end, self.heading, {
            'enable_sensor': self.enable_sensor,
        })
        self.robot.base.wait_for_result()
        return 'Preempted' if self.preempt_requested() else 'Done'
