from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
from agv05_webserver.systemx.models import Heading
import geometry_msgs.msg
import math
import rospy
import tf

from .dock import DockMixin
from .nav_x import ForwardX, ReverseX, BezierForwardX, BezierReverseX
from .plan_x import ResetAgvPositionTrackerX


class DynamicXMixin(object):

    class Meta:
        name = 'Dynamic Mixin (trackless) (Extend Me Please~)'
        params = [
            {
                'name': 'goal_tolerance',
                'type': 'double',
                'description': 'Goal tolerance (in meter) from endpoint (-1: default mid, -2: default goal)',
                'default': -1.0,
                'min': -2.0,
                'max': 100.0,
            },
        ]

    def __str__(self):
        return 'Dynamic ' + super(DynamicXMixin, self).__str__()

    def _get_constraints(self):
        ret = super(DynamicXMixin, self)._get_constraints()
        ret['goal_tolerance'] = self.goal_tolerance
        return ret


class GoToWaypointMixin(Skill):

    class Meta(DynamicXMixin.Meta):
        params = [
            {
                'name': 'speed',
                'type': 'double',
                'description': 'Speed in m/s',
                'default': 0.7,
                'min': 0.01,
                'max': 2.0,
            },
            {
                'name': 'enable_sensor',
                'type': 'bool',
                'description': 'Enable obstacle sensor',
                'default': True,
            },
            {
                'name': 'forward',
                'type': 'bool',
                'description': 'Forward motion',
                'default': True,
            },
        ] + DynamicXMixin.Meta.params
        outcomes = ['Success', 'Failed']
        mutexes = ['base']

    def __str__(self):
        return self.Meta.name

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.forward:
            base_method = self.robot.base.dynamic_waypoint_forward
            base_motion = 'forward'
        else:
            base_method = self.robot.base.dynamic_waypoint_reverse
            base_motion = 'reverse'

        if 'dynamic' not in self.robot.models.allowed_motions:
            self.fail('"Dynamic" motion is not allowed.')
        if base_motion not in self.robot.models.allowed_motions:
            self.fail('"%s" motion is not allowed.' %
                base_motion.replace('_', ' ').title())

        end_heading = self._compute_end_heading()
        if self.preempt_requested():
            return 'Preempted'
        elif not end_heading:
            return 'Failed'

        base_method(*end_heading, constraints=self._get_constraints())
        self.robot.base.wait_for_result()

        return 'Preempted' if self.preempt_requested() else 'Success'

    def _compute_end_heading(self):
        raise NotImplementedError()

    def _get_constraints(self):
        return {
            'speed': self.speed,
            'goal_tolerance': self.goal_tolerance,
            'enable_sensor': self.enable_sensor,
        }


class GoToWaypoint(GoToWaypointMixin):

    class Meta(GoToWaypointMixin.Meta):
        name = 'Go To Waypoint'
        params = [
            {
                'name': 'x',
                'type': 'double',
                'description': 'X coordinate (in meter)',
            },
            {
                'name': 'y',
                'type': 'double',
                'description': 'Y coordinate (in meter)',
            },
            {
                'name': 'heading',
                'type': 'int',
                'description': 'Heading (in degree). Ignored if 0',
                'default': 0,
                'min': 0,
                'max': 360,
            },
        ] + GoToWaypointMixin.Meta.params

    def __str__(self):
        return 'Going to waypoint at (%s, %s, %s)' % (self.x, self.y, self.heading)

    def _compute_end_heading(self):
        end = {'x': self.x, 'y': self.y}
        return (end, self.heading)


class GoToStationWaypoint(GoToWaypointMixin):

    class Meta(GoToWaypointMixin.Meta):
        name = 'Go To Station Waypoint'
        params = [
            {
                'name': 'station',
                'type': 'Station',
                'description': 'Station',
            },
        ] + GoToWaypointMixin.Meta.params

    def __str__(self):
        return 'Going to station waypoint "%s"' % self.station

    def execute(self, ud):
        ret = super(GoToStationWaypoint, self).execute(ud)
        if ret == 'Success':
            ResetAgvPositionTrackerX(self.robot, station=self.station).execute({})
        return 'Preempted' if self.preempt_requested() else 'Success'

    def _compute_end_heading(self):
        j, h = self.robot.models.get_location_from_station(self.station)
        end = self.robot.models.get_junction(j)
        if h == Heading.NA:
            h = 0
        elif h <= 0:
            h += 360
        return (end, h)


class GoToMarkerWaypoint(GoToWaypointMixin):

    class Meta(GoToWaypointMixin.Meta):
        name = 'Go To Marker Waypoint'
        params = DockMixin.Meta.params[:3] + [
            {
                'name': 'target_heading',
                'type': 'int',
                'description': 'Target heading (in degrees) in the marker frame. Ignored if 0',
                'default': 0,
                'min': 0,
                'max': 360,
            },
            {
                'name': 'timeout',
                'type': 'int',
                'description': 'Timeout duration in seconds.',
                'default': 3,
                'min': -1,
                'max': 9999,
            },
        ] + GoToWaypointMixin.Meta.params

    def __str__(self):
        return 'Going to waypoint at (%s, %s, %s) of marker' % (self.target_x, self.target_y, self.target_heading)

    def _compute_end_heading(self):
        self.robot.base.publish_marker_type(self.marker_type)
        pose = self._get_target_pose()
        self.robot.base.publish_marker_type('')

        if not pose:
            return False

        end = {'x': pose.x, 'y': pose.y}
        heading = 0
        if self.target_heading > 0:
            heading = math.degrees(pose.theta)
            while heading <= 0.0:
                heading += 360.0
        return (end, heading)

    def _get_target_pose(self):
        target_pose_stamped = geometry_msgs.msg.PoseStamped()
        target_pose_stamped.header.frame_id = 'marker'
        target_pose_stamped.header.stamp = rospy.Time()

        target_pose_stamped.pose.position.x = self.target_x
        target_pose_stamped.pose.position.y = self.target_y
        target_pose_stamped.pose.position.z = 0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(self.target_heading))
        target_pose_stamped.pose.orientation.x = quaternion[0]
        target_pose_stamped.pose.orientation.y = quaternion[1]
        target_pose_stamped.pose.orientation.z = quaternion[2]
        target_pose_stamped.pose.orientation.w = quaternion[3]

        start_time = rospy.get_time()
        while not self.preempt_requested():
            if self.timeout is not None:
                if rospy.get_time() - start_time > self.timeout:
                    break
            try:
                t = self.robot.base.tf_listener.getLatestCommonTime('map', 'marker')
                if start_time - t.to_sec() <= 0.5:  # ensure transform is recent
                    # (trans, rot) = self.robot.base.tf_listener.lookupTransform('map', 'marker', rospy.Time())
                    target_pose_stamped.header.stamp = rospy.Time()
                    target_pose = self.robot.base.tf_listener.transformPose('map', target_pose_stamped).pose
                    quaternion = (
                        target_pose.orientation.x,
                        target_pose.orientation.y,
                        target_pose.orientation.z,
                        target_pose.orientation.w,
                    )
                    rpy = tf.transformations.euler_from_quaternion(quaternion)
                    pose = geometry_msgs.msg.Pose2D()
                    pose.x = target_pose.position.x
                    pose.y = target_pose.position.y
                    pose.theta = rpy[2]
                    return pose
            except Exception:
                pass

            rospy.sleep(0.2)


class DynamicLineForwardX(DynamicXMixin, ForwardX):
    base_method = 'dynamic_line_forward'

    class Meta(ForwardX.Meta):
        name = 'Dynamic ' + ForwardX.Meta.name
        params = ForwardX.Meta.params + DynamicXMixin.Meta.params


class DynamicLineReverseX(DynamicXMixin, ReverseX):
    base_method = 'dynamic_line_reverse'

    class Meta(ReverseX.Meta):
        name = 'Dynamic ' + ReverseX.Meta.name
        params = ReverseX.Meta.params + DynamicXMixin.Meta.params


class DynamicBezierForwardX(DynamicXMixin, BezierForwardX):
    base_method = 'dynamic_bezier_forward'

    class Meta(BezierForwardX.Meta):
        name = 'Dynamic ' + BezierForwardX.Meta.name
        params = BezierForwardX.Meta.params + DynamicXMixin.Meta.params


class DynamicBezierReverseX(DynamicXMixin, BezierReverseX):
    base_method = 'dynamic_bezier_reverse'

    class Meta(BezierReverseX.Meta):
        name = 'Dynamic ' + BezierReverseX.Meta.name
        params = BezierReverseX.Meta.params + DynamicXMixin.Meta.params
