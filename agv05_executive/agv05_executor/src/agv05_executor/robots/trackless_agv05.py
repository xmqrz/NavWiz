from __future__ import absolute_import
from __future__ import unicode_literals

import agv05_msgs.msg
import dynamic_reconfigure.client
import geometry_msgs.msg
import math
import nav_msgs.srv
import rospy
import std_srvs.srv
import tf

from .agv05 import Agv05
from .base import TrackedBaseMixin, TracklessBase
from .robot import TracklessMixin
from .tracked_agv05 import TrackedAgv05Base
from ..models.map_tracker_x import MapTrackerX

Goal = agv05_msgs.msg.NavxActionGoal
NavControl = agv05_msgs.msg.NavControl
Pose2D = geometry_msgs.msg.Pose2D


class TracklessAgv05Base(TrackedBaseMixin, TracklessBase):
    manual_cmd_vel_topic = '/agv05/nav/manual_cmd_vel'
    action_server = '/navx_action'
    action_spec = agv05_msgs.msg.NavxActionAction
    map_tracker_class = MapTrackerX

    navx_control = '/agv05/navx/navx_control'
    set_map_service = '/set_map'
    reset_odom_service = '/reset_odom'
    tracked_methods = [
        'forward', 'reverse',
        'branch_forward_left', 'branch_forward_right',
        'branch_reverse_left', 'branch_reverse_right',
        'rotate_left', 'rotate_right',
        'uturn_left', 'uturn_right',
        'rotate3q_left', 'rotate3q_right',
        'search_line_left', 'search_line_right',
        'wait_traffic',
        'free_motor', 'manual_control', 'line_calibrate', 'line_tune_pid',
        'scan_laser_area',
        'stop_wait_traffic',
    ]

    def __init__(self, robot):
        super(TracklessAgv05Base, self).__init__(robot)
        self._costmap_profile = None
        self.__navx_control_pub = rospy.Publisher(self.navx_control, NavControl, queue_size=1)
        self.__initialpose_pub = rospy.Publisher('/initialpose_bag', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)
        self.__set_map_srv = rospy.ServiceProxy(self.set_map_service, nav_msgs.srv.SetMap, persistent=False)
        self.__reset_odom_srv = rospy.ServiceProxy(self.reset_odom_service, std_srvs.srv.Empty, persistent=False)
        self.tf_listener = tf.TransformListener()

        # monkey-patching
        self.__tracked_base = TrackedAgv05Base(robot)
        for m in self.tracked_methods:
            setattr(self, m, getattr(self.__tracked_base, m))

        self.action_client.stop_tracking_goal = self.stop_tracking_goal
        self.__tracked_base.action_client.stop_tracking_goal = self.stop_tracking_goal
        self.__tracked_base.map_tracker = self.map_tracker

    def dynamic_waypoint_forward(self, end, heading, constraints):
        params = {
            'nav': Goal.NAV_DYNAMIC_FORWARD,
            'goal_tolerance': Goal.TOLERANCE_GOAL,
            'path_end': Pose2D(theta=math.radians(heading), **end),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        # TODO: map_tracker

    def dynamic_waypoint_reverse(self, end, heading, constraints):
        params = {
            'nav': Goal.NAV_DYNAMIC_REVERSE,
            'goal_tolerance': Goal.TOLERANCE_GOAL,
            'path_end': Pose2D(theta=math.radians(heading), **end),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        # TODO: map_tracker

    def dynamic_line_forward(self, start, end, constraints):
        params = {
            'nav': Goal.NAV_DYNAMIC_LINE_FORWARD,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(x=end['x'], y=end['y'], theta=end.get('theta', -1.0)),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.forward(hint=end)

    def dynamic_line_reverse(self, start, end, constraints):
        params = {
            'nav': Goal.NAV_DYNAMIC_LINE_REVERSE,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(x=end['x'], y=end['y'], theta=end.get('theta', -1.0)),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.reverse(hint=end)

    def dynamic_bezier_forward(self, start, end, cp1, cp2, constraints):
        params = {
            'nav': Goal.NAV_DYNAMIC_BEZIER_FORWARD,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(x=end['x'], y=end['y'], theta=end.get('theta', -1.0)),
            'path_cp1': Pose2D(**cp1),
            'path_cp2': Pose2D(**cp2),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.forward(hint=end)

    def dynamic_bezier_reverse(self, start, end, cp1, cp2, constraints):
        params = {
            'nav': Goal.NAV_DYNAMIC_BEZIER_REVERSE,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(x=end['x'], y=end['y'], theta=end.get('theta', -1.0)),
            'path_cp1': Pose2D(**cp1),
            'path_cp2': Pose2D(**cp2),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.reverse(hint=end)

    def move_forward(self, start, end, constraints):
        params = {
            'nav': Goal.NAV_FORWARD,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(**end),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.forward(hint=end)

    def move_reverse(self, start, end, constraints):
        params = {
            'nav': Goal.NAV_REVERSE,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(**end),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.reverse(hint=end)

    def bezier_forward(self, start, end, cp1, cp2, constraints):
        params = {
            'nav': Goal.NAV_BEZIER_FORWARD,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(**end),
            'path_cp1': Pose2D(**cp1),
            'path_cp2': Pose2D(**cp2),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.forward(hint=end)

    def bezier_reverse(self, start, end, cp1, cp2, constraints):
        params = {
            'nav': Goal.NAV_BEZIER_REVERSE,
            'path_start': Pose2D(**start),
            'path_end': Pose2D(**end),
            'path_cp1': Pose2D(**cp1),
            'path_cp2': Pose2D(**cp2),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.reverse(hint=end)

    def forward_dock(self, marker_type, target, alignment_distance, constraints):
        params = {
            'nav': Goal.NAV_FORWARD_DOCK,
            'marker_type': marker_type,
            'path_end': Pose2D(**target),
            'path_cp2': Pose2D(x=alignment_distance),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        # TODO: map_tracker

    def reverse_dock(self, marker_type, target, alignment_distance, constraints):
        params = {
            'nav': Goal.NAV_REVERSE_DOCK,
            'marker_type': marker_type,
            'path_end': Pose2D(**target),
            'path_cp2': Pose2D(x=alignment_distance),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        # TODO: map_tracker

    def omni_dock(self, marker_type, target, alignment_distance, constraints):
        params = {
            'nav': Goal.NAV_OMNI_DOCK,
            'marker_type': marker_type,
            'path_end': Pose2D(**target),
            'path_cp2': Pose2D(x=alignment_distance),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        # TODO: map_tracker

    def forward_undock(self, marker_type, target, alignment_distance, undock_end, constraints):
        params = {
            'nav': Goal.NAV_FORWARD_UNDOCK,
            'marker_type': marker_type,
            'path_start': Pose2D(**undock_end),
            'path_end': Pose2D(**target),
            'path_cp2': Pose2D(x=alignment_distance),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        # TODO: map_tracker

    def reverse_undock(self, marker_type, target, alignment_distance, undock_end, constraints):
        params = {
            'nav': Goal.NAV_REVERSE_UNDOCK,
            'marker_type': marker_type,
            'path_start': Pose2D(**undock_end),
            'path_end': Pose2D(**target),
            'path_cp2': Pose2D(x=alignment_distance),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        # TODO: map_tracker

    def free_rotate_left(self, end, heading, constraints):
        params = {
            'nav': Goal.NAV_ROTATE_LEFT,
            'path_end': Pose2D(theta=math.radians(heading), **end),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.rotate_left_to_heading(heading)

    def free_rotate_right(self, end, heading, constraints):
        params = {
            'nav': Goal.NAV_ROTATE_RIGHT,
            'path_end': Pose2D(theta=math.radians(heading), **end),
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.rotate_right_to_heading(heading)

    def select_nav_profile(self, profile, constraints={}):
        self.__tracked_base.select_nav_profile(profile, constraints)
        self.wait_for_result()

        if profile < 1 or profile > 5:
            return

        params = {
            'nav': Goal.NAV_SELECT_NAV_PROFILE1 + profile - 1,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def select_laser_profile(self, profile, constraints={}):
        self.__tracked_base.select_laser_profile(profile, constraints)
        self.wait_for_result()

        if profile < 1 or profile > 10:
            return

        params = {
            'nav': Goal.NAV_SELECT_LASER_PROFILE1 + profile - 1,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)

    def stop(self, constraints={}, make_confused=True):
        if not self.action_client.gh:
            return self.__tracked_base.stop(constraints, make_confused)

        self.cancel_goal()
        self.__navx_control_pub.publish(
            NavControl(control=NavControl.CONTROL_ABORT))
        params = {
            'nav': Goal.NAV_IDLE,
        }
        params.update(constraints)
        self.make_and_send_goal(**params)
        self.map_tracker.stop()
        if make_confused:
            self.map_tracker.set_confused()

    def cancel_forward_flag(self):
        if not self.action_client.gh:
            return self.__tracked_base.cancel_forward_flag()
        self.__navx_control_pub.publish(
            NavControl(control=NavControl.CONTROL_CANCEL_FORWARD_FLAG))
        params = {
            'nav': Goal.NAV_IDLE,
        }
        self.make_and_send_goal(**params)

    def pause(self):
        self.__tracked_base.pause()
        self.__navx_control_pub.publish(
            NavControl(control=NavControl.CONTROL_PAUSE))

    def resume(self):
        self.__tracked_base.resume()
        self.__navx_control_pub.publish(
            NavControl(control=NavControl.CONTROL_CONTINUE))

    def safety_resume(self):
        self.__tracked_base.safety_resume()
        self.__navx_control_pub.publish(
            NavControl(control=NavControl.CONTROL_SAFETY_RESUME))

    # monkey-patch methods in SimpleActionClient
    def stop_tracking_goal(self):
        self.action_client.gh = None
        self.__tracked_base.action_client.gh = None

    # override methods in ActionClientMixin
    def register_feedback_callback(self, cb):
        super(TracklessAgv05Base, self).register_feedback_callback(cb)
        self.__tracked_base.register_feedback_callback(cb)

    def get_feedback(self):
        if self.feedback_timestamp >= self.__tracked_base.feedback_timestamp:
            return super(TracklessAgv05Base, self).get_feedback()
        else:
            return self.__tracked_base.get_feedback()

    def get_result(self):
        if self.action_client.gh:
            return super(TracklessAgv05Base, self).get_result()
        else:
            return self.__tracked_base.get_result()

    def wait_for_result(self, timeout=rospy.Duration()):
        if self.action_client.gh:
            result = super(TracklessAgv05Base, self).wait_for_result(timeout)
            if result:
                self.map_tracker.stop()
            return result
        else:
            return self.__tracked_base.wait_for_result(timeout)

    def wait_for_server(self, timeout=rospy.Duration()):
        return (super(TracklessAgv05Base, self).wait_for_server(timeout) and
            self.__tracked_base.wait_for_server(timeout))

    def is_idle(self):
        return (super(TracklessAgv05Base, self).is_idle() and
            self.__tracked_base.is_idle())

    # location tracking
    def get_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base', rospy.Time())
            rpy = tf.transformations.euler_from_quaternion(rot)
            return Pose2D(x=trans[0], y=trans[1], theta=rpy[2])
        except tf.Exception as ex:
            rospy.logerr('Error obtaining transformation of base frame: %s', ex)
        except Exception as ex:
            rospy.logerr('Error obtaining transformation of base frame: %s', ex)

    def set_initial_pose(self, pose, precise=False):
        rot = tf.transformations.quaternion_from_euler(0, 0, pose.theta)

        pcs = geometry_msgs.msg.PoseWithCovarianceStamped()
        pcs.header.frame_id = 'map'
        pcs.pose.pose.position.x = pose.x
        pcs.pose.pose.position.y = pose.y
        pcs.pose.pose.orientation.x = rot[0]
        pcs.pose.pose.orientation.y = rot[1]
        pcs.pose.pose.orientation.z = rot[2]
        pcs.pose.pose.orientation.w = rot[3]
        if precise:
            pcs.pose.covariance[0] = 0.0025
            pcs.pose.covariance[7] = 0.0025
            pcs.pose.covariance[35] = (math.pi / 72) ** 2
        else:
            pcs.pose.covariance[0] = 0.25
            pcs.pose.covariance[7] = 0.25
            pcs.pose.covariance[35] = (math.pi / 12) ** 2

        map_idx = self.map_tracker._get_map_idx()
        if self.robot.models.ocg and len(self.robot.models.ocg) > map_idx:
            ocg = self.robot.models.ocg[map_idx]
        else:
            return

        self.__initialpose_pub.publish(pcs)
        try:
            res = self.__set_map_srv(ocg, pcs)
            if not res.success:
                raise RuntimeError('Unsuccessful')
        except Exception as ex:
            rospy.logerr('Error calling set_map service: %s', ex)

        # wait for amcl to publish tf pose
        rospy.sleep(0.5)

    def reset_odom(self):
        try:
            self.__reset_odom_srv()
        except Exception as ex:
            rospy.logerr('Error calling reset_odom service: %s', ex)

    def set_initial_map_and_location(self, map, location):
        self.__tracked_base.set_initial_map_and_location(map, location)
        self.set_initial_pose(self.map_tracker.get_pose())

    def set_initial_map_and_location_tracker(self, map, location):
        self.__tracked_base.set_initial_map_and_location(map, location)

    # dimension
    def set_dimension_profile(self, profile):
        if not super(TracklessAgv05Base, self).set_dimension_profile(profile):
            return
        if 'dynamic' in self.robot.models.allowed_motions:
            rr = self.robot.models.rr[profile]
            if rr != self._costmap_profile:
                if not self._set_dimension_costmap(*rr):
                    return
                self._costmap_profile = rr
        return True

    def clear_dimension_profile(self):
        super(TracklessAgv05Base, self).clear_dimension_profile()
        self._costmap_profile = None

    def _set_dimension_costmap(self, robot_radius, inflation_radius):
        client = None
        try:
            client = dynamic_reconfigure.client.Client('/agv05_navx/costmap', timeout=2.0)
            client.update_configuration({'robot_radius': robot_radius})
            client.close()
            client = None

            client = dynamic_reconfigure.client.Client('/agv05_navx/costmap/path_layer', timeout=0.5)
            config = client.get_configuration(timeout=0.5)
            path_cost = config.cost if config else 10
            client.close()
            client = None

            client = dynamic_reconfigure.client.Client('/agv05_navx/costmap/inflation_layer', timeout=0.5)
            config = client.get_configuration(timeout=0.5)
            lethal_cost = config.footprint_lethal_cost if config else 253  # INSCRIBED_INFLATED_OBSTACLE
            cost_scaling_factor = 100.0
            if inflation_radius > robot_radius and path_cost and path_cost < lethal_cost:
                cost_scaling_factor = math.log((lethal_cost - 1) / path_cost) / (inflation_radius - robot_radius)
                if cost_scaling_factor > 100.0:
                    cost_scaling_factor = 100.0
            client.update_configuration({
                'inflation_radius': inflation_radius,
                'cost_scaling_factor': cost_scaling_factor,
            })
            return True
        except Exception as ex:
            rospy.logwarn('Error setting costmap parameters: %s', ex)
        finally:
            if client:
                client.close()


class TracklessAgv05(TracklessMixin, Agv05):
    name = 'trackless_agv05'
    base_cls = TracklessAgv05Base

    def start(self, mode):
        if self.is_alive():
            rospy.logwarn('Robot is already started.')
            return

        super(TracklessAgv05, self).start(mode)

        if mode in (1, 3):
            self.start_amcl()
            try:
                rospy.wait_for_service('/set_map', timeout=10.0)
            except Exception:
                try:
                    self.stop()
                except Exception:
                    pass
                raise RuntimeError('Timeout while waiting for localization components to start.')
            rospy.sleep(1.0)  # wait for map layout publisher to connect to amcl

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

        super(TracklessAgv05, self).stop()

    def start_amcl(self):
        self.add_capability('std_capabilities/Navigation2D', 'agv05_capabilities/nav2d_amcl')

    def stop_amcl(self):
        self.remove_capability('std_capabilities/Navigation2D')

    def start_slam(self):
        self.add_capability('std_capabilities/Navigation2D', 'agv05_capabilities/nav2d_%s' % self.config.mapping_method)

    def stop_slam(self):
        self.remove_capability('std_capabilities/Navigation2D')
