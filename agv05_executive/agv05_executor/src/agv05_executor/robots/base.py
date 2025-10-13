from __future__ import absolute_import
from __future__ import unicode_literals

from actionlib import ActionException, SimpleActionClient, SimpleGoalState
from agv05_webserver.system.models import Variable
import geometry_msgs.msg
import rospy
import std_msgs.msg
import struct
import ujson as json

from ..models.map_tracker import MapTracker


class Base(object):
    cmd_vel_topic = '/cmd_vel'
    manual_cmd_vel_topic = '/cmd_vel'
    marker_type_topic = '/marker_type'
    mileage_topic = '/agv05/motor/mileage'

    # Hack: Use the smallest positive denormal to signal no obstacle sensing
    denormal = struct.unpack('<d', struct.pack('<q', 1))[0]

    def __init__(self, robot):
        self.robot = robot
        self.cmd_vel = geometry_msgs.msg.Twist()
        self.human_follower_running = False
        self.human_follower_cb = None
        self.mileage = Variable.get_mileage()
        self.__cmd_vel = rospy.Publisher(self.manual_cmd_vel_topic, geometry_msgs.msg.Twist, queue_size=10)
        self.__human_follower = rospy.Publisher('/human_follower/enable', std_msgs.msg.Bool, queue_size=1)
        self.__marker_type = rospy.Publisher(self.marker_type_topic, std_msgs.msg.String, queue_size=1)
        self.__cmd_vel_sub = rospy.Subscriber(self.cmd_vel_topic, geometry_msgs.msg.Twist, self._handle_cmd_vel, queue_size=1)
        self.__human_follower_sub = rospy.Subscriber('/human_follower/running', std_msgs.msg.Bool, self._handle_human_follower, queue_size=1)
        self.__mileage_sub = rospy.Subscriber(self.mileage_topic, std_msgs.msg.Float64, self._handle_mileage, queue_size=1)

    def force_drive(self, vx, vy, vth, noos=False, timeout=None):
        v = geometry_msgs.msg.Twist()
        start = rospy.Time.now()

        # Drive
        v.linear.x = vx
        v.linear.y = vy
        v.angular.z = vth

        # No obstacle sensing
        if noos:
            v.linear.z = self.denormal

        # Does not stop driving
        if timeout is None:
            self.__cmd_vel.publish(v)
            return True

        while (rospy.Time.now() - start) < rospy.Duration(timeout):
            self.__cmd_vel.publish(v)
            rospy.sleep(0.1)

        # Stop driving
        v.linear.x = 0
        v.linear.y = 0
        v.angular.z = 0
        self.__cmd_vel.publish(v)
        return True

    def has_human_follower(self):
        camera = rospy.get_param('/human_follower/camera', None)
        return camera and camera != '-'

    def enable_human_follower(self, enable, cb):
        self.human_follower_cb = cb
        msg = std_msgs.msg.Bool(enable)
        self.__human_follower.publish(msg)

    def publish_marker_type(self, marker_type):
        msg = std_msgs.msg.String(marker_type)
        self.__marker_type.publish(msg)

    def _handle_cmd_vel(self, v):
        self.cmd_vel = v

    def _handle_human_follower(self, msg):
        self.human_follower_running = msg.data
        if self.human_follower_cb:
            self.human_follower_cb(self.human_follower_running)

    def _handle_mileage(self, msg):
        self.mileage = msg.data


class ActionClientMixin(object):
    action_server = '/move_base'
    action_spec = None
    feedback_cooldown_interval = 0.5

    def __init__(self, robot):
        super(ActionClientMixin, self).__init__(robot)
        self.feedback_timestamp = 0
        self.feedback = None
        self.__feedback_callback = None
        self.__done_callback = None

        if not self.action_spec:
            raise NotImplementedError()

        self.action_client = SimpleActionClient(self.action_server, self.action_spec)
        try:
            a = self.action_spec()
            self.goal_class = type(a.action_goal.goal)
        except AttributeError:
            raise ActionException('Type is not an action spec: %s' % self.action_spec)

        self.__nav_feedback_sub = rospy.Subscriber(self.action_server + '/feedback',
            type(self.action_spec().action_feedback),
            lambda feedback: self._handle_feedback(feedback.feedback), queue_size=10)

    def register_feedback_callback(self, cb):
        self.__feedback_callback = cb

    def register_done_callback(self, cb):
        self.__done_callback = cb

    def make_goal(self, **kwargs):
        return self.goal_class(**kwargs)

    def send_goal(self, g):
        self.action_client.send_goal(g, done_cb=self._handle_done)
        self.feedback = None

    def make_and_send_goal(self, **kwargs):
        self.send_goal(self.make_goal(**kwargs))

    def cancel_goal(self):
        self.action_client.cancel_goal()

    def _handle_feedback(self, feedback):
        self.feedback = feedback
        self.feedback_timestamp = rospy.get_time()
        if self.__feedback_callback:
            self.__feedback_callback(feedback)

    def _handle_done(self, feedback, result):
        if self.__done_callback:
            self.__done_callback(feedback, result)

    def get_feedback(self):
        if rospy.get_time() - self.feedback_timestamp < self.feedback_cooldown_interval:
            return self.feedback

    def get_result(self):
        return self.action_client.get_result()

    def wait_for_result(self, timeout=rospy.Duration()):
        return self.action_client.wait_for_result(timeout)

    def wait_for_server(self, timeout=rospy.Duration()):
        return self.action_client.wait_for_server(timeout)

    def is_idle(self):
        return self.action_client.simple_state == SimpleGoalState.DONE


class TrackedBaseMixin(object):
    map_tracker_class = MapTracker

    def __init__(self, robot):
        super(TrackedBaseMixin, self).__init__(robot)
        self.map_tracker = type(self).map_tracker_class()

    def forward(self, constraints, straight=True):
        raise NotImplementedError()

    def reverse(self, constraints, straight=True):
        raise NotImplementedError()

    def branch_forward_left(self, constraints, straight=True, track_dir=True):
        raise NotImplementedError()

    def branch_forward_right(self, constraints, straight=True, track_dir=True):
        raise NotImplementedError()

    def branch_reverse_left(self, constraints, straight=True, track_dir=True):
        raise NotImplementedError()

    def branch_reverse_right(self, constraints, straight=True, track_dir=True):
        raise NotImplementedError()

    def rotate_left(self, constraints):
        raise NotImplementedError()

    def rotate_right(self, constraints):
        raise NotImplementedError()

    def uturn_left(self, constraints):
        raise NotImplementedError()

    def uturn_right(self, constraints):
        raise NotImplementedError()

    def rotate3q_left(self, constraints):
        raise NotImplementedError()

    def rotate3q_right(self, constraints):
        raise NotImplementedError()

    def search_line_left(self, constraints):
        raise NotImplementedError()

    def search_line_right(self, constraints):
        raise NotImplementedError()

    def wait_traffic(self, constraints):
        raise NotImplementedError()

    def free_motor(self, constraints):
        raise NotImplementedError()

    def stop(self, constraints={}):
        raise NotImplementedError()

    def stop_wait_traffic(self, constraints={}):
        raise NotImplementedError()

    def cancel_forward_flag(self):
        raise NotImplementedError()

    def pause(self):
        raise NotImplementedError()

    def resume(self):
        raise NotImplementedError()

    def safety_resume(self):
        raise NotImplementedError()

    # Explanation of the parameters:
    # 1. location: a tuple consisting of (j, direction), where
    #              j is the junction id, and
    #              direction is the direction enum.
    # Note: The location tuple is also returned by get_location().

    def set_initial_map_and_location(self, map, location):
        self.map_tracker.set_map(map)
        self.map_tracker.fetch_models(self.robot.models)
        self.map_tracker.set_initial_location(location)
        self.map_tracker.patch_models(self.robot.models)
        self.set_dimension_profile(self.get_dimension_profile())  # publish robot_svg

    def get_location(self):
        return self.map_tracker.get_location()

    def get_prev_location(self):
        return self.map_tracker.get_prev_location()

    def get_location_hint(self):
        return self.map_tracker.get_location_hint()

    def is_location_aware(self):
        return self.map_tracker.is_aware()

    def get_motion(self):
        return self.map_tracker.get_motion()

    # Dimension
    def set_dimension_profile(self, profile):
        return self.map_tracker.set_dimension_profile(profile)

    def clear_dimension_profile(self):
        self.map_tracker.clear_dimension_profile()

    def get_dimension_profile(self):
        return self.map_tracker.get_dimension_profile()


class TracklessBaseMixin(object):

    # Explanation of the parameters:
    # 1. start: start coordinate
    # 2. end: end coordinate
    # 3. cp1: control point 1
    # 4. cp2: control point 2
    # 5. heading: yaw angle (in degrees)
    # Note: All coordinates are represented as a dictionary with keys 'x' and 'y'.

    def dynamic_waypoint_forward(self, end, heading, constraints):
        raise NotImplementedError()

    def dynamic_waypoint_reverse(self, end, heading, constraints):
        raise NotImplementedError()

    def dynamic_line_forward(self, start, end, constraints):
        raise NotImplementedError()

    def dynamic_line_reverse(self, start, end, constraints):
        raise NotImplementedError()

    def dynamic_bezier_forward(self, start, end, cp1, cp2, constraints):
        raise NotImplementedError()

    def dynamic_bezier_reverse(self, start, end, cp1, cp2, constraints):
        raise NotImplementedError()

    def move_forward(self, start, end, constraints):
        raise NotImplementedError()

    def move_reverse(self, start, end, constraints):
        raise NotImplementedError()

    def bezier_forward(self, start, end, cp1, cp2, constraints):
        raise NotImplementedError()

    def bezier_reverse(self, start, end, cp1, cp2, constraints):
        raise NotImplementedError()

    def free_rotate_left(self, end, heading, constraints):
        raise NotImplementedError()

    def free_rotate_right(self, end, heading, constraints):
        raise NotImplementedError()

    # Explanation of the parameters:
    # 1. pose: geometry_msgs.msg.Pose2D
    # Note: The pose struct is also returned by get_pose().

    def set_initial_pose(self, pose, precise=False):
        raise NotImplementedError()

    def get_pose(self):
        raise NotImplementedError()

    def reset_odom(self):
        raise NotImplementedError()


class TrackedBase(TrackedBaseMixin, ActionClientMixin, Base):
    pass


class TracklessBase(TracklessBaseMixin, ActionClientMixin, Base):
    pass
