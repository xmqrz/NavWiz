from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_msgs.msg import MotorFeedback, SteeringFeedback, MecanumFeedback, NavActionResult
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import cmath
import dynamic_reconfigure.client
import rospy
import tf
import threading

from .module import Module


class CalibrationMotorMixin(object):

    def __init__(self, distance):
        # ROS subscribers
        self._subs = [rospy.Subscriber('odom/' + topic, Odometry, self.handle_odom_scan, topic, queue_size=1) for
            topic in rospy.get_param('nav2d_scan_topics', 'scan').split()]
        self.odom = dict()
        self.odom_start = dict()
        self.odom_distance, self.odom_angle = cmath.polar(distance)
        self.odom_theta = 0

    def __del__(self):
        rospy.loginfo('CalibrationMotorMixin Object removed')

    def start(self):
        for key, value in self.odom.items():
            pose_start = Pose2D()
            pose_start.x = value.pose.pose.position.x
            pose_start.y = value.pose.pose.position.y
            pose_start.theta = self._get_yaw(value.pose.pose.orientation)
            self.odom_start.update({key: pose_start})

    def stop(self):
        # ROS subscribers
        for sub in self._subs:
            sub.unregister()

    def loop(self):
        count = 0
        angle = complex(0)
        distances = complex(0)
        for key, value in self.odom.items():
            if key in self.odom_start.keys():
                pose_start = complex(self.odom_start[key].x, self.odom_start[key].y)
                pose_end = complex(value.pose.pose.position.x, value.pose.pose.position.y)
                distance, theta = cmath.polar(pose_end - pose_start)
                theta -= self.odom_start[key].theta
                distances += distance * complex(cmath.cos(theta).real, cmath.sin(theta).real)
                theta = self._get_yaw(value.pose.pose.orientation) - self.odom_start[key].theta
                angle += complex(cmath.cos(theta).real, cmath.sin(theta).real)
                count += 1
        if count > 0:
            self.odom_distance, self.odom_theta = cmath.polar(distances / count)
            self.odom_angle = cmath.phase(angle)

    def _get_yaw(self, orientation):
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        return rpy[2]

    def handle_odom_scan(self, msg, topic):
        self.odom[topic] = msg


class CalibrationDifferentialMixin(CalibrationMotorMixin):

    def __init__(self, distance):
        super(CalibrationDifferentialMixin, self).__init__(distance)

        # dynamic cfg
        client = dynamic_reconfigure.client.Client('/agv05_motor', timeout=0.5)
        config = client.get_configuration(timeout=0.5)
        self.track_width = config.track_width if config else 0.5
        client.close()

        # ROS subscribers
        self._subs += [
            rospy.Subscriber('agv05/motor/feedback', MotorFeedback, self.handle_feedback, queue_size=1),
        ]
        self.feedback_left_distance = 0
        self.feedback_right_distance = 0
        self.left_start = 0
        self.right_start = 0
        self.left_distance = 0
        self.right_distance = 0
        self.left_distance_actual = self.odom_distance
        self.right_distance_actual = self.odom_distance
        self.yaw_angle_actual = abs(self.odom_angle * 180 / cmath.pi)
        if self.odom_angle < 0.0:
            self.yaw_angle_actual = 360 - self.yaw_angle_actual

    def start(self):
        super(CalibrationDifferentialMixin, self).start()

        self.left_start = self.feedback_left_distance
        self.right_start = self.feedback_right_distance

    def loop(self):
        super(CalibrationDifferentialMixin, self).loop()

        self.left_distance = self.feedback_left_distance - self.left_start
        self.right_distance = self.feedback_right_distance - self.right_start

    def handle_feedback(self, msg):
        self.feedback_left_distance = msg.left_distance
        self.feedback_right_distance = msg.right_distance


class CalibrationDifferentialWheel(CalibrationDifferentialMixin):

    def loop(self):
        super(CalibrationDifferentialWheel, self).loop()

        dx = 0.5 * self.track_width * cmath.tan(self.odom_angle)
        left_distance_actual = self.odom_distance + (dx if self.left_distance < 0.0 else -dx)
        right_distance_actual = self.odom_distance + (dx if self.right_distance > 0.0 else -dx)
        self.left_distance_actual = left_distance_actual.real
        self.right_distance_actual = right_distance_actual.real

    def data(self):
        return {
            'left_distance': self.left_distance_actual,
            'right_distance': self.right_distance_actual,
        }

    def save(self):
        # dynamic cfg
        client = dynamic_reconfigure.client.Client('/agv05_motor', timeout=0.5)
        left_motor = self.left_distance_actual / abs(self.left_distance)
        right_motor = self.right_distance_actual / abs(self.right_distance)
        client.update_configuration({'left_motor_calibration': left_motor, 'right_motor_calibration': right_motor})
        client.close()


class CalibrationDifferentialYaw(CalibrationDifferentialMixin):

    YAW_ANGLE_MIN_DEG = 10.0

    def loop(self):
        super(CalibrationDifferentialYaw, self).loop()

        yaw_angle = (self.right_distance - self.left_distance) / self.track_width
        yaw_angle_actual = abs(self.odom_angle * 180 / cmath.pi)
        if yaw_angle * self.odom_angle < 0.0:
            yaw_angle_actual = (360 - yaw_angle_actual) if yaw_angle_actual > self.YAW_ANGLE_MIN_DEG else 0.0
        self.yaw_angle_actual = yaw_angle_actual

    def data(self):
        return {
            'yaw_angle': self.yaw_angle_actual,
        }

    def save(self):
        assert self.yaw_angle_actual > self.YAW_ANGLE_MIN_DEG, 'Yaw angle < %s deg.' % self.YAW_ANGLE_MIN_DEG

        # dynamic cfg
        client = dynamic_reconfigure.client.Client('/agv05_motor', timeout=0.5)
        config = client.get_configuration(timeout=0.5)
        left_distance = self.left_distance * (config.left_motor_calibration if config else 1.0)
        right_distance = self.right_distance * (config.right_motor_calibration if config else 1.0)
        self.track_width = abs((right_distance - left_distance) * 180 / (self.yaw_angle_actual * cmath.pi))
        client.update_configuration({'track_width': self.track_width})
        client.close()


class CalibrationTricycleMixin(CalibrationMotorMixin):

    def __init__(self, distance):
        super(CalibrationTricycleMixin, self).__init__(distance)

        # ROS subscribers
        self._subs += [
            rospy.Subscriber('agv05/motor/steering/feedback', SteeringFeedback, self.handle_feedback, queue_size=1),
            rospy.Subscriber('agv05/motor/straight_distance', Float64, self.handle_straight_distance, queue_size=1),
            rospy.Subscriber('agv05/motor/rotational_distance', Float64, self.handle_rotational_distance, queue_size=1),
        ]
        self.feedback_stamp = rospy.get_time()
        self.feedback_distance = 0
        self.feedback_start = 0
        self.rotational_distance = 0
        self.rotational_start = 0
        self.straight_distance = 0
        self.distance_start = 0
        self.wheel_distance = 0
        self.distance_actual = self.odom_distance
        self.yaw_angle_actual = abs(self.odom_angle * 180 / cmath.pi)
        if self.odom_angle < 0.0:
            self.yaw_angle_actual = 360 - self.yaw_angle_actual

    def start(self):
        super(CalibrationTricycleMixin, self).start()

        self.feedback_start = self.feedback_distance
        self.distance_start = self.straight_distance
        self.rotational_start = self.rotational_distance

    def loop(self):
        super(CalibrationTricycleMixin, self).loop()

        self.wheel_distance = self.straight_distance - self.distance_start

    def handle_feedback(self, msg):
        stamp = rospy.get_time()
        drive_speed = -msg.drive_speed if msg.steering_angle < 0.0 else msg.drive_speed
        self.feedback_distance += drive_speed * (stamp - self.feedback_stamp)
        self.feedback_stamp = stamp

    def handle_straight_distance(self, msg):
        self.straight_distance = msg.data

    def handle_rotational_distance(self, msg):
        self.rotational_distance = msg.data


class CalibrationTricycleWheel(CalibrationTricycleMixin):

    def loop(self):
        super(CalibrationTricycleWheel, self).loop()

        self.distance_actual = self.odom_distance

    def data(self):
        return {
            'wheel_distance': self.distance_actual,
        }

    def save(self):
        # dynamic cfg
        client = dynamic_reconfigure.client.Client('/agv05_motor', timeout=0.5)
        config = client.get_configuration(timeout=0.5)
        drive_motor = config.drive_motor_calibration if config else 1.0
        drive_motor *= self.distance_actual / abs(self.wheel_distance)
        client.update_configuration({'drive_motor_calibration': drive_motor})
        client.close()


class CalibrationTricycleYaw(CalibrationTricycleMixin):

    YAW_ANGLE_MIN_DEG = 10.0

    def loop(self):
        super(CalibrationTricycleYaw, self).loop()

        yaw_angle = self.rotational_distance - self.rotational_start
        yaw_angle_actual = abs(self.odom_angle * 180 / cmath.pi)
        if yaw_angle * self.odom_angle < 0.0:
            yaw_angle_actual = (360 - yaw_angle_actual) if yaw_angle_actual > self.YAW_ANGLE_MIN_DEG else 0.0
        self.yaw_angle_actual = yaw_angle_actual

        self.distance_actual = -self.odom_distance if self.odom_theta < 0.0 else self.odom_distance

    def data(self):
        return {
            'yaw_angle': self.yaw_angle_actual,
            'wheel_distance': self.distance_actual,
        }

    def save(self):
        assert self.yaw_angle_actual > self.YAW_ANGLE_MIN_DEG, 'Yaw angle < %s deg.' % self.YAW_ANGLE_MIN_DEG

        yaw_angle_actual = abs(self.yaw_angle_actual * cmath.pi / 180)
        wheelbase = abs(self.feedback_distance - self.feedback_start) / yaw_angle_actual
        drive_center_offset = 0.5 * self.distance_actual / cmath.sin(0.5 * yaw_angle_actual).real

        # dynamic cfg
        client = dynamic_reconfigure.client.Client('/agv05_motor', timeout=0.5)
        config = client.get_configuration(timeout=0.5)
        if config:
            drive_center_offset += config.drive_center_offset
            offset = config.drive_center_offset
            if config.steer_angle_turn and abs(config.steer_angle_turn) < 90.0:
                offset -= config.wheelbase / cmath.tan(config.steer_angle_turn * cmath.pi / 180).real
            wheelbase *= config.drive_motor_calibration
            wheelbase = cmath.sqrt(wheelbase * wheelbase - offset * offset).real
        client.update_configuration({'wheelbase': wheelbase, 'drive_center_offset': drive_center_offset})
        client.close()


class CalibrationMecanumMixin(CalibrationMotorMixin):

    def __init__(self, distance):
        super(CalibrationMecanumMixin, self).__init__(distance)

        # dynamic cfg
        client = dynamic_reconfigure.client.Client('/agv05_motor', timeout=0.5)
        config = client.get_configuration(timeout=0.5)
        self.wheelbase = config.mecanum_wheelbase if config else 0.5
        self.track_width = config.mecanum_track_width if config else 0.5
        client.close()

        # ROS subscribers
        self._subs += [
            rospy.Subscriber('agv05/motor/mecanum/feedback', MecanumFeedback, self.handle_feedback, queue_size=1),
        ]
        self.feedback_left_front_distance = 0
        self.feedback_left_rear_distance = 0
        self.feedback_right_rear_distance = 0
        self.feedback_right_front_distance = 0
        self.left_front_start = 0
        self.left_rear_start = 0
        self.right_rear_start = 0
        self.right_front_start = 0
        self.left_front_distance = 0
        self.left_rear_distance = 0
        self.right_rear_distance = 0
        self.right_front_distance = 0
        self.left_front_distance_actual = self.odom_distance
        self.left_rear_distance_actual = self.odom_distance
        self.right_rear_distance_actual = self.odom_distance
        self.right_front_distance_actual = self.odom_distance
        self.yaw_angle_actual = abs(self.odom_angle * 180 / cmath.pi)
        if self.odom_angle < 0.0:
            self.yaw_angle_actual = 360 - self.yaw_angle_actual

    def start(self):
        super(CalibrationMecanumMixin, self).start()

        self.left_front_start = self.feedback_left_front_distance
        self.left_rear_start = self.feedback_left_rear_distance
        self.right_rear_start = self.feedback_right_rear_distance
        self.right_front_start = self.feedback_right_front_distance

    def loop(self):
        super(CalibrationMecanumMixin, self).loop()

        self.left_front_distance = self.feedback_left_front_distance - self.left_front_start
        self.left_rear_distance = self.feedback_left_rear_distance - self.left_rear_start
        self.right_rear_distance = self.feedback_right_rear_distance - self.right_rear_start
        self.right_front_distance = self.feedback_right_front_distance - self.right_front_start

    def handle_feedback(self, msg):
        self.feedback_left_front_distance = msg.left_front.distance
        self.feedback_left_rear_distance = msg.left_rear.distance
        self.feedback_right_rear_distance = msg.right_rear.distance
        self.feedback_right_front_distance = msg.right_front.distance


class CalibrationMecanumWheel(CalibrationMecanumMixin):

    def loop(self):
        super(CalibrationMecanumWheel, self).loop()

        left_distance = self.left_front_distance + self.left_rear_distance
        right_distance = self.right_front_distance + self.right_rear_distance
        front_distance = self.right_front_distance - self.left_front_distance
        rear_distance = self.left_rear_distance - self.right_rear_distance

        if abs(left_distance + right_distance) > abs(front_distance + rear_distance):
            dx = 0.5 * self.track_width * cmath.tan(self.odom_angle)
            left_distance_actual = self.odom_distance + (dx if left_distance < 0.0 else -dx)
            right_distance_actual = self.odom_distance + (dx if right_distance > 0.0 else -dx)
            self.left_front_distance_actual = left_distance_actual.real
            self.left_rear_distance_actual = left_distance_actual.real
            self.right_rear_distance_actual = right_distance_actual.real
            self.right_front_distance_actual = right_distance_actual.real
        else:
            dy = 0.5 * self.wheelbase * cmath.tan(self.odom_angle)
            front_distance_actual = self.odom_distance + (dy if front_distance > 0.0 else -dy)
            rear_distance_actual = self.odom_distance + (dy if rear_distance < 0.0 else -dy)
            self.left_front_distance_actual = front_distance_actual.real
            self.left_rear_distance_actual = rear_distance_actual.real
            self.right_rear_distance_actual = rear_distance_actual.real
            self.right_front_distance_actual = front_distance_actual.real

    def data(self):
        return {
            'left_front_distance': self.left_front_distance_actual,
            'left_rear_distance': self.left_rear_distance_actual,
            'right_rear_distance': self.right_rear_distance_actual,
            'right_front_distance': self.right_front_distance_actual,
        }

    def save(self):
        # dynamic cfg
        client = dynamic_reconfigure.client.Client('/agv05_motor', timeout=0.5)
        left_front_motor = self.left_front_distance_actual / abs(self.left_front_distance)
        left_rear_motor = self.left_rear_distance_actual / abs(self.left_rear_distance)
        right_rear_motor = self.right_rear_distance_actual / abs(self.right_rear_distance)
        right_front_motor = self.right_front_distance_actual / abs(self.right_front_distance)
        client.update_configuration({
            'left_front_motor_calibration': left_front_motor,
            'left_rear_motor_calibration': left_rear_motor,
            'right_rear_motor_calibration': right_rear_motor,
            'right_front_motor_calibration': right_front_motor,
        })
        client.close()


class CalibrationMecanumYaw(CalibrationMecanumMixin):

    YAW_ANGLE_MIN_DEG = 10.0

    def loop(self):
        super(CalibrationMecanumYaw, self).loop()

        left_distance = self.left_front_distance + self.left_rear_distance
        right_distance = self.right_front_distance + self.right_rear_distance
        yaw_angle = 0.5 * (right_distance - left_distance) / (self.wheelbase + self.track_width)
        yaw_angle_actual = abs(self.odom_angle * 180 / cmath.pi)
        if yaw_angle * self.odom_angle < 0.0:
            yaw_angle_actual = (360 - yaw_angle_actual) if yaw_angle_actual > self.YAW_ANGLE_MIN_DEG else 0.0
        self.yaw_angle_actual = yaw_angle_actual

    def data(self):
        return {
            'yaw_angle': self.yaw_angle_actual,
        }

    def save(self):
        assert self.yaw_angle_actual > self.YAW_ANGLE_MIN_DEG, 'Yaw angle < %s deg.' % self.YAW_ANGLE_MIN_DEG

        # dynamic cfg
        client = dynamic_reconfigure.client.Client('/agv05_motor', timeout=0.5)
        config = client.get_configuration(timeout=0.5)
        left_front_distance = self.left_front_distance * (config.left_front_motor_calibration if config else 1.0)
        left_rear_distance = self.left_rear_distance * (config.left_rear_motor_calibration if config else 1.0)
        right_rear_distance = self.right_rear_distance * (config.right_rear_motor_calibration if config else 1.0)
        right_front_distance = self.right_front_distance * (config.right_front_motor_calibration if config else 1.0)
        left_distance = left_front_distance + left_rear_distance
        right_distance = right_front_distance + right_rear_distance
        self.track_width = abs((right_distance - left_distance) * 90 / (self.yaw_angle_actual * cmath.pi)) - self.wheelbase
        client.update_configuration({'mecanum_track_width': self.track_width})
        client.close()


class CalibrationMixin(object):

    def __init__(self, robot, send_error, send_outcome):
        self.calibration = None
        self.robot = robot
        self.error_flag = False
        self._send_error = send_error
        self._send_outcome = send_outcome

    def __del__(self):
        rospy.loginfo('%s removed' % self.status())

    def status(self):
        return 'calibrate'

    def start(self):
        try:
            if self.calibration:
                self.calibration.start()
        except Exception as ex:
            rospy.logerr('[Calibration] Exception: %s', ex)
            self._send_error('Exception: %s' % ex)

    def wait(self):
        return True

    def stop(self, success):
        try:
            self.robot.base.stop()
            if self.calibration:
                self.calibration.stop()
        except Exception as ex:
            rospy.logerr('[Calibration] Exception: %s', ex)
            self._send_error('Exception: %s' % ex)
        else:
            if not success:
                self.calibration = None
            if not self.calibration:
                self._outcome(success)

    def save(self):
        try:
            if self.calibration:
                self.calibration.save()
                self.calibration = None
        except Exception as ex:
            rospy.logerr('[Calibration] Exception: %s', ex)
            self._send_error('Exception: %s' % ex)
        else:
            self._outcome()

    def _outcome(self, success=True):
        if not self.error_flag:
            result = ' Done' if success else ' Failed'
            outcome = self.status().replace('_', ' ').title() + result
            rospy.loginfo('[Calibration] Outcome: %s', outcome)
            self._send_outcome(outcome)

    def data(self):
        if self.calibration:
            self.calibration.loop()
            return self.calibration.data()
        return None

    def handle_in_pipe(self, data):
        pass


class CalibrationLine(CalibrationMixin):

    def status(self):
        return 'calibrate_line'

    def start(self):
        try:
            assert {'rotate_left', 'rotate_right'}.issubset(self.robot.models.allowed_motions), \
                '"Rotate Left" or "Rotate Right" motion is not allowed.'
            self.robot.base.line_calibrate()
        except Exception as ex:
            rospy.logerr('[Calibration] Exception: %s', ex)
            self._send_error('Exception: %s' % ex)


class CalibrationLinePID(CalibrationMixin):

    def __init__(self, robot, send_error, send_outcome, profile, ahead, distance):
        super(CalibrationLinePID, self).__init__(robot, send_error, send_outcome)

        self.distance = distance
        try:
            base_method = 'reverse' if distance < 0 else 'forward'
            assert {base_method}.issubset(self.robot.models.allowed_motions), \
                '"%s" motion is not allowed.' % base_method.title()

            assert 1 <= profile <= 5, \
                'Profile %d is not supported. Available are 1 to 5.' % profile
            self.robot.base.select_nav_profile(profile)

            # dynamic cfg
            client = dynamic_reconfigure.client.Client('/agv05_nav', timeout=0.5)
            client.update_configuration({'line_follow_ahead_distance%s' % profile: ahead})
            client.close()

        except Exception as ex:
            self.error_flag = True
            rospy.logerr('[Calibration] Exception: %s', ex)
            self._send_error('Exception: %s' % ex)

    def status(self):
        return 'calibrate_line_pid'

    def start(self):
        try:
            self.robot.base.wait_for_result()
            self.robot.base.line_tune_pid(self._get_constraints())

        except Exception as ex:
            rospy.logerr('[Calibration] Exception: %s', ex)
            self._send_error('Exception: %s' % ex)

    def _get_constraints(self):
        return {
            'distance': self.distance,
        }


class CalibrationMotorWheel(CalibrationMixin):

    def __init__(self, robot, send_error, send_outcome, distance, reverse=False):
        super(CalibrationMotorWheel, self).__init__(robot, send_error, send_outcome)

        self.distance = distance
        try:
            self.base_method = 'reverse' if reverse else 'forward'
            assert {self.base_method}.issubset(self.robot.models.allowed_motions), \
                '"%s" motion is not allowed.' % self.base_method.title()

            # dynamic cfg
            client = dynamic_reconfigure.client.Client('/agv05_nav', timeout=0.5)
            config = client.get_configuration(timeout=0.5)
            self.min_speed = config.straight_junction_min_speed if config else 0.15
            client.close()
            client = dynamic_reconfigure.client.Client('/agv05_motor', timeout=0.5)
            config = client.get_configuration(timeout=0.5)
            self.drive_type = config.drive_type if config else 'differential'
            client.close()
            self.calibration = CalibrationDifferentialWheel(distance) if self.drive_type == 'differential' else \
                CalibrationTricycleWheel(distance) if self.drive_type == 'tricycle_steering' else \
                CalibrationMecanumWheel(distance) if self.drive_type == 'mecanum' else None
            assert self.calibration is not None, 'Drive type "%s" is not supported.' % self.drive_type

        except Exception as ex:
            self.error_flag = True
            rospy.logerr('[Calibration] Exception: %s', ex)
            self._send_error('Exception: %s' % ex)

    def status(self):
        return 'calibrate_motor_wheel_' + self.drive_type

    def start(self):
        try:
            self.calibration.start()
            getattr(self.robot.base, self.base_method)(self._get_constraints())

        except Exception as ex:
            rospy.logerr('[Calibration] Exception: %s', ex)
            self._send_error('Exception: %s' % ex)

    def _get_constraints(self):
        return {
            'line_follow_type': 1,
            'speed': self.min_speed,
            'enable_sensor': False,
            'next_motion': 0,
            'next_speed': 0.0,
            'distance': self.distance,
        }

    def handle_in_pipe(self, data):
        cmd = data['command']
        if cmd == 'save':
            if self.calibration:
                if 'wheel_distance' in data['data']:
                    self.calibration.distance_actual = abs(float(data['data']['wheel_distance']))
                if 'left_distance' in data['data']:
                    self.calibration.left_distance_actual = abs(float(data['data']['left_distance']))
                if 'right_distance' in data['data']:
                    self.calibration.right_distance_actual = abs(float(data['data']['right_distance']))
                if 'left_front_distance' in data['data']:
                    self.calibration.left_front_distance_actual = abs(float(data['data']['left_front_distance']))
                if 'left_rear_distance' in data['data']:
                    self.calibration.left_rear_distance_actual = abs(float(data['data']['left_rear_distance']))
                if 'right_rear_distance' in data['data']:
                    self.calibration.right_rear_distance_actual = abs(float(data['data']['right_rear_distance']))
                if 'right_front_distance' in data['data']:
                    self.calibration.right_front_distance_actual = abs(float(data['data']['right_front_distance']))


class CalibrationMotorYaw(CalibrationMixin):

    def __init__(self, robot, send_error, send_outcome):
        super(CalibrationMotorYaw, self).__init__(robot, send_error, send_outcome)

        try:
            assert {'rotate_left', 'rotate_right'}.issubset(self.robot.models.allowed_motions), \
                '"Rotate Left" or "Rotate Right" motion is not allowed.'

            # dynamic cfg
            client = dynamic_reconfigure.client.Client('/agv05_motor', timeout=0.5)
            config = client.get_configuration(timeout=0.5)
            self.drive_type = config.drive_type if config else 'differential'
            client.close()
            self.calibration = CalibrationDifferentialYaw(-1) if self.drive_type == 'differential' else \
                CalibrationTricycleYaw(-1) if self.drive_type == 'tricycle_steering' else \
                CalibrationMecanumYaw(-1) if self.drive_type == 'mecanum' else None
            assert self.calibration is not None, 'Drive type "%s" is not supported.' % self.drive_type

        except Exception as ex:
            self.error_flag = True
            rospy.logerr('[Calibration] Exception: %s', ex)
            self._send_error('Exception: %s' % ex)

    def status(self):
        return 'calibrate_motor_yaw_' + self.drive_type

    def start(self):
        try:
            self.calibration.start()

            self.robot.base.stop()
            self.robot.base.manual_control()

        except Exception as ex:
            rospy.logerr('[Calibration] Exception: %s', ex)
            self._send_error('Exception: %s' % ex)

    def wait(self):
        return False

    def handle_in_pipe(self, data):
        cmd = data['command']
        if cmd == 'calibrate_motor' and data['type'] == 'yaw':
            self.robot.base.force_drive(0.0, 0.0, float(data['speed']), noos=True)
        elif cmd == 'save':
            if self.calibration:
                if 'yaw_angle' in data['data']:
                    self.calibration.yaw_angle_actual = abs(float(data['data']['yaw_angle']))
                if 'wheel_distance' in data['data']:
                    self.calibration.distance_actual = float(data['data']['wheel_distance'])


class Calibration(Module):
    id = 'calibration'

    def __init__(self, *args, **kwargs):
        super(Calibration, self).__init__(*args, **kwargs)
        self._lock = threading.RLock()
        self._execute_thread = None
        self._calibrating = False
        self._handler = None
        self._timer = None

    def stop(self):
        with self._lock:
            # self.robot.base.register_done_callback(None)
            self.handle_in_pipe({'command': 'cancel'})
        if self._execute_thread:
            t = self._execute_thread
            t.join(1)
            while t.is_alive():
                self.robot.base.stop()
                t.join(1)
            with self._lock:
                self._execute_thread = None

    def handle_in_pipe(self, data):
        try:
            with self._lock:
                cmd = data['command']
                if cmd == 'status':
                    self._send_updates()
                elif cmd == 'calibrate_battery':
                    if self._calibrating:
                        return
                    self._handler = None
                    self.robot.power.recalibrate_battery()
                    self._send_outcome('Done')
                elif cmd == 'calibrate_line':
                    if self._calibrating or self._execute_thread:
                        return
                    self._handler = CalibrationLine(self.robot, self._send_error, self._send_outcome)
                    if self._calibrate_start():
                        self._send_updates()
                elif cmd == 'calibrate_motor':
                    if self._execute_thread:
                        return
                    if self._calibrating:
                        if self._handler:
                            self._handler.handle_in_pipe(data)
                        return
                    if data['type'] == 'wheel':
                        distance = float(data['distance'])
                        self._handler = CalibrationMotorWheel(self.robot, self._send_error, self._send_outcome,
                                                              abs(distance), distance < 0)
                    elif data['type'] == 'yaw':
                        self._handler = CalibrationMotorYaw(self.robot, self._send_error, self._send_outcome)
                    else:
                        self._handler = None
                    if self._calibrate_start():
                        self._send_updates()
                elif cmd == 'calibrate_line_pid':
                    if self._calibrating or self._execute_thread:
                        return
                    profile = int(data['profile'])
                    ahead = float(data['ahead'])
                    distance = float(data['distance'])
                    self._handler = CalibrationLinePID(self.robot, self._send_error, self._send_outcome,
                                                       profile, ahead, distance)
                    if self._calibrate_start():
                        self._send_updates()
                elif cmd == 'cancel':
                    if self._handler:
                        self._handler.error_flag = True
                    self._calibrate_stop()
                    self._handler = None
                    self._send_updates()
                elif cmd == 'done':
                    success = data['success'] if 'success' in data else True
                    if self._calibrate_stop(success):
                        # error during self._handler.stop()
                        self._handler = None
                    self._send_updates()
                elif cmd == 'save':
                    if self._calibrating:
                        return
                    if self._handler:
                        self._handler.handle_in_pipe(data)
                        self._handler.save()
                        if self._handler.error_flag:
                            # error during self._handler.save()
                            pass
                        self._handler = None
                    self._send_updates()
                elif self._handler:
                    self._handler.handle_in_pipe(data)

        except KeyError as ex:
            rospy.logerr('[Calibration] invalid input: %s', ex)
        except ValueError as ex:
            rospy.logerr('[Calibration] invalid input: %s', ex)
        except Exception as ex:
            rospy.logerr('Exception: %s', ex)

    def handle_timer(self, timer_event):
        with self._lock:
            if self._handler:
                data = self._handler.data()
                if data:
                    self._send_data(data)
            elif self._timer:
                self._timer.shutdown()
                self._timer = None

    def _calibrate_start(self):
        if self._handler:
            if self._handler.error_flag:
                # error during self._handler.__init__()
                self.handle_in_pipe({'command': 'cancel'})
                return False
            self._calibrating = True
            rospy.loginfo('[Calibration] %s' % self._handler.status())
            rospy.sleep(1.0)
            self._handler.start()
            if self._handler.error_flag:
                # error during self._handler.start()
                self.handle_in_pipe({'command': 'cancel'})
                return False
            if self._handler and self._handler.wait():
                # self.robot.base.register_done_callback(self._done_cb)
                self._execute_thread = threading.Thread(target=self._done_cb)
                self._execute_thread.start()
            self._timer = rospy.Timer(rospy.Duration(0.2), self.handle_timer)
        return self._calibrating

    def _calibrate_stop(self, success=True):
        self._calibrating = False
        ret = False
        if self._handler:
            self._handler.stop(success)
            ret = self._handler.error_flag
            if self._handler.calibration is None:
                self._handler = None
            else:
                rospy.loginfo('[Calibration] %s_done' % self._handler.status())
        if self._timer:
            self._timer.shutdown()
            self._timer = None
        return ret

    def _send_updates(self):
        status = 'idle'
        data = None

        if self._handler:
            status = self._handler.status()
            if self._calibrating:
                data = self._handler.data()
            else:
                status += '_done'

        self.out({
            'command': 'status',
            'status': status,
        })
        if data:
            self._send_data(data)

    def _send_outcome(self, outcome):
        self.out({
            'command': 'outcome',
            'outcome': outcome,
        })

    def _send_error(self, error):
        self.out({
            'command': 'error',
            'error': error,
        })
        if self._handler:
            self._handler.error_flag = True

    def _send_data(self, data):
        self.out({
            'command': 'data',
            'data': data,
        })

    # def _done_cb(self, feedback, result):
        # self.robot.base.register_done_callback(None)
        # self.handle_in_pipe({'command': 'done'})

    def _done_cb(self):
        self.robot.base.wait_for_result()
        success = (self.robot.base.get_result().result == NavActionResult.RESULT_SUCCESS)
        if self._calibrating:
            rospy.sleep(1.0)
            self.handle_in_pipe({
                'command': 'done',
                'success': success,
            })
        with self._lock:
            self._execute_thread = None
