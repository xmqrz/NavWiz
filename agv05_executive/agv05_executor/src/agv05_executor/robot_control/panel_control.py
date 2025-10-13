from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import ErrorCode as errc
from django.utils.encoding import force_text
import agv05_msgs.msg
import logging
import rospy
import std_msgs.msg
import ujson as json

Feedback = agv05_msgs.msg.NavActionFeedback


class PanelControl(object):
    type_dict = {
        agv05_msgs.msg.UIPopup.POPUP_NONE: 'none',
        agv05_msgs.msg.UIPopup.POPUP_NON_INTERACTIVE: 'non-interactive',
        agv05_msgs.msg.UIPopup.POPUP_ALERT: 'alert',
        agv05_msgs.msg.UIPopup.POPUP_CONFIRM: 'confirm',
        agv05_msgs.msg.UIPopup.POPUP_KEYPAD: 'keypad',
    }
    safety_message_dict = {
        Feedback.STATUS_NAVIGATION_FAILED: 'Navigation failed',
        Feedback.STATUS_OUT_OF_LINE: 'Out of line',
        Feedback.STATUS_BUMPER_BLOCKED: 'Bumper blocked',
        Feedback.STATUS_EXTERNAL_SAFETY_TRIGGER: 'External safety trigger',
        Feedback.STATUS_EMERGENCY_BUTTON_PRESSED: 'Emergency button pressed',
        Feedback.STATUS_CHARGER_CONNECTED: 'Charger connected',
        Feedback.STATUS_LASER_MALFUNCTION: 'Laser malfunction',
        Feedback.STATUS_LINE_SENSOR_ERROR: 'Line sensor error',
        Feedback.STATUS_DRIVE_OVERLIMIT_ERROR: 'Drive overlimit error',
        Feedback.STATUS_MOTOR_FAULT: 'Motor fault',
        Feedback.STATUS_WHEEL_SLIPPAGE: 'Wheel slippage',
        Feedback.STATUS_SYSTEM_ERROR: 'System error',
        Feedback.STATUS_OBSTACLE_BLOCKED: 'Obstacle blocked',
        Feedback.STATUS_PLAN_EMPTY: 'Path blocked',
        Feedback.STATUS_WAIT_TRAFFIC: 'Waiting for traffic controller',
    }
    muted_list = {
        Feedback.STATUS_WAIT_TRAFFIC,
    }
    manual_resume_list = {
        Feedback.STATUS_NAVIGATION_FAILED,
        Feedback.STATUS_OUT_OF_LINE,
        Feedback.STATUS_BUMPER_BLOCKED,
        Feedback.STATUS_EXTERNAL_SAFETY_TRIGGER,
        Feedback.STATUS_EMERGENCY_BUTTON_PRESSED,
        Feedback.STATUS_CHARGER_CONNECTED,
        Feedback.STATUS_LASER_MALFUNCTION,
        Feedback.STATUS_LINE_SENSOR_ERROR,
        Feedback.STATUS_DRIVE_OVERLIMIT_ERROR,
        Feedback.STATUS_MOTOR_FAULT,
        Feedback.STATUS_WHEEL_SLIPPAGE,
        Feedback.STATUS_SYSTEM_ERROR,
        Feedback.STATUS_PLAN_EMPTY,  # manual resume required if path blocked for a prolonged period of time
    }
    safety_error_code = {
        Feedback.STATUS_NAVIGATION_FAILED: errc.NAVIGATION_FAILED,
        Feedback.STATUS_OUT_OF_LINE: errc.OUT_OF_LINE,
        Feedback.STATUS_BUMPER_BLOCKED: errc.BUMPER_BLOCKED,
        Feedback.STATUS_EXTERNAL_SAFETY_TRIGGER: errc.EXTERNAL_SAFETY_TRIGGER,
        Feedback.STATUS_EMERGENCY_BUTTON_PRESSED: errc.EMERGENCY_BUTTON_PRESSED,
        Feedback.STATUS_CHARGER_CONNECTED: errc.CHARGER_CONNECTED,
        Feedback.STATUS_LASER_MALFUNCTION: errc.LASER_MALFUNCTION,
        Feedback.STATUS_LINE_SENSOR_ERROR: errc.LINE_SENSOR_ERROR,
        Feedback.STATUS_DRIVE_OVERLIMIT_ERROR: errc.DRIVE_OVERLIMIT_ERROR,
        Feedback.STATUS_MOTOR_FAULT: errc.MOTOR_FAULT,
        Feedback.STATUS_WHEEL_SLIPPAGE: errc.WHEEL_SLIPPAGE,
        Feedback.STATUS_SYSTEM_ERROR: errc.SYSTEM_ERROR,
        Feedback.STATUS_OBSTACLE_BLOCKED: errc.OBSTACLE_BLOCKED,
        Feedback.STATUS_PLAN_EMPTY: errc.PATH_BLOCKED,
        Feedback.STATUS_WAIT_TRAFFIC: errc.WAITING_TRAFFIC,
        Feedback.STATUS_SAFETY_TRIGGERED: errc.SAFETY_TRIGGERED,
    }

    external_message_topic = 'agv05/safety/safety_external_message'
    internal_message_topic = 'agv05/safety/safety_internal_message'

    def __init__(self, robot):
        self.robot = robot
        self.running = False

        self.user_popup = None
        self.safety_popup = None
        self.safety_status = None
        self.safety_resume_list = set()
        self.safety_resume_timer = None
        self.external_message = ''
        self.internal_message = ''
        self.active_safety_message = ''

        self.safety_last_seen = rospy.get_rostime()
        self.safety_cooldown = rospy.Duration(0.5)
        self.safety_cooldown_timer = rospy.Timer(self.safety_cooldown, self.handle_safety)

        # Create all ROS subscribers and publishers
        self.robot.base.register_feedback_callback(self.handle_safety)
        self.robot.panel.register_button_release_callback(self.handle_button_release)
        self.__panel_popup_sub = rospy.Subscriber(self.robot.panel.popup_topic,
                                                  agv05_msgs.msg.UIPopup, self.handle_popup, queue_size=1)
        self.__external_message_sub = rospy.Subscriber(self.external_message_topic,
                                                       std_msgs.msg.String, self.handle_external_message, queue_size=1)
        self.__internal_message_sub = rospy.Subscriber(self.internal_message_topic,
                                                       std_msgs.msg.String, self.handle_internal_message, queue_size=1)
        self.__panel_yes_pub = rospy.Publisher(self.robot.panel.start_button_topic, std_msgs.msg.Bool, queue_size=2)
        self.__panel_no_pub = rospy.Publisher(self.robot.panel.stop_button_topic, std_msgs.msg.Bool, queue_size=2)
        self.__panel_keypad_pub = rospy.Publisher(self.robot.panel.keypad_topic, std_msgs.msg.UInt8, queue_size=1)

        self.__in_pipe = rospy.Subscriber('~panel_control_in', std_msgs.msg.String, self.handle_in_pipe)
        self.__out_pipe = rospy.Publisher('~panel_control_out', std_msgs.msg.String, queue_size=10)

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def handle_in_pipe(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            pass
        else:
            self.handle_panel_control(data)

    def out(self, data):
        self.__out_pipe.publish(std_msgs.msg.String(data=json.dumps(data)))

    def handle_panel_control(self, data):
        if not self.running:
            return
        try:
            type = data['type']
            if ((not self.user_popup or self.user_popup['type'] != type) and
                    (not self.safety_popup or self.safety_popup['type'] != type)):
                return

            if type == 'alert':
                if data['value']:
                    self.__panel_yes_pub.publish(std_msgs.msg.Bool(data=True))
                    rospy.sleep(0.01)
                    self.__panel_yes_pub.publish(std_msgs.msg.Bool(data=False))
            elif type == 'confirm':
                if data['value']:
                    self.__panel_yes_pub.publish(std_msgs.msg.Bool(data=True))
                    rospy.sleep(0.01)
                    self.__panel_yes_pub.publish(std_msgs.msg.Bool(data=False))
                else:
                    self.__panel_no_pub.publish(std_msgs.msg.Bool(data=True))
                    rospy.sleep(0.01)
                    self.__panel_no_pub.publish(std_msgs.msg.Bool(data=False))
            elif type == 'keypad':
                self.__panel_keypad_pub.publish(std_msgs.msg.UInt8(data=ord(data['value'][0])))
            elif type == 'safety-resume':
                if data['value']:
                    self.safety_resume()
        except (KeyError, ValueError):
            pass

    def safety_resume(self):
        if self.safety_resume_timer:
            self.safety_resume_timer.shutdown()
            self.safety_resume_timer = None

        self.safety_popup = {
            'type': 'safety-resuming',
            'message': 'AGV operation will resume.\nPlease stand clear of the AGV\'s path.',
            'muted': True,
            'timer': 2,
            'until': rospy.get_time() + 2 + 0.2,  # additional 0.2s to address latency in message transfer
        }
        if not self.user_popup:
            self.out(self.safety_popup)

        self.safety_resume_timer = rospy.Timer(rospy.Duration(2.0), self.publish_safety_resume, oneshot=True)

    def publish_safety_resume(self, timer_event):
        self.robot.base.safety_resume()

    def handle_button_release(self, button):
        if not self.running or self.user_popup:
            return
        if button == 'start_button':
            if self.safety_popup and self.safety_popup['type'] == 'safety-resume':
                self.safety_resume()
        elif button == 'stop_button':
            if self.safety_popup:
                if self.robot.config.stop_button_will_mute_alarm:
                    self.robot.audio.stop_alarm()

    def handle_popup(self, popup):
        if not self.running:
            return
        try:
            if popup.type == agv05_msgs.msg.UIPopup.POPUP_NONE:
                self.user_popup = None
                if self.safety_popup:
                    self.out(self.safety_popup)
                else:
                    self.out({'type': 'none'})
            else:
                self.user_popup = {
                    'type': self.type_dict[popup.type],
                    'message': popup.message,
                    'timer': popup.timer,
                    'until': rospy.get_time() + popup.timer + 0.2 if popup.timer >= 0 else None,
                }
                self.out(self.user_popup)
        except KeyError:
            pass

    def handle_external_message(self, msg):
        self.external_message = msg.data

    def handle_internal_message(self, msg):
        self.internal_message = msg.data

    def handle_safety(self, feedback):
        if isinstance(feedback, rospy.timer.TimerEvent):
            # triggered by safety_cooldown_timer
            if rospy.get_rostime() - self.safety_last_seen < self.safety_cooldown:
                return
            feedback = Feedback(status=Feedback.STATUS_NORMAL)
        elif not self.running:
            return
        else:
            self.safety_last_seen = rospy.get_rostime()

        if feedback.status == self.safety_status:
            if feedback.status == Feedback.STATUS_SAFETY_TRIGGERED:
                return
            if feedback.status == Feedback.STATUS_EXTERNAL_SAFETY_TRIGGER:
                if self.external_message == self.active_safety_message:
                    return
            elif self.internal_message == self.active_safety_message:
                return
        else:
            self.safety_status = feedback.status

        # Clear safety resume timer
        if self.safety_resume_timer:
            self.safety_resume_timer.shutdown()
            self.safety_resume_timer = None

        # Create popup for safety triggered
        if self.safety_status in self.safety_message_dict:
            safety_message = self.safety_message_dict[self.safety_status]

            # Obtain detail safety message
            self.active_safety_message = self.external_message \
                if self.safety_status == Feedback.STATUS_EXTERNAL_SAFETY_TRIGGER else self.internal_message
            if self.active_safety_message:
                safety_message = '%s: %s' % (safety_message, force_text(self.active_safety_message))

            self.safety_popup = {
                'type': 'safety-triggered',
                'message': ('The following safety checks are triggered:\n - %s\n\n' +
                    'Please clear all the safety triggers.') % safety_message,
                'muted': self.safety_status in self.muted_list,
                'timer': -1,
                'until': None,
            }
            if self.safety_status in self.manual_resume_list:
                self.safety_resume_list.add(self.safety_status)
            if not self.user_popup:
                self.out(self.safety_popup)
                self.robot.panel.set_button_led(start=False, stop=self.robot.config.stop_button_will_mute_alarm)
            if self.robot.config.log_safety_trigger and self.safety_status not in self.muted_list:
                self.robot.panel.log(logging.ERROR, self.safety_message_dict[self.safety_status])
            return

        # Create popup for safety resume
        if self.safety_status == Feedback.STATUS_SAFETY_TRIGGERED:
            self.safety_popup = {
                'type': 'safety-resume',
                'message': ('Some safety checks have been triggered previously:\n%s\n\nResume AGV operation?' %
                    '\n'.join(sorted(['- %s' % self.safety_message_dict[s] for s in self.safety_resume_list]))),
                'timer': -1,
                'until': None,
            }
            if not self.user_popup:
                self.out(self.safety_popup)
                self.robot.panel.set_button_led(start=True, stop=False)
            return

        # Clear popup if no safety issue
        self.safety_resume_list = set()
        self.safety_popup = None
        if not self.user_popup:
            self.out({'type': 'none'})
            self.robot.panel.set_button_led()

    def get_safety_message(self):
        return self.safety_popup

    def get_user_message(self):
        return self.user_popup

    def get_error_code(self):
        if self.user_popup:
            return errc.WAITING_USER
        elif self.safety_status in self.safety_error_code:
            return self.safety_error_code[self.safety_status]
        return 0
