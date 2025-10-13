from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.cfg import ExecutorConfig
from django.conf import settings as django_settings
import agv05_executive_msgs.srv
import dynamic_reconfigure.server
import importlib
import os
import rospy
import std_msgs.msg
import std_srvs.srv
import subprocess
import threading
import traceback

from .diagnostic import Diagnostic
from .models import Models
from .modules import ModuleManager
from .register import RegisterManager
from .variable import VariableManager
from .robot_control import RobotControl
from .robots import Robot, robots
from .skill import SkillManager
from .app import AppManager


class Executor():

    def __init__(self, robot):
        self.robot = robot
        self.robot.registry = RegisterManager()
        self.robot.skill_manager = SkillManager(robot)
        self.robot.app_manager = AppManager(robot)
        self.robot.models = Models(robot)
        self.robot.robot_control = RobotControl(robot)
        self.robot.variable = VariableManager(robot)
        self.module_manager = ModuleManager(robot)

        self.auto_softreboot = False

        if self.robot.models.is_valid() and self.robot.models.is_fms_mode():
            from .fms_manager import FmsManager
            self.robot.fms_manager = FmsManager(self.robot)
            self.robot.robot_control.set_fms_cb()
        else:
            self.robot.fms_manager = None

    def spin(self):
        self.__dyncfg_server = dynamic_reconfigure.server.Server(ExecutorConfig, self.reconfigure)

        self.__robot_name_pub = rospy.Publisher('~robot_name', std_msgs.msg.String, latch=True, queue_size=1)
        self.__robot_name_pub.publish(std_msgs.msg.String(data=self.robot.name))

        self.__robot_running_pub = rospy.Publisher('~robot_running', std_msgs.msg.UInt8, latch=True, queue_size=1)
        self.__robot_running_pub.publish(std_msgs.msg.UInt8(data=False))

        self.__set_robot_running_service = rospy.Service('~set_robot_running', agv05_executive_msgs.srv.SetRobotRunning, self.handle_set_robot_running)
        self.__is_robot_running_service = rospy.Service('~is_robot_running', std_srvs.srv.Trigger, self.handle_is_robot_running)

        self.diagnostic = Diagnostic(self.robot)
        self.module_manager.setup_agv_status_update(self.diagnostic)

        if rospy.get_param('~auto_start', False):
            t = threading.Timer(1.0, self.handle_set_robot_running, [agv05_executive_msgs.srv.SetRobotRunningRequest(mode=1)])
            t.start()

        if self.robot.models.is_valid() and self.robot.models.is_fms_mode():
            from .fms_comm import FmsComm
            try:
                fms_comm = FmsComm(self.robot.models.fms_metadata['broker'])
                self.robot.fms_manager.setup()
                fms_comm.spin()
            except KeyboardInterrupt:
                rospy.loginfo('[Fms Comm]: SIGINT received.')
            except Exception as ex:
                rospy.logerr('[Fms Comm]: Uncaught exception: %s', ex)
                traceback.print_exc()
        else:
            rospy.spin()
        self.diagnostic.signal_shutdown()

    def reconfigure(self, config, level):
        self.robot.config = config
        return config

    def handle_set_robot_running(self, req):
        try:
            if req.mode:
                r = self.module_manager.validate()
                if not r.valid:
                    if not r.features:
                        msg = 'The license key is invalid.'
                    elif r.days < 0:
                        msg = 'Your %s license has expired.' % ('subscription' if r.features & 1 else 'trial')
                    else:
                        msg = 'Some features are not licensed.'
                    return False, msg

                if req.mode in [1, 3] and not self.robot.models.is_valid():
                    rospy.logerr('Unable to start robot due to invalid settings: %s', self.robot.models.get_validation_msg())
                    return False, 'Invalid settings detected: %s' % self.robot.models.get_validation_msg()

                plugin_error = os.environ.get('NAVWIZ_PLUGIN_ERROR')
                if plugin_error:
                    return False, 'Hardware plugin error:\n%s' % plugin_error

                if self.robot.is_alive():
                    return True, 'Robot is already running.'

                rospy.loginfo((
                    '',
                    'Starting robot.',
                    'Starting robot for mapping.',
                    'Starting robot in live app mode.',
                )[req.mode])

                warning = self.robot.models.get_warning_msg() if req.mode in [1, 3] else ''
                if warning:
                    rospy.logwarn(warning)

                self.robot.start(req.mode)

                update_costmap = False
                if req.mode == 1 and (not self.robot.fms_manager or self.robot.fms_manager.first_synced):
                    self.robot.base.set_initial_map_and_location(
                        self.robot.models.graph,
                        self.robot.models.get_agv_home_location())
                    update_costmap = bool(self.robot.dynamic_path_planning)
                else:
                    # Temp fix: patch models so that ResetAgvPosition can work.
                    self.robot.base.map_tracker.patch_models(self.robot.models)

                self.robot.robot_control.start()
                self.module_manager.start()
                self.__robot_running_pub.publish(std_msgs.msg.UInt8(data=req.mode))
                if req.mode == 3:
                    self.auto_softreboot = True
                if update_costmap:
                    self.robot.base.set_dimension_profile(self.robot.base.get_dimension_profile())
                rospy.loginfo('Robot started.')
                return True, warning
            else:
                rospy.loginfo('Stopping robot.')
                self.module_manager.stop()
                self.robot.robot_control.stop()
                self.robot.stop()
                self.__robot_running_pub.publish(std_msgs.msg.UInt8(data=False))
                rospy.loginfo('Robot stopped.')
                if self.auto_softreboot:
                    subprocess.Popen(['sudo', 'supervisorctl', 'restart', 'agv05:*'], preexec_fn=os.setsid)
                return True, ''
        except Exception as ex:
            if req.mode:
                rospy.logerr('Error starting robot: %s', ex)
            else:
                rospy.logerr('Error stopping robot: %s', ex)
            traceback.print_exc()
            return False, '%s' % ex

    def handle_is_robot_running(self, req):
        is_alive = self.robot.is_alive()
        mode = is_alive and self.robot.get_mode()
        self.__robot_running_pub.publish(std_msgs.msg.UInt8(data=mode))
        return is_alive, ''


def main():
    # obtain robot_name
    try:
        robot_name = rospy.get_param('robot_name')
        assert robot_name in robots
        assert django_settings.TRACKLESS == robot_name.startswith('trackless')
    except Exception:
        robot_name = 'trackless_agv05' if django_settings.TRACKLESS else 'tracked_agv05'
        rospy.logwarn('Invalid robot_name. Defaulting to %s', robot_name)

    try:
        module_name, cls_name = robots.get(robot_name).rsplit('.', 1)
        module = importlib.import_module(module_name)
        robot_cls = getattr(module, cls_name)
        if issubclass(robot_cls, Robot):
            robot = robot_cls()
        else:
            raise RuntimeError()
    except Exception as ex:
        rospy.logfatal('Cannot find robot matching robot_name: %s. Exception: %s. Exiting...', robot_name, ex)
        traceback.print_exc()
        return 1

    # set dynamic_path_planning attribute
    robot.dynamic_path_planning = django_settings.DYNAMIC_PATH_PLANNING

    # create and run executor
    executor = Executor(robot)
    executor.spin()
