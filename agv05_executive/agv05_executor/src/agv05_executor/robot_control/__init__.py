from __future__ import absolute_import
from __future__ import unicode_literals

from .app_control import AppControl
from .boot_control import BootControl
from .panel_control import PanelControl


class RobotControl(object):

    def __init__(self, robot):
        self.robot = robot
        self.app_control = AppControl(robot)
        self.boot_control = BootControl(robot)
        self.panel_control = PanelControl(robot)

    def start(self):
        self.app_control.start()
        self.boot_control.start()
        self.panel_control.start()

    def stop(self):
        self.app_control.stop()
        self.boot_control.stop()
        self.panel_control.stop()

    def set_fms_cb(self):
        assert self.robot.fms_manager
        self.robot.fms_manager.set_robot_control_cb(self.handle_robot_control)

    def handle_robot_control(self, data):
        try:
            control = data['control']
        except KeyError:
            return

        if control == 'app':
            self.app_control.handle_app_control(data)
        elif control == 'boot':
            self.boot_control.handle_boot_control(data)
        elif control == 'panel':
            self.panel_control.handle_panel_control(data)
