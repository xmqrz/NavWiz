from __future__ import absolute_import
from __future__ import unicode_literals

import rospy

from ..module import Module
import std_msgs.msg
import ujson as json

from .trackless_agv import TracklessAgv


class LiveApp(Module):
    id = 'live-app'

    robot_cls = []
    trackless_robot_cls = [TracklessAgv]

    def __init__(self, *args, **kwargs):
        super(LiveApp, self).__init__(*args, **kwargs)
        robot_cls = type(self).robot_cls
        if self.robot.trackless:
            robot_cls += type(self).trackless_robot_cls

        self.robot_cls = robot_cls
        self.robots = {}
        self.fms_manager = None

    def _load_robots(self):
        for cls in self.robot_cls:
            if cls.id in self.robots:
                continue
            self.robots[cls.id] = cls(self)

    def _start_robots(self):
        for robot_id in self.robots:
            self.robots[robot_id].start()

    def _stop_robots(self):
        for robot_id in self.robots:
            self.robots[robot_id].stop()

    def start(self):
        # get ref before monkey patch.
        self.fms_manager = self.robot.fms_manager if self.robot.fms_manager else None
        self.running = False

        if not self.fms_manager:
            self.__live_in_pipe = rospy.Subscriber('~live_app_in', std_msgs.msg.String, self.handle_live_in_pipe)
            self.__live_out_pipe = rospy.Publisher('~live_app_out', std_msgs.msg.String, queue_size=10)
        else:
            self.fms_manager.set_live_app_cmd_cb(self.handle_live_cmd)
            self.fms_manager.setup_live_app()

        try:
            self._load_robots()
            self.running = True
            self._start_robots()
            self.robot.base.stop()
        except Exception:
            pass

    def stop(self):
        try:
            self._stop_robots()  # allow robot to send before stop
            self.running = False
            self.robot.base.stop()
        except Exception:
            pass

    # Module Part
    def handle_in_pipe(self, data):
        try:
            cmd = data['command']
            if cmd == 'run_mode':
                self._send_run_mode()
        except KeyError as ex:
            rospy.logerr('[LiveApp] invalid input: %s', ex)

    def _send_run_mode(self):
        self.out({
            'command': 'run_mode',
            'run_mode': 'Standalone' if not self.fms_manager else 'DFleet',
        })

    # Live Robot
    def handle_live_in_pipe(self, msg):
        try:
            msg = json.loads(msg.data)
            self.handle_live_cmd(msg)
        except Exception as ex:
            rospy.logerr('[LiveApp] invalid live in pipe: %s', ex)

    def handle_live_cmd(self, msg):
        if not self.running:
            return
        try:
            robot_id = msg['id']
            if robot_id == '__':
                pass
            elif robot_id in self.robots:
                self.robots[robot_id].handle_in_pipe(msg['data'])
        except KeyError as ex:
            rospy.logerr('[LiveApp] invalid live cmd: %s', ex)

    def live_out(self, data):
        if not self.running:
            return

        if not self.fms_manager:
            self.__live_out_pipe.publish(std_msgs.msg.String(data=json.dumps(data)))
        else:
            self.fms_manager.send_live_app(data)
