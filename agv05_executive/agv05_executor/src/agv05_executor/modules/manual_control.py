from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_msgs.srv import GetTwist

import geometry_msgs.msg
import math
import rospy

from .module import Module


class ManualControl(Module):
    id = 'manual-control'

    def __init__(self, *args, **kwargs):
        super(ManualControl, self).__init__(*args, **kwargs)
        self.is_motor_free = False
        self.nav = None
        self.is_omni_drive = None

    def start(self):
        try:
            self.robot.base.stop()
            self.robot.base.manual_control()
            self.is_motor_free = False
            self.nav = None
            if self.is_omni_drive is None:
                get_twist = rospy.ServiceProxy('/agv05/motor/get_twist', GetTwist)
                self.is_omni_drive = bool(get_twist(True).data.linear.y)
                get_twist.close()
        except Exception:
            pass

    def stop(self):
        try:
            self.robot.base.enable_human_follower(False, None)
            self.robot.base.register_done_callback(None)
            self.robot.base.stop()
            self.nav = None
        except Exception:
            pass

    def handle_in_pipe(self, data):
        try:
            if data['command'] == 'force_drive':
                self.robot.base.force_drive(data['vx'], data['vy'], data['vth'], data['noos'])

            elif data['command'] == 'nav':
                if not self.is_motor_free:
                    self.robot.base.register_done_callback(None)
                    self.robot.base.stop()
                    self.nav = data['nav']
                    if data['nav'] is None:
                        self.robot.base.manual_control()
                    else:
                        end = {'x': data['nav']['x'], 'y': data['nav']['y']}
                        heading = 0
                        if data['nav']['theta'] is not None:
                            heading = math.degrees(data['nav']['theta'])
                            while heading <= 0.0:
                                heading += 360.0

                        speed = float(data['speed'])
                        params = {
                            'speed': abs(speed),
                            'enable_sensor': True,
                        }
                        self.robot.base.register_done_callback(self._done_cb)
                        if speed < 0.0:
                            self.robot.base.dynamic_waypoint_reverse(end, heading, params)
                        else:
                            self.robot.base.dynamic_waypoint_forward(end, heading, params)
                self._send_status()

            elif data['command'] == 'set_pose':
                pose = geometry_msgs.msg.Pose2D(**data['set_pose'])
                self.robot.base.set_initial_pose(pose, precise=True)
                self._send_status()

            elif data['command'] == 'free_motor':
                self.robot.base.register_done_callback(None)
                self.robot.base.stop()
                self.nav = None
                if data['free_motor']:
                    self.robot.base.free_motor()
                    self.is_motor_free = True
                else:
                    self.robot.base.manual_control()
                    self.is_motor_free = False
                self._send_status()

            elif data['command'] == 'is_motor_free':
                self._send_status()
                self._send_human_follower()

            elif data['command'] == 'human_follower':
                self.robot.base.enable_human_follower(data['enable'], self._send_human_follower)

        except KeyError as ex:
            rospy.logerr('[ManualControl] invalid input: %s', ex)

    def _send_status(self):
        self.out({
            'command': 'status',
            'is_motor_free': self.is_motor_free,
            'dynamic_path_planning': self.robot.dynamic_path_planning,
            'is_omni_drive': bool(self.is_omni_drive),
        })
        self.out({
            'command': 'nav',
            'nav': self.nav,
        })

    def _send_human_follower(self, *args):
        self.out({
            'command': 'human_follower',
            'ready': self.robot.base.has_human_follower(),
            'running': self.robot.base.human_follower_running,
        })

    def _done_cb(self, feedback, result):
        self.robot.base.register_done_callback(None)
        self.nav = None
        if self.is_motor_free:
            self.robot.base.free_motor()
        else:
            self.robot.base.manual_control()
        self._send_status()
