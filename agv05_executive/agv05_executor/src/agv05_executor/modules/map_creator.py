from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_msgs.srv import GetTwist

import rospy

from .module import Module


class MapCreator(Module):
    id = 'map-creator'

    def __init__(self, *args, **kwargs):
        super(MapCreator, self).__init__(*args, **kwargs)
        self.is_omni_drive = None

    def start(self):
        try:
            self.robot.base.stop()
            self.robot.base.manual_control()
            if self.is_omni_drive is None:
                get_twist = rospy.ServiceProxy('/agv05/motor/get_twist', GetTwist)
                self.is_omni_drive = bool(get_twist(True).data.linear.y)
                get_twist.close()
        except Exception:
            pass

        self.status = 'idle'

    def stop(self):
        try:
            self.robot.base.enable_human_follower(False, None)
            self.robot.base.stop()
        except Exception:
            pass

        self._stop_mapping()

    def handle_in_pipe(self, data):
        try:
            cmd = data['command']
            if cmd == 'status':
                self._send_updates()
                self._send_human_follower()
            elif cmd == 'start_mapping':
                self._start_mapping()
            elif cmd == 'stop_mapping':
                self._stop_mapping()
            elif cmd == 'force_drive':
                self.robot.base.force_drive(data['vx'], data['vy'], data['vth'], data['noos'])
            elif cmd == 'human_follower':
                self.robot.base.enable_human_follower(data['enable'], self._send_human_follower)
        except KeyError as ex:
            rospy.logerr('[MapCreator] invalid input: %s', ex)
        except ValueError as ex:
            rospy.logerr('[MapCreator] invalid input: %s', ex)
        except Exception as ex:
            rospy.logerr('Exception: %s', ex)

    def _start_mapping(self):
        if self.status == 'mapping':
            return

        self.status = 'mapping'
        try:
            self.robot.base.reset_odom()
            self.robot.start_slam()
        except Exception as ex:
            rospy.logerr('[Map Creator] Error starting SLAM: %s', ex)

        self._send_updates()
        rospy.loginfo('[Map Creator] Mapping started.')

    def _stop_mapping(self):
        if self.status != 'mapping':
            return

        try:
            self.robot.stop_slam()
        except Exception as ex:
            rospy.logerr('[Map Creator] Error stopping SLAM: %s', ex)

        self.status = 'idle'
        self._send_updates()
        rospy.loginfo('[Map Creator] Mapping stopped.')

    def _send_updates(self):
        self.out({
            'command': 'status',
            'status': self.status,
            'is_omni_drive': bool(self.is_omni_drive),
        })

    def _send_human_follower(self, *args):
        self.out({
            'command': 'human_follower',
            'ready': self.robot.base.has_human_follower(),
            'running': self.robot.base.human_follower_running,
        })
