from __future__ import absolute_import
from __future__ import unicode_literals

import rospy

from .module import Module


class ManualLineFollow(Module):
    id = 'manual-line-follow'

    def __init__(self, *args, **kwargs):
        super(ManualLineFollow, self).__init__(*args, **kwargs)
        self.is_motor_free = False

    def start(self):
        try:
            self.robot.base.stop()
            self.is_motor_free = False
        except Exception:
            pass

    def stop(self):
        try:
            self.robot.base.stop()
        except Exception:
            pass

    def handle_in_pipe(self, data):
        try:
            cmd = data['command']
            if cmd == 'list_allowed_motions':
                self.out({
                    'command': 'list_allowed_motions',
                    'allowed_motions': list(self.robot.models.allowed_motions),
                })

            elif cmd == 'nav':
                base_method = data['nav']
                if base_method == 'stop' or base_method in self.robot.models.allowed_motions:
                    getattr(self.robot.base, base_method)({
                        'speed': data['speed'],
                        'next_motion': 0,
                        'enable_sensor': True,
                    })

            elif cmd == 'free_motor':
                if data['free_motor']:
                    self.robot.base.stop()
                    self.robot.base.free_motor()
                    self.is_motor_free = True
                else:
                    self.robot.base.stop()
                    self.is_motor_free = False
                self._send_status()

            elif cmd == 'is_motor_free':
                self._send_status()

        except KeyError as ex:
            rospy.logerr('[ManualLineFollow] invalid input: %s', ex)
        except ValueError as ex:
            rospy.logerr('[ManualLineFollow] invalid input: %s', ex)
        except Exception as ex:
            rospy.logerr('Exception: %s', ex)

    def _send_status(self):
        self.out({
            'command': 'is_motor_free',
            'is_motor_free': self.is_motor_free,
        })
