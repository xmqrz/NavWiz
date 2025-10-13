from __future__ import absolute_import
from __future__ import unicode_literals

import rospkg
import rospy

from .module import Module


class DevelopmentMode(Module):
    id = 'dev-mode'

    def __init__(self, *args, **kwargs):
        super(DevelopmentMode, self).__init__(*args, **kwargs)
        self._capab_running = False
        self._server_version = ''

    def start(self):
        try:
            self.robot.add_capability('dev_mode_capabilities/DevModeServer', 'dev_mode_capabilities/dev_mode_server')
            self._capab_running = True
        except Exception:
            return

        try:
            self._server_version = rospkg.RosPack().get_manifest('dev_mode_server').version
        except Exception:
            pass

    def stop(self):
        if self._capab_running:
            try:
                self.robot.remove_capability('dev_mode_capabilities/DevModeServer')
            except Exception:
                pass
        self._capab_running = False

    def handle_in_pipe(self, data):
        try:
            cmd = data['command']
            if cmd == 'status':
                self._send_updates()

        except KeyError as ex:
            rospy.logerr('[DevMode] invalid input: %s', ex)
        except ValueError as ex:
            rospy.logerr('[DevMode] invalid input: %s', ex)
        except Exception as ex:
            rospy.logerr('Exception: %s', ex)

    def _send_updates(self):
        self.out({
            'command': 'status',
            'status': 'running' if self._capab_running else 'idle',
            'server_version': self._server_version,
        })
