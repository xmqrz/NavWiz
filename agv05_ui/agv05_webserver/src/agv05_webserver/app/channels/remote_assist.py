from __future__ import absolute_import
from __future__ import unicode_literals

import logging
import os
import signal
import subprocess
import threading
from django.utils.functional import cached_property

from .channel import Channel

logger = logging.getLogger(__name__)


class RemoteAssistChannel(Channel):
    id = 'remote-assist'
    exec_args_tpl = 'ssh -o ConnectTimeout=3 -o ConnectionAttempts=1 -o ServerAliveInterval=3 ' + \
        '-N -R 0.0.0.0:22222:localhost:22 %s'
    hosts = ['dfhub', 'dfhub2', 'dfhub3', 'dfhub4', 'dfhub5', 'dfhub6']

    def __init__(self, *args, **kwargs):
        super(RemoteAssistChannel, self).__init__(*args, **kwargs)
        self.remote_connected = False
        self._thread = None
        self._process_handle = None

    def __del__(self):
        super(RemoteAssistChannel, self).__del__()
        self.remote_disconnect()

    @cached_property
    def exec_args(self):
        return ' || '.join([self.exec_args_tpl % n for n in self.hosts])

    def on_subscribe(self, connection):
        self.retrieve_all(connection)

    def on_unsubscribe(self, connection):
        pass

    def on_message(self, data, connection):
        try:
            if data['id'] == 'retrieve_all':
                self.retrieve_all(connection)
            elif data['id'] == 'remote_connect':
                self.remote_connect()
            elif data['id'] == 'remote_disconnect':
                self.remote_disconnect()
        except KeyError:
            pass

    def retrieve_all(self, connection):
        self.send({
            'id': 'status',
            'value': self.remote_connected,
        }, connection)

    def broadcast_status(self):
        self.broadcast({
            'id': 'status',
            'value': self.remote_connected,
        })

    def remote_connect(self):
        if self.remote_connected:
            return

        self.remote_connected = True
        self.broadcast_status()

        self._thread = threading.Thread(target=self._execute)
        self._thread.start()

    def remote_disconnect(self):
        if not self.remote_connected:
            return

        if self._process_handle:
            os.killpg(os.getpgid(self._process_handle.pid), signal.SIGTERM)

    def _execute(self):
        try:
            self._process_handle = subprocess.Popen(self.exec_args, shell=True, preexec_fn=os.setsid)
            retcode = self._process_handle.wait()
        except Exception as ex:
            logger.error('Failed to start remote assistance: %s', ex)
            retcode = -1
        finally:
            self._process_handle = None

        if retcode > 0:
            self.broadcast({
                'id': 'error',
                'value': retcode,
            })

        self.remote_connected = False
        self.broadcast_status()
