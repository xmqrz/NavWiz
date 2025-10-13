from __future__ import absolute_import
from __future__ import unicode_literals

from django.utils import timezone
from tornado.ioloop import PeriodicCallback
import subprocess

from .channel import Channel


class NotificationChannel(Channel):
    id = 'notification'

    def __init__(self, *args, **kwargs):
        super(NotificationChannel, self).__init__(*args, **kwargs)
        self.data = {
            'disk_space': None,
        }

        self.disk_space_timer = PeriodicCallback(self.check_disk_space, 30 * 60 * 1000)  # 30 mins
        self.disk_space_timer.start()
        self.check_disk_space()

    def on_subscribe(self, connection):
        self.retrieve_all(connection)

    def on_unsubscribe(self, connection):
        pass

    def on_message(self, data, connection):
        try:
            if data['id'] == 'retrieve_all':
                self.retrieve_all(connection)
        except KeyError:
            pass

    def retrieve_all(self, connection):
        self._send_time(connection)

        if self.data['disk_space']:
            self.send(self.data['disk_space'], connection)

    def check_disk_space(self):
        try:
            p = subprocess.Popen(['df', '--output=pcent', '/'], stdout=subprocess.PIPE, universal_newlines=True)
            output = p.communicate()[0]

            free_space = 100 - int(output.split()[-1].replace('%', ''))
            data = {
                'id': 'disk_space',
                'remaining': free_space,
                'warning': free_space < 40,
            }
            self.broadcast(data)
            self.data['disk_space'] = data
        except Exception:
            pass

    def _send_time(self, connection):
        data = {
            'id': 'server_time',
            'now': timezone.localtime().isoformat()
        }
        self.send(data, connection)
