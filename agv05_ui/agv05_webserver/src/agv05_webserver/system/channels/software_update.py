from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from functools import partial
import os.path

from .channel import Channel
from .logtail import Logtail


class SoftwareUpdateChannel(Channel):
    id = 'software-update'

    def __init__(self, *args, **kwargs):
        super(SoftwareUpdateChannel, self).__init__(*args, **kwargs)
        self._connections = {}
        self._logtails = {}

    def on_subscribe(self, connection):
        pass

    def on_unsubscribe(self, connection):
        for k, v in list(self._connections.items()):
            try:
                v.remove(connection)
            except Exception:
                pass
            else:
                if not v:
                    del self._connections[k]
                    self._logtails[k].stop()
                    del self._logtails[k]

    def on_message(self, data, connection):
        if data['id'] == 'subscribe':
            a = data['a']
            if os.path.sep in a:
                return
            filename = os.path.join(django_settings.MEDIA_ROOT, 'software-update', a)

            if filename not in self._connections:
                self._connections[filename] = {connection}
                self._logtails[filename] = Logtail(filename, 500)
                self._logtails[filename].start(partial(self._publish_log, filename))

            elif connection not in self._connections[filename]:
                self._connections[filename].add(connection)
                for line in self._logtails[filename].iter_lines():
                    self.send({
                        'id': 'line',
                        'line': line,
                    }, connection)

    def _publish_log(self, filename, line):
        for connection in self._connections[filename]:
            self.send({
                'id': 'line',
                'line': line,
            }, connection)
