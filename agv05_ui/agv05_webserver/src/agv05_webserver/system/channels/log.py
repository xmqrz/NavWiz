from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
import os.path

from .channel import Channel
from .logtail import Logtail


class LogChannel(Channel):
    id = 'log'

    def __init__(self, *args, **kwargs):
        super(LogChannel, self).__init__(*args, **kwargs)
        filename = os.path.join(django_settings.MEDIA_ROOT, 'log/custom.log')
        self.logtail = Logtail(filename)

    def on_subscribe(self, connection):
        if self.get_num_subscribers() <= 1:
            self.logtail.start(self._publish_log)
        else:
            for line in self.logtail.iter_lines():
                self.send({
                    'id': 'line',
                    'line': line,
                }, connection)

    def on_unsubscribe(self, connection):
        if self.get_num_subscribers() <= 1:
            self.logtail.stop()

    def on_message(self, data, connection):
        pass

    def _publish_log(self, line):
        self.broadcast({
            'id': 'line',
            'line': line,
        })
