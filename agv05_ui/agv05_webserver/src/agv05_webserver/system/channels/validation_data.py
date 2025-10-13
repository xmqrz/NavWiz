from __future__ import absolute_import
from __future__ import unicode_literals

from django.core.cache import cache
from django.utils.html import strip_tags
from tornado.ioloop import PeriodicCallback
import hashlib
import logging
import ujson as json

from agv05_webserver.system.models import Cache, db_auto_reconnect
from .channel import Channel

logger = logging.getLogger(__name__)


class ValidationDataChannel(Channel):
    id = 'validation-data'

    def __init__(self, *args, **kwargs):
        self.old_hash = ''
        super(ValidationDataChannel, self).__init__(*args, **kwargs)

    def on_subscribe(self, connection):
        self.retrieve_all(connection)
        if self.get_num_subscribers() <= 1:
            self._start()

    def on_unsubscribe(self, connection):
        if self.get_num_subscribers() <= 1:
            self._stop()

    def on_message(self, data, connection):
        pass

    def _start(self):
        self.__timer = PeriodicCallback(self.timer, 3 * 1000)  # 3 seconds
        self.__timer.start()

    def _stop(self):
        self.__timer.stop()

    def timer(self):
        self.retrieve_all()

    @db_auto_reconnect()
    def retrieve_all(self, connection=None):
        data = {
            'id': 'validation-data',
            'state': 'clean',
            'msg': '',
            'params': {},
        }

        flag = Cache.get_flag()
        if flag in [Cache.Flag.Dirty.value, Cache.Flag.Validating.value]:
            data['state'] = 'validating'
            data['msg'] = 'Changes are being validated in the background.'
        elif flag == Cache.Flag.Invalid.value:
            validation_data = Cache.get_validation_data()
            data['state'] = 'invalid'
            if validation_data:
                data['msg'] = validation_data['error_msg']
                data['params'] = validation_data['params']
            else:
                # HACK: get message from older version.
                data['msg'] = strip_tags(cache.get('validation_msg', 'Changes are invalid but failed to obtain validation message. Please trigger invalidation.'))
                data['params'] = {}
        elif flag == Cache.Flag.Valid.value:
            data['state'] = 'valid'
            data['msg'] = 'Changes are valid. Please perform a soft reboot to apply the changes.'
        elif flag == Cache.Flag.Reloadable.value:
            data['state'] = 'reloadable'
            data['msg'] = 'Changes are valid. Please perform a hot reload or soft reboot to apply the changes.'

        new_hash = hashlib.md5(json.dumps(data).encode('utf-8')).hexdigest()
        if connection:
            self.send(data, connection)
        elif self.old_hash != new_hash:
            self.broadcast(data)
        self.old_hash = new_hash
