from __future__ import absolute_import
from __future__ import unicode_literals

from collections import defaultdict
import diagnostic_msgs.msg
import rospy
import six

from .channel import Channel


class StatusChannel(Channel):
    id = 'status'

    def __init__(self, *args, **kwargs):
        super(StatusChannel, self).__init__(*args, **kwargs)
        self.__diagnostics_sub = None

    def on_subscribe(self, connection):
        if self.get_num_subscribers() <= 1:
            self.__diagnostics_sub = rospy.Subscriber('/diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.handle_diagnostics, queue_size=1)

    def on_unsubscribe(self, connection):
        if self.get_num_subscribers() <= 1:
            self.__diagnostics_sub.unregister()

    def on_message(self, data, connection):
        pass

    def handle_diagnostics(self, msg):
        d = dict()
        d['timestamp'] = msg.header.stamp.secs
        d['status'] = {
            'level': self._get_level(msg),
            'children': self._get_children(msg),
        }
        self.broadcast({
            'id': 'diagnostics',
            'diagnostics': d,
        })

    def _get_level(self, msg):
        level = 0
        min_level = 255

        for status in msg.status:
            if status.level > level:
                level = status.level
            if status.level < min_level:
                min_level = status.level

        # Stale items should be reported as errors unless all stale
        if level > 2 and min_level <= 2:
            level = 2

        return level

    def _get_children(self, msg):
        nodes = defaultdict(lambda: dict({'children': []}))

        for status in msg.status:
            parent, name = six.ensure_text(status.name, errors='replace').rsplit('/', 1)
            parent = parent if parent else '/'

            nodes[status.name].update({
                'level': status.level,
                'name': name,
                'message': six.ensure_text(status.message, errors='replace'),
                'hardware_id': six.ensure_text(status.hardware_id, errors='replace'),
                'values': [{
                    'key': six.ensure_text(kv.key, errors='replace'),
                    'value': six.ensure_text(kv.value, errors='replace'),
                } for kv in status.values],
            })
            nodes[parent]['children'].append(nodes[status.name])

        return nodes['/']['children']
