from __future__ import absolute_import
from __future__ import unicode_literals


class LiveRobot(object):
    id = '__robot__'

    def __init__(self, module):
        self.module = module
        self.robot = module.robot

    def start(self):
        pass

    def stop(self):
        pass

    def handle_in_pipe(self, data):
        raise NotImplementedError()

    def out(self, data):
        self.module.live_out({
            'id': self.id,
            'data': data,
        })
