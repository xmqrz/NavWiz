from __future__ import absolute_import
from __future__ import unicode_literals


class Module(object):
    id = '__module__'

    def __init__(self, manager):
        self.manager = manager
        self.robot = manager.robot

    def start(self):
        pass

    def stop(self):
        pass

    def handle_in_pipe(self, data):
        raise NotImplementedError()

    def out(self, data):
        if self.robot.get_reservation() != self.id:
            return
        self.manager.out({
            'id': self.id,
            'data': data,
        })
