from __future__ import absolute_import
from __future__ import unicode_literals

import rospy
import threading
import ujson as json

from .skill import Skill


class FmsSkill(Skill):
    MAX_REQUEST_ID = 1000
    __lock = threading.Lock()
    __request_id = None

    def __init__(self, robot, **kwargs):
        super(FmsSkill, self).__init__(robot)
        self.params = kwargs

    def __str__(self):
        return 'Running DFleet skill "%s"' % self.Meta.name

    def execute(self, ud):
        rospy.loginfo('%s', self)
        assert self.robot.fms_manager

        self.request_id = self._generate_request_id()
        self.ev = threading.Event()
        self.robot.fms_manager.set_execution_feedback_cb(self.handle_execution_feedback)
        outcome = self._execute0()
        self.robot.fms_manager.set_execution_feedback_cb(None)
        return outcome

    def _execute0(self):
        self.outcome = None
        self.params_by_reference = {}

        params = {}
        for d in self.Meta.params:
            name = d['name']
            v = self.params[name]
            if d['type'] == 'Register' or d['type'].startswith('v'):
                params[name] = {
                    'id': v.id,
                    'value': v.value,
                }
            else:
                params[name] = v

        if self.preempt_requested():
            return 'Preempted'

        self._send_execution_request(params)

        while not self.ev.wait(1):
            if self.preempt_requested():
                self._send_preempt_request()
                return 'Preempted'
            self._send_execution_request(params)

        for d in self.Meta.params:
            name = d['name']
            v = self.params[name]
            if d['type'] == 'Register' or d['type'].startswith('v'):
                try:
                    v.value = self.params_by_reference[name]['value']
                except Exception:
                    pass
        return self.outcome

    def handle_execution_feedback(self, msg):
        if msg['id'] != self.request_id:
            return
        self.outcome = msg['outcome']
        self.params_by_reference = msg['params_by_reference']
        self.ev.set()

    def _send_execution_request(self, params):
        self.robot.fms_manager.send_execution_request({
            'id': self.request_id,
            'skill_id': self.Meta.id,
            'params': params,
        })

    def _send_preempt_request(self):
        self.robot.fms_manager.send_execution_request({
            'id': self.request_id,
            'preempt_requested': True,
        })

    def _generate_request_id(self):
        # Must use FmsSkill.x instead of type(self).x here
        # because fms_skill_factory creates distinct classes
        return FmsSkill.__generate_request_id()

    @classmethod
    def __generate_request_id(cls):
        with cls.__lock:
            if cls.__request_id is None:
                # Initiate request_id with a value chronological in time,
                # and greater than MAX_REQUEST_ID, to ensure that the
                # request_id will be different from the previous one.
                cls.__request_id = int(rospy.get_time() * 1000)
            else:
                cls.__request_id += 1
                if cls.__request_id >= cls.MAX_REQUEST_ID:
                    cls.__request_id = 1
            return cls.__request_id


def fms_skill_factory(fms_skill_desc):
    return type(str('FmsSkill'), (FmsSkill,), {
        'Meta': type(str('Meta'), (object,), {
            'id': fms_skill_desc['id'],
            'name': '[DFleet] %s' % fms_skill_desc['name'],
            'params': [json.loads(p) for p in fms_skill_desc['params']],
            'outcomes': fms_skill_desc['outcomes'] + ['DFleet Error'],
            'mutexes': fms_skill_desc['mutexes'],
        }),
    })
