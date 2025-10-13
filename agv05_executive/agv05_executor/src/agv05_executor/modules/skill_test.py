from __future__ import absolute_import
from __future__ import unicode_literals

import rospy
import threading

from .module import Module
from ..state_machine import StateConstructError, state_factory


class SkillTest(Module):
    id = 'skill-test'

    def __init__(self, *args, **kwargs):
        super(SkillTest, self).__init__(*args, **kwargs)
        self._lock = threading.Lock()
        self._skill = None
        self._execute_thread = None

    def stop(self):
        with self._lock:
            self._stop_skill()
            self._send_updates()

    def handle_in_pipe(self, data):
        try:
            cmd = data['command']
            if cmd == 'list_all':
                self.out({
                    'command': 'list_all',
                    'skills': self._collect_skills(),
                    'stations': self._collect_stations(),
                    'registers': self._collect_registers(),
                    'variables': self._collect_variables(),
                })
            elif cmd == 'list_skills':
                self.out({
                    'command': 'list_skills',
                    'skills': self._collect_skills(),
                })
            elif cmd == 'list_stations':
                self.out({
                    'command': 'list_stations',
                    'stations': self._collect_stations(),
                })
            elif cmd == 'list_registers':
                self.out({
                    'command': 'list_registers',
                    'registers': self._collect_registers(),
                })
            elif cmd == 'list_variables':
                self.out({
                    'command': 'list_variables',
                    'registers': self._collect_variables(),
                })
            elif cmd == 'status':
                with self._lock:
                    self._send_updates()
            elif cmd == 'start_skill':
                with self._lock:
                    self._start_skill(data['skill_id'], data['skill_params'])
                    self._send_updates()
            elif cmd == 'stop_skill':
                with self._lock:
                    self._stop_skill()
                    self._send_updates()

        except KeyError as ex:
            rospy.logerr('[SkillTest] invalid input: %s', ex)
        except ValueError as ex:
            rospy.logerr('[SkillTest] invalid input: %s', ex)
        except Exception as ex:
            rospy.logerr('Exception: %s', ex)

    def _start_skill(self, skill_id, skill_params):
        if self._skill or self._execute_thread:
            return
        try:
            with self.robot.models.read_lock:
                self._skill = state_factory(self.robot, skill_id, skill_params)
        except StateConstructError as ex:
            rospy.logerr('[Skill Test] %s' % ex)
            self._send_error('%s' % ex)
            return
        except Exception as ex:
            rospy.logerr('[Skill Test] Uncaught exception: %s' % ex)
            self._send_error('Uncaught exception: %s' % ex)
            return

        self._execute_thread = threading.Thread(target=self._execute)
        self._execute_thread.start()

    def _stop_skill(self):
        if self._skill:
            self._skill.request_preempt()
            self._skill = None

        inner_sm = getattr(self.robot, 'inner_sm', None)
        if inner_sm:
            inner_sm.request_preempt()

        self.robot.base.stop()
        if self._execute_thread:
            t = self._execute_thread
            self._lock.release()
            t.join(1)
            while t.is_alive():
                self.robot.base.stop()
                t.join(1)
            self._lock.acquire()
            self._execute_thread = None

        self.robot.audio.stop_alarm()
        self.robot.audio.stop_music()

    def _execute(self):
        rospy.loginfo('[Skill Test] Executing...')
        try:
            outcome = self._skill.execute({})
        except NotImplementedError:
            rospy.logerr('[Skill Text] NotImplmentedError.')
            self._send_error('Exception: Skill has not been fully implemented.')
        except Exception as ex:
            rospy.logerr('[Skill Test] Exception: %s', ex)
            self._send_error('Exception: %s' % ex)
        else:
            rospy.loginfo('[Skill Test] Outcome: %s' % outcome)
            self._send_outcome(outcome)

        with self._lock:
            self._skill = None
            self._execute_thread = None
            self._send_updates()

    def _collect_skills(self):
        return self.robot.models.skill_descriptions

    def _collect_stations(self):
        return self.robot.models.station_names or []

    def _collect_registers(self):
        return self.robot.models.register_list

    def _collect_variables(self):
        return self.robot.models.variables or []

    def _send_updates(self):
        self.out({
            'command': 'status',
            'status': 'running' if self._skill and self._execute_thread else 'idle',
            'text': '%s' % self._skill if self._skill else '',
        })

    def _send_outcome(self, outcome):
        self.out({
            'command': 'outcome',
            'outcome': outcome,
        })

    def _send_error(self, error):
        self.out({
            'command': 'error',
            'error': error,
        })
