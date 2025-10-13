from __future__ import absolute_import
from __future__ import unicode_literals

import rospy
import threading

from .module import Module
from ..state_machine import StateConstructError, state_factory


class Homing(Module):
    id = 'homing'
    homing_skill_id = 'agv05_skills.plan.NavigateTo'
    homing_skill_param_defaults = {
        'align_station_type': 0,
        'next_motion': 0,
        'next_speed': 0.0,
    }

    def __init__(self, *args, **kwargs):
        super(Homing, self).__init__(*args, **kwargs)
        self._lock = threading.Lock()
        self._homing_skill = None
        self._execute_thread = None

    def start(self):
        self.manager.set_agv_update_hook(self.hook_agv_update)

    def stop(self):
        with self._lock:
            self._stop_homing()
            self._send_updates()
        self.manager.set_agv_update_hook(None)

    def handle_in_pipe(self, data):
        try:
            cmd = data['command']
            if cmd == 'list_all':
                self._send_all()
            elif cmd == 'status':
                self._send_updates()
            elif cmd == 'start_homing':
                with self._lock:
                    self._start_homing(data['station'])
                    self._send_updates()
            elif cmd == 'stop_homing':
                with self._lock:
                    self._stop_homing()
                    self._send_updates()

        except KeyError as ex:
            rospy.logerr('[Homing] invalid input: %s', ex)
        except ValueError as ex:
            rospy.logerr('[Homing] invalid input: %s', ex)
        except Exception as ex:
            rospy.logerr('Exception: %s', ex)

    def hook_agv_update(self, agv):
        agv['status'] = 'Running App'
        if self.robot.fms_manager:
            if self.robot.fms_manager.pending_apply:
                with self._lock:
                    running = self._homing_skill or self._execute_thread
                    if not running and self.robot.fms_manager.apply_downloadables():
                        self._send_all()
            if not self.robot.fms_manager.is_sync_ready():
                agv['status'] = 'Running App (Sync Pending)'
        return agv

    def _start_homing(self, station):
        if self._homing_skill or self._execute_thread:
            return

        if self.robot.fms_manager and not self.robot.fms_manager.is_sync_ready():
            self._send_error('Sync Pending')
            return

        try:
            with self.robot.models.read_lock:
                self._homing_skill = state_factory(self.robot, self.homing_skill_id,
                    dict(self.homing_skill_param_defaults, station=self.robot.models.agv_home))
        except StateConstructError as ex:
            rospy.logerr('[Homing] Internal error: %s' % ex)
            self._send_error('Internal error: %s' % ex)
            return
        except Exception as ex:
            rospy.logerr('[Homing] Uncaught exception: %s' % ex)
            self._send_error('Uncaught exception: %s' % ex)
            return

        self._execute_thread = threading.Thread(target=self._execute, args=(station, ))
        self._execute_thread.start()

    def _stop_homing(self):
        if self._homing_skill:
            self._homing_skill.request_preempt()
            self._homing_skill = None

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

    def _execute(self, station):
        rospy.loginfo('[Homing] Executing...')
        try:
            location = self.robot.models.get_location_from_station(station)
            if not self.robot.models.is_valid_heading(location[1]):
                raise RuntimeError('Cannot start from a headingless station.')
            self.robot.base.set_initial_map_and_location(self.robot.models.graph, location)
            outcome = self._homing_skill.execute({})
        except Exception as ex:
            rospy.logerr('[Homing] Exception: %s', ex)
            self._send_error('Exception: %s' % ex)
        else:
            rospy.loginfo('[Homing] Outcome: %s' % outcome)
            self._send_outcome(outcome)

        with self._lock:
            self._homing_skill = None
            self._execute_thread = None
            self._send_updates()

    def _collect_stations(self):
        return self.robot.models.station_names or []

    def _send_all(self):
        self.out({
            'command': 'list_all',
            'stations': self._collect_stations(),
        })

    def _send_updates(self):
        self.out({
            'command': 'status',
            'status': 'running' if self._homing_skill and self._execute_thread else 'idle',
            'text': '%s' % self._homing_skill if self._homing_skill else '',
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
