from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_msgs.srv import GetTwist
from agv05_webserver.system.models import ErrorCode, Task, Variable, db_auto_reconnect
from django.db import transaction
from django.utils.encoding import force_text
from functools import partial
from six.moves import queue as Queue
import agv05_executive_msgs.srv
import geometry_msgs.msg
import logging
import requests
import rospy
import std_msgs.msg
import threading
import time
import traceback
import ujson as json

from .module import Module
from ..skill import UserError
from ..state_machine import StateConstructError, set_pause_check, state_machine_factory

LAST_MODIFIED_TIMEOUT = 10


class TaskRunner(Module):
    id = 'task-runner'

    unhandled_error_msg = 'Unhandled error. Please check the log files for more details.'

    def __init__(self, *args, **kwargs):
        super(TaskRunner, self).__init__(*args, **kwargs)
        self.running = False
        self.paused = False
        self.resuming = False
        self.resume_timer = None
        self.is_omni_drive = None
        self._lock = threading.Lock()
        self._execute_thread = None
        self._task = None
        self._sm = None
        self._sm_outcome = ''
        self._init_queue = Queue.Queue()
        self._custom_init_sm = None
        self.__new_task_evt = threading.Event()
        self._sm_action = ''
        self._sm_call_stack = []

        self.robot.panel.register_button_release_callback(self.handle_button_release)

        self.__task_runner_initialized_pub = rospy.Publisher('~task_runner_initialized', std_msgs.msg.Bool, queue_size=1, latch=True)
        self.__task_runner_initialized_pub.publish(std_msgs.msg.Bool(data=False))

        self.__create_task_service = rospy.Service('~create_task', agv05_executive_msgs.srv.TriggerTask, self.handle_create_task)
        self.__abort_task_service = rospy.Service('~abort_task', agv05_executive_msgs.srv.TriggerTask, self.handle_abort_task)
        self.__change_task_service = rospy.Service('~change_task', agv05_executive_msgs.srv.TriggerTask, self.handle_change_task)

        self.__fms_polling_thread = None
        self.__fms_polling_last_modified = None
        self.__fms_polling_last_request = 0
        self.__fms_polling_last_modified_transaction = None
        self.__fms_polling_last_request_transaction = 0
        self.__fms_task_list = []
        self.__fms_transaction_lists = []
        self.__load_fms_unsynced_tasks()
        self.__last_task_id = None
        self.__abortable_task_id = None
        self.__sm_monitor_timer = None

        self.__executing_transaction = False
        self.__transaction_enabled = False

    @db_auto_reconnect()
    def start(self):
        self.robot.registry.init_global()
        self.robot.variable.init()
        self.running = True
        self.paused = False
        self.resuming = False

        self.__new_task_evt.clear()
        with transaction.atomic():
            self._remove_all_tasks(Task.objects.running().select_for_update().defer('owner'),
                stale_only=not self.robot.config.clear_pending_tasks_when_task_runner_starts)
        self._task = None

        try:
            if self.is_omni_drive is None:
                get_twist = rospy.ServiceProxy('/agv05/motor/get_twist', GetTwist)
                self.is_omni_drive = bool(get_twist(True).data.linear.y)
                get_twist.close()
        except Exception:
            pass

        self.init_status = 'unaware'
        self._send_init_status()
        self.default_init = None

        self.__batt_below_min = False
        self.__batt_was_below_min = False
        self.__approved_task_id = None

        self.__task_runner_command_sub = rospy.Subscriber('~task_runner_command', std_msgs.msg.String, self.handle_task_runner_command, queue_size=1)

        if self.robot.models.default_paused:
            self.pause()

        self.__executing_transaction = False

        self.__transaction_enabled = False
        if self.robot.fms_manager:
            self.__transaction_enabled = self.robot.models.transaction_enabled

        self._execute_thread = threading.Thread(target=self.run)
        self._execute_thread.start()

    @db_auto_reconnect()
    def stop(self):
        if self.init_status == 'initialized':
            Variable.objects.update_or_create(name='/agv05_executor/previous_location', defaults={
                'value': json.dumps(self.robot.base.get_prev_location()),
            })

        self.__task_runner_initialized_pub.publish(std_msgs.msg.Bool(data=False))
        self.running = False

        custom_init_sm = self._custom_init_sm  # make a copy to prevent race condition
        if custom_init_sm:
            custom_init_sm.request_preempt()

        with self._lock:
            if self._task:
                self._stop_task()

        if self.__sm_monitor_timer:
            self.__sm_monitor_timer.shutdown()
            self.__sm_monitor_timer = None

        inner_sm = getattr(self.robot, 'inner_sm', None)
        if inner_sm:
            inner_sm.request_preempt()

        self.__task_runner_command_sub.unregister()

        self.robot.base.enable_human_follower(False, None)
        self.robot.base.stop()
        if self._execute_thread:
            t = self._execute_thread
            t.join(1)
            while t.is_alive():
                self.robot.base.stop()
                t.join(1)
            self._execute_thread = None

        self.robot.audio.stop_alarm()
        self.robot.audio.stop_music()
        self.robot.panel.toggle_led_paused(None)

        set_pause_check(None)
        self.robot.models.update_task_progress = None
        self.manager.set_agv_update_hook(None)
        if self.robot.fms_manager:
            self.robot.fms_manager.set_navigation_approval_cb(None)
            self.robot.fms_manager.set_task_assignment_cb(None)
            self.robot.fms_manager.set_task_abort_cb(None)
            self.robot.fms_manager.set_task_runner_command_cb(None)
            if self.__fms_polling_thread:
                self.__fms_polling_thread.join()
                self.__fms_polling_thread = None
            self.__fms_polling_last_modified = None
            self.__fms_polling_last_modified_transaction = None
            self.__fms_task_list = []
            self.__fms_transaction_lists = []
            self.__save_fms_unsynced_tasks()

        self.robot.registry.init_global()
        self.robot.variable.init()

        self.__executing_transaction = False

    def pause(self):
        with self._lock:
            if self.paused and not self.resuming:
                return
            self.paused = True
            self.resuming = False

            if self.resume_timer:
                self.resume_timer.shutdown()
                self.resume_timer = None

            self.robot.base.pause()
            self.robot.panel.toggle_led_paused(True)
            self._send_pause_status()

    def __is_paused(self, trigger=False):
        if trigger:
            self.pause()
        return self.paused

    def resume(self):
        with self._lock:
            if not self.paused or self.resuming:
                return
            self.resuming = True
            self.robot.base.enable_human_follower(False, None)

            if self.resume_timer:
                self.resume_timer.shutdown()
                self.resume_timer = None
            self.resume_timer = rospy.Timer(rospy.Duration(2.0), self._resume, oneshot=True)
            self._send_pause_status()

    def _resume(self, timer_event):
        with self._lock:
            if not self.paused or not self.resuming:
                return
            self.paused = False
            self.resuming = False

            self.robot.base.resume()
            self.robot.panel.toggle_led_paused(False)
            self._send_pause_status()

    def handle_task_runner_command(self, msg):
        try:
            data = json.loads(msg.data)
            self.handle_in_pipe(data)
        except Exception as ex:
            rospy.logerr('Exception in task_runner_command subscriber: %s', ex)

    @db_auto_reconnect()
    def handle_in_pipe(self, data):
        try:
            cmd = data['command']
            if cmd == 'list_templates':
                self._send_templates()
            elif cmd == 'list_tasks':
                self._send_updates()
            elif cmd == 'list_transactions':
                self._send_transaction_updates()
            elif cmd == 'get_action':
                self._send_action()
            elif cmd == 'init_status':
                self._send_init_status()
                self._resend_fms_status()
            elif cmd == 'location_hint' and self.init_status == 'unaware':
                if self.robot.fms_manager and not self.robot.fms_manager.is_sync_ready():
                    return
                if data['hint'] == 'home':
                    self.init_status = data['hint']
                    self._send_init_status()
                    self._init_queue.put(self._dock)
                elif data['hint'] == 'forward_dock':
                    self.init_status = data['hint']
                    self._send_init_status()
                    self._init_queue.put(self._forward_dock)
                elif data['hint'] == 'reverse_dock':
                    self.init_status = data['hint']
                    self._send_init_status()
                    self._init_queue.put(self._reverse_dock)
                elif data['hint'] == 'station':
                    self.init_status = data['hint']
                    self._send_init_status()
                    self._init_queue.put(partial(self._reset_position, data['station']))
                elif data['hint'].startswith('custom_init_'):
                    task_template_id = int(data['hint'][len('custom_init_'):])
                    if task_template_id not in self.robot.models.custom_init:
                        return
                    self.init_status = data['hint']
                    self._send_init_status()
                    params = data.get('params', {})
                    self._init_queue.put(partial(self._custom_init, task_template_id, params))
            elif cmd == 'pre_init' and self.init_status == 'unaware':
                if not data['type'].startswith('pre_init_'):
                    return
                task_template_id = int(data['type'][len('pre_init_'):])
                if task_template_id not in self.robot.models.pre_init:
                    return
                self.init_status = 'pre_init'
                self._send_init_status()
                params = data.get('params', {})
                self._init_queue.put(partial(self._pre_init, task_template_id, params))
            elif cmd == 'get_default_init':
                self._send_default_init()
            elif cmd == 'cancel_default_init' and self.init_status == 'unaware':
                self._init_queue.put(cmd)
            elif cmd == 'is_paused':
                self._send_pause_status()
            elif cmd == 'pause':
                self.pause() if data['pause'] else self.resume()
            elif cmd == 'hot_reloaded':
                self._send_templates()
            elif cmd == 'set_pose':
                pose = geometry_msgs.msg.Pose2D(**data['set_pose'])
                self.robot.base.set_initial_pose(pose, precise=True)
            elif cmd == 'force_drive':
                self.pause()
                if self.robot.base.is_idle():
                    self.robot.base.manual_control({'speed': -1.0})
                self.robot.base.force_drive(data['vx'], data['vy'], data['vth'], data['noos'])
            elif cmd == 'manual_control_status':
                self._send_manual_control()
                self._send_human_follower()
            elif cmd == 'human_follower':
                enable = data['enable']
                if enable:
                    self.pause()
                self.robot.base.enable_human_follower(enable, self._send_human_follower)

        except KeyError as ex:
            rospy.logerr('[TaskRunner] invalid input: %s', ex)
        except ValueError as ex:
            rospy.logerr('[TaskRunner] invalid input: %s', ex)
        except Exception as ex:
            rospy.logerr('Exception: %s', ex)

    def handle_button_release(self, button):
        if not self.running or self.init_status != 'initialized':
            return
        if button == 'start_button':
            if self.robot.config.start_button_will_resume_action:
                self.resume()
        elif button == 'stop_button':
            if self.robot.config.stop_button_will_pause_action:
                self.pause()

    @db_auto_reconnect()
    def handle_create_task(self, req):
        if not self.running:
            return False, 'Task runner is not running.'

        self._abort_abortable_task()
        self.__new_task_evt.set()
        self._send_updates()
        return True, ''

    @db_auto_reconnect()
    def handle_abort_task(self, req):
        if not self.running:
            return False, 'Task runner is not running.'

        with self._lock:
            if (self._task and self._task.id == req.task_id and
                    self._task.status < Task.Status.Aborting.value):
                self._task.status = Task.Status.Aborting.value
                self._stop_task()
                self._send_updates()
                self._fms_send_update(self._task)
        return True, ''

    @db_auto_reconnect()
    def handle_change_task(self, req):
        if not self.running:
            return False, 'Task runner is not running.'

        self._send_updates()
        return True, ''

    @db_auto_reconnect()
    def handle_task_assignment(self, data):
        fms_task_id = data['id']

        if (Task.objects.running().filter(fms_task_id=fms_task_id).exists() or
                fms_task_id in self.__fms_agv_recent_tasks or
                fms_task_id in self.__fms_agv_unsynced_tasks):
            return

        task_params = data.get('params', {})
        self._add_task(data['template_id'], task_params, data['name'], fms_task_id)

    @db_auto_reconnect()
    def handle_task_abort(self, data):
        self._abort_task(data['id'])

    @db_auto_reconnect()
    def hook_agv_update(self, agv):
        if self.robot.fms_manager and self.robot.fms_manager.is_sync_pending():
            agv['status'] = 'Sync Pending' if not self.robot.fms_manager.reloadable else 'Syncing'
            return agv

        if Task.objects.running().exists() or self.__executing_transaction:
            agv['status'] = 'Paused' if self.paused else 'Working'
        else:
            agv['status'] = 'Suspended' if self.paused else 'Idle'

        if self.paused and not agv['error_code']:
            agv['error_code'] = ErrorCode.PAUSED

        agv['action'] = self._sm_action
        return agv

    def run_fms_polling(self):
        while self.running and not rospy.is_shutdown():
            rospy.sleep(1.0)
            self._fms_poll_task_list()
            # TODO: shoulde we use diff thread?
            self._fms_poll_transaction_list()

    @db_auto_reconnect()
    def _fms_poll_task_list(self):
        if not self.running or rospy.is_shutdown():
            return

        agv_uuid = self.robot.models.agv_uuid
        fms_metadata = self.robot.models.fms_metadata

        endpoint = '%s/running?limit=100' % fms_metadata['task_endpoint']
        headers = {
            'Authorization': 'AgvToken %s:%s' % (agv_uuid, fms_metadata['token']),
            'Content-Type': 'application/json',
        }
        now = time.time()
        if (self.__fms_polling_last_modified and
                now - self.__fms_polling_last_request < LAST_MODIFIED_TIMEOUT):
            headers['If-Modified-Since'] = self.__fms_polling_last_modified

        next_endpoint = endpoint
        error = False
        results = []
        last_modified = None

        while self.running and not rospy.is_shutdown() and next_endpoint and not error:
            try:
                r = requests.get(next_endpoint, headers=headers, timeout=(3.05, 7), verify=False)
                if r.status_code == 200:
                    if not last_modified:
                        last_modified = r.headers.get('Last-Modified')

                    data = json.loads(r.content)
                    next_endpoint = data['next']
                    results += data['results']
                else:
                    error = True
            except Exception:
                error = True

        if not self.running or rospy.is_shutdown() or error:
            return

        self.__fms_polling_last_modified = last_modified
        self.__fms_polling_last_request = now

        with self._lock:
            self.__fms_agv_unsynced_tasks.update(self.__fms_agv_recent_tasks)
            self.__fms_agv_recent_tasks = {}

        active_task = self._task
        task_list = []
        agv_unsynced_tasks = {}

        for t in results:
            try:
                t['status'] = Task.Status[t['status'].capitalize()].value
            except Exception:
                continue

            if active_task and active_task.fms_task_id == t['id']:
                if t['status'] > active_task.status:
                    if t['status'] == Task.Status.Aborting.value:
                        self._abort_task(t['id'], send_updates=False)

                elif t['status'] < active_task.status:
                    if active_task.status == Task.Status.In_progress.value:
                        self._fms_send_update(active_task)

            elif t['id'] in self.__fms_agv_unsynced_tasks:
                self._fms_send_update(self.__fms_agv_unsynced_tasks[t['id']])
                agv_unsynced_tasks[t['id']] = self.__fms_agv_unsynced_tasks[t['id']]

            else:
                task_list.append(t)

        self.__fms_task_list = task_list
        self.__fms_agv_unsynced_tasks = agv_unsynced_tasks
        self._send_updates()

    @db_auto_reconnect()
    def _fms_poll_transaction_list(self):
        if not self.running or not self.__transaction_enabled or rospy.is_shutdown():
            return

        agv_uuid = self.robot.models.agv_uuid
        fms_metadata = self.robot.models.fms_metadata

        endpoint = '%s/running?limit=100' % fms_metadata['transaction_endpoint']
        headers = {
            'Authorization': 'AgvToken %s:%s' % (agv_uuid, fms_metadata['token']),
            'Content-Type': 'application/json',
        }
        now = time.time()
        if (self.__fms_polling_last_modified_transaction and
                now - self.__fms_polling_last_request_transaction < LAST_MODIFIED_TIMEOUT):
            headers['If-Modified-Since'] = self.__fms_polling_last_modified_transaction

        next_endpoint = endpoint
        error = False
        results = []
        last_modified = None

        while self.running and not rospy.is_shutdown() and next_endpoint and not error:
            try:
                r = requests.get(next_endpoint, headers=headers, timeout=(3.05, 7), verify=False)
                if r.status_code == 200:
                    if not last_modified:
                        last_modified = r.headers.get('Last-Modified')

                    data = json.loads(r.content)
                    next_endpoint = data['next']
                    results += data['results']
                else:
                    error = True
            except Exception:
                error = True

        if not self.running or rospy.is_shutdown() or error:
            return

        self.__fms_polling_last_modified_transaction = last_modified
        self.__fms_polling_last_request_transaction = now

        agv_uuid = self.robot.models.agv_uuid
        agv_name = self.robot.models.agv_name
        executing_transaction = False
        for t in results:
            if t['agv_id'] == agv_uuid:
                t['agv_id'] = True
                t['agv'] = agv_name
                if t['status'] in \
                        ['waiting', 'cancelling', 'paused', 'transferring', 'aborting']:
                    executing_transaction = True
            else:
                t['agv_id'] = False

        self.__executing_transaction = executing_transaction
        self.__fms_transaction_lists = results
        self._send_transaction_updates()

    def _add_task(self, template_id, params={}, name=None, fms_task_id=None):
        """Helper to add task. Initiated by fms or internal trigger only."""
        with self.robot.models.read_lock:
            if not self.robot.models.task_templates_assoc:
                return
            tt = self.robot.models.task_templates_assoc.get(template_id)
        if not tt:
            rospy.logerr('[Task Runner] TaskTemplate is not found.')
            self._send_error('', 'TaskTemplate is not found.')
            self._fms_send_update({
                'id': fms_task_id,
                'status': 'Aborted',
                'progress': '',
            })
            return

        task = Task(name=name or tt['name'],
            task_template_id=tt['id'],
            task_template=tt['name'],
            params=json.dumps(params),
            fms_task_id=fms_task_id,
            status=Task.Status.Pending.value)

        task.save()

        self.__approved_task_id = task.id  # overwrite min-batt and execute this task
        self.__new_task_evt.set()
        self._send_updates()
        return task.id

    def _abort_task(self, fms_task_id, send_updates=True):
        """Helper to abort task. Initiated by fms only."""
        with self._lock:
            if (self._task and self._task.fms_task_id == fms_task_id and
                    self._task.status < Task.Status.Aborting.value):
                self._task.abort()
                self._stop_task()
                if send_updates:
                    self._send_updates()

    def _abort_abortable_task(self, send_updates=True):
        """Abort running abortable task."""
        if self._is_batt_below_min():
            return
        with self._lock:
            if (self._task and self.__abortable_task_id and self._task.id == self.__abortable_task_id and
                    self._task.status < Task.Status.Aborting.value):
                self._task.abort()
                self._stop_task()
                if send_updates:
                    self._send_updates()

    def _stop_task(self):
        if self._sm:
            self._sm.request_preempt()
            self.robot.audio.stop_alarm()
            self.robot.audio.stop_music()

        inner_sm = getattr(self.robot, 'inner_sm', None)
        if inner_sm:
            inner_sm.request_preempt()

    def run(self):
        self.default_init = self._collect_default_init()
        self._send_default_init()

        idle_since = rospy.get_time()
        idle_trigger = self.robot.models.task_triggers['agv_idle'] if not self.robot.fms_manager else None
        idle_trigger_active = False
        idle_trigger_task_id = None
        batt_low_trigger = self.robot.models.task_triggers['battery_low'] if not self.robot.fms_manager else None
        batt_low_trigger_active = False
        batt_low_trigger_task_id = None

        while self.running and not rospy.is_shutdown():
            if self.init_status != 'initialized':
                if self._execute_init():
                    if self.default_init != 0:
                        self.default_init = 0  # use zero(0) to mark as initialized
                        self._send_default_init()
                idle_since = rospy.get_time()

            elif self._execute_paused() or self._execute_tasks():
                idle_since = rospy.get_time()
                if self.__last_task_id:
                    if self.__last_task_id != idle_trigger_task_id:
                        idle_trigger_active = False
                    if self.__last_task_id != batt_low_trigger_task_id:
                        batt_low_trigger_active = False

            # idle trigger
            elif not self.robot.fms_manager and idle_trigger and \
                    rospy.get_time() - idle_since > idle_trigger['timeout'] and \
                    not ((idle_trigger['ignore_home'] and self._is_agv_at_home()) or
                        any(self._is_agv_at_station(s) for s in idle_trigger['ignore_stations']) or
                        (idle_trigger['ignore_charging'] and self.robot.power.charging_status) or
                        (idle_trigger['ignore_low_battery'] and self._is_batt_below_min()) or
                        (idle_trigger['ignore_trigger_active'] and idle_trigger_active)):
                idle_trigger_active = True
                with db_auto_reconnect():
                    idle_trigger_task_id = self._add_task(idle_trigger['action']['template_id'],
                            params=idle_trigger['action']['params'], name='AGV Idle Task')

                if idle_trigger['abortable']:
                    self.__abortable_task_id = idle_trigger_task_id

            # battery low trigger
            elif not self.robot.fms_manager and batt_low_trigger and \
                    self.robot.power.battery_percentage < batt_low_trigger['threshold'] and \
                    not ((batt_low_trigger['ignore_home'] and self._is_agv_at_home()) or
                        any(self._is_agv_at_station(s) for s in batt_low_trigger['ignore_stations']) or
                        (batt_low_trigger['ignore_charging'] and self.robot.power.charging_status) or
                        (batt_low_trigger['ignore_low_battery'] and self._is_batt_below_min()) or
                        (batt_low_trigger['ignore_trigger_active'] and batt_low_trigger_active)):
                batt_low_trigger_active = True
                with db_auto_reconnect():
                    batt_low_trigger_task_id = self._add_task(batt_low_trigger['action']['template_id'],
                            params=batt_low_trigger['action']['params'], name='Battery Low Task')

                if batt_low_trigger['abortable']:
                    self.__abortable_task_id = batt_low_trigger_task_id

    def _execute_init(self):
        """Returns True to clear default_init."""
        init_func = None
        try:
            init_func = self._init_queue.get(timeout=1)
        except Queue.Empty:
            pass

        if init_func:
            # check if cancel_default_init requested
            if init_func == 'cancel_default_init':
                return True

            # check if it is pre_init type
            if getattr(init_func, 'func', None) == self._pre_init:
                init_func()
                return True

        if self.robot.fms_manager:
            if self.robot.fms_manager.pending_apply:
                if self.init_status not in ['unaware', 'pre_init']:
                    self.init_status = 'unaware'
                hot_reloadable = self.robot.fms_manager.reloadable
                if self.robot.fms_manager.apply_downloadables():
                    self._send_templates()
                    if self.default_init != 0:
                        self.default_init = self._collect_default_init()
                        self._send_default_init()
                    if not hot_reloadable:
                        self.robot.variable.init()
                return

            if not self.robot.fms_manager.is_sync_ready():
                if self.init_status not in ['unaware', 'pre_init']:
                    self.init_status = 'unaware'
                return

        if init_func:
            init_func()
            return True

        if not self.default_init:
            return

        # countdown and trigger default_init
        self.default_init['countdown'] -= 1
        self._send_default_init()

        if self.default_init['countdown'] <= 0:
            self.handle_in_pipe({
                'command': 'location_hint',
                'hint': self.default_init['type'],
            })

    def _execute_paused(self):
        """Returns True if paused."""
        if not self.paused:
            return
        rospy.sleep(1)
        if self.robot.fms_manager:
            if self.robot.fms_manager.pending_apply and self._is_agv_at_home():
                hot_reloadable = self.robot.fms_manager.reloadable
                if self.robot.fms_manager.apply_downloadables():
                    self._set_location_to_home()
                    self._send_templates()
                    if not hot_reloadable:
                        self.robot.variable.init()
                    self.__new_task_evt.set()  # trigger check for new task
            elif self.robot.fms_manager.pending_apply and self.robot.fms_manager.reloadable:
                if self.robot.fms_manager.hot_reload_downloadables():
                    self._send_templates()
                    self.__new_task_evt.set()  # trigger check for new task
        return True

    def _execute_tasks(self):
        """Returns True to reset idle_since."""
        has_tasks = self.__new_task_evt.wait(1)

        if not self._is_batt_below_min() and self.__batt_was_below_min:
            has_tasks = True  # battery recovered so trigger check for new task

        self.__batt_was_below_min = self.__batt_below_min

        if self.robot.fms_manager:
            if self.robot.fms_manager.pending_apply and self._is_agv_at_home():
                # To backup the value because apply_downloadables() will change it
                hot_reloadable = self.robot.fms_manager.reloadable

                if self.robot.fms_manager.apply_downloadables():
                    self._set_location_to_home()
                    self._send_templates()

                    # Restart variable manager after updating variable models
                    # Only if hot reloadable is false (to avoid modifying value of current variables)
                    if not hot_reloadable:
                        self.robot.variable.init()

                    has_tasks = True  # trigger check for new task
            elif self.robot.fms_manager.pending_apply and self.robot.fms_manager.reloadable:
                if self.robot.fms_manager.hot_reload_downloadables():
                    self._send_templates()
                    has_tasks = True  # trigger check for new task

            if not self.robot.fms_manager.is_sync_ready():
                return True

        if not has_tasks:
            return
        has_tasks = False

        self.__new_task_evt.clear()
        while self.running and not rospy.is_shutdown():
            if self.paused:
                self.__new_task_evt.set()  # new task still available
                break
            if not self._execute_one_task():
                break
            has_tasks = True
        return has_tasks

    def _execute_one_task(self):
        with db_auto_reconnect():
            with transaction.atomic(), self._lock:
                f = {'status': Task.Status.Pending.value}
                if self._is_batt_below_min():
                    f['id'] = self.__approved_task_id
                self._task = Task.objects.running().select_for_update().defer('owner') \
                    .filter(**f).first()
                if not self._task:
                    return
                if self._task.id == self.__abortable_task_id:
                    if Task.objects.running().filter(**f).count() > 1:
                        self._task.cancel()
                        self._task = None
                        self._send_updates()
                        return True
                self.__last_task_id = self._task.id
                self._task.status = Task.Status.In_progress.value
                self._task.save()

            self.robot.registry.init_local()
            self._send_updates()
            self._fms_send_update(self._task)

            self.__sm_monitor_timer = rospy.Timer(rospy.Duration(1.0), self._sm_monitor)

        rospy.loginfo('[Task Runner] Executing task "%s"...', self._task)
        if self._build_and_run_sm():
            with self._lock:
                if self._sm_outcome in ['Completed', 'Aborted']:
                    self._task.status = Task.Status[self._sm_outcome].value
                else:
                    self._task.status = Task.Status.Aborted.value

            rospy.loginfo('[Task Runner] Outcome of task "%s": %s', self._task, self._task.get_status_display())
            self._send_outcome(self._task, self._task.get_status_display())
        else:
            with self._lock:
                self._task.status = Task.Status.Aborted.value

        self.robot.registry.init_local()
        self._fms_send_update(self._task)
        if self.robot.fms_manager:
            self.robot.fms_manager.set_navigation_approval_cb(None)

        if self.__sm_monitor_timer:
            self.__sm_monitor_timer.shutdown()
            self.__sm_monitor_timer = None
        self._send_action()

        with db_auto_reconnect():
            self.robot.models.incr_task_counter()
            self._task.save()
            with self._lock:
                if self.robot.fms_manager and self._task.fms_task_id:
                    self.__fms_agv_recent_tasks[self._task.fms_task_id] = self.__fms_serialize_task(self._task)
                    self.__save_fms_unsynced_tasks()
                self._task = None
                self._sm = None
            self._sm_outcome = ''
            self._send_updates()
        return True

    def _build_and_run_sm(self):
        self._sm_action = ''
        self._sm_call_stack = []
        self._send_action()

        # Setup the state machine
        try:
            with self._lock, self.robot.models.read_lock:
                self._sm = state_machine_factory(self.robot, self._task.task_template_id, json.loads(self._task.params), True)
                self._sm.register_action_cb(self._sm_on_action_callback)
        except StateConstructError as ex:
            rospy.logerr('[Task Runner] Task "%s": %s' % (self._task, ex))
            self._send_error(self._task, force_text(ex))
            return
        except Exception as ex:
            rospy.logerr('[Task Runner] Task "%s", Uncaught exception: %s' % (self._task, ex))
            self._send_error(self._task, self.unhandled_error_msg)
            return

        success = None
        try:
            rospy.loginfo('Executing...')
            self._sm_outcome = self._sm.execute()
            rospy.loginfo('Outcome: %s' % self._sm_outcome)
            success = True
        except UserError as ex:
            self._send_error(self._task, force_text(ex))
            rospy.logerr('Error: %s', ex)
            if self.robot.config.log_task_failure:
                self.robot.panel.log(logging.ERROR, force_text(ex))
        except Exception as ex:
            self._send_error(self._task, self.unhandled_error_msg)
            rospy.logerr('Exception: %s', ex)
            rospy.logerr(traceback.format_exc())

        self._sm_action = ''
        self._sm_call_stack = []
        return success

    def _sm_on_action_callback(self, action, call_stack):
        self._sm_action = force_text(action)
        self._sm_call_stack = call_stack

    def _sm_monitor(self, timer_event):
        self._send_action()
        if self._task.id == self.__abortable_task_id:
            self._abortable_monitor()

        if self.robot.fms_manager:
            if self.robot.fms_manager.pending_apply and self.robot.fms_manager.reloadable:
                if self.robot.fms_manager.hot_reload_downloadables():
                    self._send_templates()

    @db_auto_reconnect()
    def _abortable_monitor(self):
        if self._is_batt_below_min():
            return
        if Task.objects.running().filter(status=Task.Status.Pending.value).exists():
            self._abort_abortable_task()

    def _forward_dock(self):
        if 'forward' not in self.robot.models.allowed_motions:
            self._send_error('Initializing...', '"Forward" motion is not allowed.')
            self.init_status = 'unaware'
            self._send_init_status()
            return
        self.robot.base.forward({
            'speed': 0.2,
            'next_motion': 0,
            'enable_sensor': False,
        })
        self.robot.base.wait_for_result()
        self._dock()

    def _reverse_dock(self):
        if 'reverse' not in self.robot.models.allowed_motions:
            self._send_error('Initializing...', '"Reverse" motion is not allowed.')
            self.init_status = 'unaware'
            self._send_init_status()
            return
        self.robot.base.reverse({
            'speed': 0.2,
            'next_motion': 0,
            'enable_sensor': False,
        })
        self.robot.base.wait_for_result()
        self._dock()

    def _dock(self):
        if not self.running:
            return
        try:
            self.robot.power.enable_charging(True)
        except Exception:
            pass
        self._set_location_to_home()
        self._finalize_init()

    def _previous_location(self):
        try:
            location = json.loads(Variable.objects.get(name='/agv05_executor/previous_location').value)
            self.robot.base.set_initial_map_and_location(self.robot.models.graph, location)
            self._finalize_init()
        except Exception:
            rospy.logerr('[Task Runner] Lost track of previous location.')
            self._send_error('Initializing...', 'Lost track of previous location.')
            self.init_status = 'unaware'
            self._send_init_status()

    def _reset_position(self, station):
        if station == -1:
            self._previous_location()
            return

        location = self.robot.models.get_location_from_station(station)
        if not location or not self.robot.models.is_valid_heading(location[1]):
            rospy.logerr('[Task Runner] Cannot start from a headingless station.')
            self._send_error('Initializing...', 'Cannot start from a headingless station.')
            self.init_status = 'unaware'
            self._send_init_status()
            return
        self.robot.base.set_initial_map_and_location(self.robot.models.graph, location)
        self._finalize_init()

    def _custom_init(self, task_template_id, params):
        self.robot.base.map_tracker.set_confused()
        try:
            with self.robot.models.read_lock:
                task_templates_assoc = self.robot.models.task_templates_assoc  # task templates may hot reload
                self._custom_init_sm = state_machine_factory(self.robot, task_template_id, params, True)
            self._custom_init_sm.execute()
            self._custom_init_sm = None
        except (StateConstructError, UserError) as ex:
            self._custom_init_sm = None
            self._send_error('Initializing...', force_text(ex))
            rospy.logerr('[Task Runner] Custom initialization "%s", Error: %s' % (
                task_templates_assoc[task_template_id]['name'], ex))
            self.init_status = 'unaware'
            self._send_init_status()
            return
        except Exception as ex:
            self._custom_init_sm = None
            self._send_error('Initializing...', self.unhandled_error_msg)
            rospy.logerr('[Task Runner] Custom initialization "%s", Exception: %s' % (
                task_templates_assoc[task_template_id]['name'], ex))
            rospy.logerr(traceback.format_exc())
            self.init_status = 'unaware'
            self._send_init_status()
            return

        if not self.robot.base.is_location_aware():
            self._send_error('Initializing...', 'The custom initialization template does not set a starting position for AGV.')
            self.init_status = 'unaware'
            self._send_init_status()
            return

        self._finalize_init()

    def _pre_init(self, task_template_id, params):
        self.robot.base.map_tracker.set_confused()
        try:
            with self.robot.models.read_lock:
                task_templates_assoc = self.robot.models.task_templates_assoc  # task templates may hot reload
                self._custom_init_sm = state_machine_factory(self.robot, task_template_id, params, True)
            self._custom_init_sm.execute()
            self._custom_init_sm = None
        except (StateConstructError, UserError) as ex:
            self._custom_init_sm = None
            self._send_error('Pre Initializing...', force_text(ex))
            rospy.logerr('[Task Runner] Pre initialization "%s", Error: %s' % (
                task_templates_assoc[task_template_id]['name'], ex))
        except Exception as ex:
            self._custom_init_sm = None
            self._send_error('Pre Initializing...', self.unhandled_error_msg)
            rospy.logerr('[Task Runner] Pre initialization "%s", Exception: %s' % (
                task_templates_assoc[task_template_id]['name'], ex))
            rospy.logerr(traceback.format_exc())

        self.init_status = 'unaware'
        self._send_init_status()

    def _finalize_init(self):
        if self.robot.fms_manager:
            # send occupation request and wait for approval
            self.manager.set_agv_update_hook(lambda agv: agv.update({'status': 'Suspended'}) or agv)
            self.robot.fms_manager.send_occupation_request_async()
            timeout = rospy.get_rostime() + rospy.Duration(5.0)
            approved = None
            while self.running and not rospy.is_shutdown() and rospy.get_rostime() < timeout:
                rospy.sleep(0.1)
                approved = self.robot.fms_manager.check_occupation_approval()
                if isinstance(approved, bool):
                    break
            if not approved:
                self.manager.set_agv_update_hook(None)
                self.robot.fms_manager.cancel_occupation_request()
                self._send_error('Initializing...', 'Failed to obtain traffic reservation for AGV at the current position.')
                self.init_status = 'unaware'
                self._send_init_status()
                return

        self.__task_runner_initialized_pub.publish(std_msgs.msg.Bool(data=True))
        self.init_status = 'initialized'
        self._send_init_status()
        set_pause_check(self.__is_paused)
        self.robot.panel.toggle_led_paused(self.paused)
        self.robot.models.update_task_progress = self._update_task_progress
        self.manager.set_agv_update_hook(self.hook_agv_update)
        if self.robot.fms_manager:
            self.robot.fms_manager.set_task_assignment_cb(self.handle_task_assignment)
            self.robot.fms_manager.set_task_abort_cb(self.handle_task_abort)
            self.robot.fms_manager.set_task_runner_command_cb(self.handle_in_pipe)
            self.__fms_polling_last_modified = None
            self.__fms_task_list = []
            self.__fms_polling_last_modified_transaction = None
            self.__fms_transaction_lists = []
            self._fms_poll_task_list()
            self._fms_poll_transaction_list()
            self.__fms_polling_thread = threading.Thread(target=self.run_fms_polling)
            self.__fms_polling_thread.start()

    BUILTIN_INIT_OPTIONS = ({
        'label': 'AGV is parked at its home.',
        'type': 'home',
    }, {
        'label': 'Forward to charger (tracked).',
        'type': 'forward_dock',
    }, {
        'label': 'Reverse to charger (tracked).',
        'type': 'reverse_dock',
    })

    def _collect_custom_init(self):
        tta = self.robot.models.task_templates_assoc
        return [self.BUILTIN_INIT_OPTIONS[~ci] if ci < 0 else {
            'label': tta[ci]['name'],
            'type': 'custom_init_%d' % ci,
            'params': tta[ci]['params'],
        } for ci in self.robot.models.custom_init or []]

    def _collect_station_init(self):
        if self.robot.models.station_init == 1:
            station_init = [s for s in self._collect_stations()
                if self.robot.models.is_valid_heading(self.robot.models.get_location_from_station(s)[1])]
            try:
                location = json.loads(Variable.objects.get(name='/agv05_executor/previous_location').value)
                return station_init + [-1]
            except Exception:
                if station_init and len(station_init) < len(self._collect_stations()):
                    return station_init
        return self.robot.models.station_init or 0

    def _collect_pre_init(self):
        tta = self.robot.models.task_templates_assoc
        return [{
            'label': tta[pi]['name'],
            'type': 'pre_init_%d' % pi,
            'params': tta[pi]['params'],
        } for pi in self.robot.models.pre_init or []]

    def _collect_default_init(self):
        di = self.robot.models.default_init
        if not di:
            return None

        tt = di['task_template']
        if tt < 0:
            t = self.BUILTIN_INIT_OPTIONS[~tt]['type']
        else:
            t = 'custom_init_%d' % tt
        return {'type': t, 'countdown': di['timeout']}

    def _collect_task_templates(self):
        with self.robot.models.read_lock:
            return [{
                'id': tt['id'],
                'name': tt['name'],
                'params': tt['params'],
                'allowed_groups': tt['allowed_groups'],
                'allowed_users': tt['allowed_users'],
            } for tt in (self.robot.models.task_templates or []) if tt['is_top_level']]

    def _collect_stations(self):
        return self.robot.models.station_names or []

    def _collect_registers(self):
        return self.robot.models.register_list

    def _collect_variables(self):
        return self.robot.models.variables

    def _collect_tasks(self):
        with self.robot.models.read_lock:
            tasks = list(Task.objects.running().values('id', 'name', 'task_template_id',
                'task_template', 'params', 'fms_task_id', 'status', 'progress', 'owner__username'))

            fms_task_ids = set()
            agv_uuid = self.robot.models.agv_uuid
            agv_name = self.robot.models.agv_name

            for t in tasks:
                try:
                    desc_params = self.robot.models.task_templates_assoc[t['task_template_id']]['params']
                    values = json.loads(t['params'])
                    t['params'] = [{
                        'name': param['name'],
                        'value': values.get(param['name'])
                    } for param in desc_params]
                except Exception:
                    t['params'] = []

                if self.robot.fms_manager:
                    if t['fms_task_id']:
                        fms_task_ids.add(t['fms_task_id'])
                    t['agv_id'] = True
                    t['agv'] = agv_name

                t['status'] = Task.Status(t['status']).name
                t['owner'] = t['owner__username']
                del t['owner__username']

            if not self.robot.fms_manager:
                return tasks

            fms_task_list = self.__fms_task_list
            for fms_task in fms_task_list:
                if (fms_task['id'] in fms_task_ids or
                        fms_task['id'] in self.__fms_agv_recent_tasks or
                        fms_task['id'] in self.__fms_agv_unsynced_tasks):
                    continue

                t = fms_task.copy()
                try:
                    desc_params = self.robot.models.task_templates_assoc[t['task_template_id']]['params']
                    values = json.loads(t['params'])
                    t['params'] = [{
                        'name': param['name'],
                        'value': values.get(param['name'])
                    } for param in desc_params]
                except Exception:
                    t['params'] = []

                t['fms_task_id'] = t['id']
                t['id'] = None
                if t['agv_id'] == agv_uuid:
                    t['agv_id'] = True

                t['status'] = Task.Status(t['status']).name
                t['owner'] = None
                tasks.append(t)
            return tasks

    def _update_task_progress(self, progress):
        if self._task:
            with db_auto_reconnect():
                self._task.progress = progress
                self._task.save()
            self._send_updates()
            self._fms_send_update(self._task)

    def _remove_all_tasks(self, tasks, stale_only=False):
        updated = False

        for t in tasks:
            if t.status >= Task.Status.In_progress.value:
                t.status = Task.Status.Aborted.value
                t.save()
            elif t.status == Task.Status.Cancelling.value:
                t.status = Task.Status.Cancelled.value
                t.save()
            elif t.status < Task.Status.Cancelling.value:
                if stale_only:
                    self.__new_task_evt.set()
                    continue
                else:
                    t.status = Task.Status.Cancelled.value
                    t.save()

            if self.robot.fms_manager and t.fms_task_id:
                self.__fms_agv_recent_tasks[t.fms_task_id] = self.__fms_serialize_task(t)
                updated = True

        if updated:
            self.__save_fms_unsynced_tasks()

    def _set_location_to_home(self):
        self.robot.base.set_initial_map_and_location(
            self.robot.models.graph,
            self.robot.models.get_agv_home_location())

    def _is_agv_at_home(self):
        return self._is_agv_at_station(self.robot.models.agv_home)

    def _is_agv_at_station(self, station):
        j, h = self.robot.models.get_location_from_station(station)
        agv_location = self.robot.base.get_location()
        if not isinstance(agv_location, tuple):
            return False
        if agv_location[0] != j:
            return False
        if not self.robot.models.check_transition(agv_location[1], h, zero=True):
            return False
        return True

    def _is_batt_below_min(self):
        below_min = self.robot.power.battery_percentage < self.robot.models.min_battery_level
        if self.__batt_below_min != below_min:
            self.__batt_below_min = below_min
            self._send_pause_status()
        return below_min

    def _send_templates(self):
        self.out({
            'command': 'list_templates',
            'custom_init': self._collect_custom_init(),
            'station_init': self._collect_station_init(),
            'pre_init': self._collect_pre_init(),
            'task_templates': self._collect_task_templates(),
            'stations': self._collect_stations(),
            'registers': self._collect_registers(),
            'variables': self._collect_variables(),
        })

    def _send_init_status(self):
        self.out({
            'command': 'init_status',
            'status': self.init_status,
        })

    def _send_default_init(self):
        self.out({
            'command': 'default_init',
            'default_init': self.default_init or None,
        })

    def _send_pause_status(self):
        self.out({
            'command': 'is_paused',
            'paused': self.paused,
            'resuming': self.resuming,
            'battery_low': self.__batt_below_min,
        })

    def _send_updates(self):
        self.out({
            'command': 'list_tasks',
            'tasks': self._collect_tasks(),
        })

    def _send_action(self):
        self.out({
            'command': 'action',
            'action': self._sm_action,
            'call_stack': self._sm_call_stack,
        })

    def _send_outcome(self, task, outcome):
        self.out({
            'command': 'outcome',
            'task': force_text(task),
            'outcome': outcome,
        })

    def _send_error(self, task, error):
        self.out({
            'command': 'error',
            'task': force_text(task),
            'error': error,
        })

    def _send_transaction_updates(self):
        self.out({
            'command': 'list_transactions',
            'transactions': self.__fms_transaction_lists,
            'executing_transaction': self.__executing_transaction,
        })

    def _send_manual_control(self):
        self.out({
            'command': 'manual_control',
            'is_omni_drive': bool(self.is_omni_drive),
        })

    def _send_human_follower(self, *args):
        self.out({
            'command': 'human_follower',
            'ready': self.robot.base.has_human_follower(),
            'running': self.robot.base.human_follower_running,
        })

    def _resend_fms_status(self):
        if self.robot.fms_manager:
            self.robot.fms_manager.resend_status()

    def _fms_send_update(self, task):
        if not self.robot.fms_manager:
            return
        if isinstance(task, Task):
            task = self.__fms_serialize_task(task)
        if task['id']:
            self.robot.fms_manager.send_task_update(task)

    def __fms_serialize_task(self, task):
        return {
            'id': task.fms_task_id,
            'status': Task.Status(task.status).name,
            'progress': task.progress,
        }

    def __load_fms_unsynced_tasks(self):
        self.__fms_agv_recent_tasks = {}
        try:
            unsynced = json.loads(Variable.objects.get(name='/agv05_executor/fms_agv_unsynced_tasks').value)
            assert (isinstance(unsynced, dict))
        except Exception:
            unsynced = {}
        self.__fms_agv_unsynced_tasks = unsynced

    def __save_fms_unsynced_tasks(self):
        unsynced = dict(self.__fms_agv_unsynced_tasks)
        unsynced.update(self.__fms_agv_recent_tasks)
        Variable.objects.update_or_create(name='/agv05_executor/fms_agv_unsynced_tasks', defaults={
            'value': json.dumps(unsynced),
        })
