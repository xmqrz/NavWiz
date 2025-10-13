from __future__ import absolute_import
from __future__ import unicode_literals

from six.moves.queue import Queue as ConcurrencyQueue, Empty
from agv05_webserver.system.models import Cache, db_auto_reconnect, ErrorCode, Variable
from django.db import transaction
from kombu import Exchange, Queue
from kombu.asynchronous import get_event_loop
import collections
import hashlib
import requests
import rospy
import std_msgs.msg
import threading
import time
import ujson as json

from .fms_comm import get_connection, get_producer


class FmsManager(object):
    TIMER_INTERVAL = 1
    DISCONNECT_INTERVAL = 6

    __agv_update_topic = 'agv.agv_update'
    __exec_request_topic = 'agv.execution_request'
    __nav_request_topic = 'agv.navigation_request'
    __task_update_topic = 'agv.task_update'
    __custom_log_topic = 'agv.custom_log'
    __agv_feedback_topic = 'agv.%(uuid).23s-%(token).12s.agv_feedback'
    __exec_feedback_topic = 'agv.%(uuid).23s-%(token).12s.execution_feedback'
    __nav_approval_topic = 'agv.%(uuid).23s-%(token).12s.navigation_approval'
    __robot_control_topic = 'agv.%(uuid).23s-%(token).12s.robot_control'
    __task_abort_topic = 'agv.%(uuid).23s-%(token).12s.task_abort'
    __task_assignment_topic = 'agv.%(uuid).23s-%(token).12s.task_assignment'
    __task_runner_command_topic = 'agv.%(uuid).23s-%(token).12s.task_runner_command'
    __agv_live_app_topic = 'agv.%(uuid).23s-%(token).12s.live_app'
    __agv_live_app_command_topic = 'agv.%(uuid).23s-%(token).12s.live_app_command'

    def __init__(self, robot):
        self.robot = robot
        self.models = robot.models
        self.models.agv_ip = None

        self.signature = {
            'uuid': self.models.agv_uuid,
            'token': self.models.fms_metadata['token'],
        }

        self.agv_feedback_cb = None
        self.execution_feedback_cb = None
        self.navigation_approval_cb = None
        self.robot_control_cb = None
        self.task_abort_cb = None
        self.task_assignment_cb = None
        self.task_runner_command_cb = None

        self.live_app_command_cb = None

        self._lock = threading.Lock()
        self._worker_job_queue = ConcurrencyQueue()
        self._worker_thread = None
        self._custom_log_queue = collections.deque(maxlen=10)
        self._throttle_custom_log = False
        self._status = 'Disconnected'
        self._status_code = ErrorCode.DFLEET_DISCONNECTED
        self._status_broken = True
        self.first_synced = False
        self.pending_apply = False
        self.reloadable = False

        # Apply previous download if the AGV skillset remains the same.
        self.models.from_downloadables()

        # ROS topics
        self.__in_pipe = rospy.Subscriber('~fms_in', std_msgs.msg.String, self.handle_in_pipe)
        self.__out_pipe = rospy.Publisher('~fms_out', std_msgs.msg.String, queue_size=10)

    def setup(self):
        self._send_status('Disconnected', broken=True, code=ErrorCode.DFLEET_DISCONNECTED)

        with get_connection() as connection:
            exchange = Exchange('agv.direct', 'direct')

            topic = self.__agv_feedback_topic % self.signature
            self.__agv_feedback_sub = connection.AutoConsumer(
                Queue(topic, exchange, topic, durable=False, auto_delete=True),
                no_ack=True, callbacks=[self.on_agv_feedback])

            topic = self.__exec_feedback_topic % self.signature
            self.__exec_feedback_sub = connection.AutoConsumer(
                Queue(topic, exchange, topic, durable=False, auto_delete=True),
                no_ack=True, callbacks=[self.on_execution_feedback])

            topic = self.__nav_approval_topic % self.signature
            self.__nav_approval_sub = connection.AutoConsumer(
                Queue(topic, exchange, topic, durable=False, auto_delete=True),
                no_ack=True, callbacks=[self.on_navigation_approval])

            topic = self.__robot_control_topic % self.signature
            self.__robot_control_sub = connection.AutoConsumer(
                Queue(topic, exchange, topic, durable=False, auto_delete=True),
                no_ack=True, callbacks=[self.on_robot_control])

            topic = self.__task_abort_topic % self.signature
            self.__task_abort_sub = connection.AutoConsumer(
                Queue(topic, exchange, topic, durable=False, auto_delete=True),
                no_ack=True, callbacks=[self.on_task_abort])

            topic = self.__task_assignment_topic % self.signature
            self.__task_assignment_sub = connection.AutoConsumer(
                Queue(topic, exchange, topic, durable=False, auto_delete=True),
                no_ack=True, callbacks=[self.on_task_assignment])

            topic = self.__task_runner_command_topic % self.signature
            self.__task_runner_command_sub = connection.AutoConsumer(
                Queue(topic, exchange, topic, durable=False, auto_delete=True),
                no_ack=True, callbacks=[self.on_task_runner_command])

            self.__reset_trigger = connection.recoverable_connection_errors[0]

        self.agv_update_timer = get_event_loop().call_repeatedly(self.TIMER_INTERVAL, self.on_get_agv_update)

        self.__last_feedback = time.time()
        self.__last_live_sync = time.time()
        self.__reconnecting = False

        self.__dfleet_run_id = None
        self.__occupation = None
        self.__occupation_approval = None

        self._worker_thread = threading.Thread(target=self._run)
        self._worker_thread.start()

    def send_agv_update(self, msg):
        return self._publish(msg, self.__agv_update_topic)

    def send_execution_request(self, msg):
        return self._publish(msg, self.__exec_request_topic)

    def _send_navigation_request(self, msg):
        return self._publish(msg, self.__nav_request_topic)

    def send_navigation_request_async(self, msg):
        self.__occupation = None
        self._worker_job_queue.put(('send_navigation_request', msg))

    def send_occupation_request_async(self):
        base = self.robot.base
        dp = base.get_dimension_profile()
        location = base.get_location()
        if not location:
            return
        self.__occupation = (dp, location)

    def cancel_occupation_request(self):
        self.__occupation = None
        self.__occupation_approval = None

    def has_occupation_request(self):
        return self.__occupation is not None

    def check_occupation_approval(self):
        approval = self.__occupation_approval
        if approval:
            if approval == (False, self.__occupation):
                return False
            if approval == (self.__dfleet_run_id, self.__occupation):
                return True

    def send_task_update(self, msg):
        return self._publish(msg, self.__task_update_topic)

    def _send_custom_log(self, msg):
        return self._publish(msg, self.__custom_log_topic)

    def send_custom_log_async(self, msg):
        self._custom_log_queue.append(msg)

    def _publish(self, msg, topic):
        msg.update(self.signature)
        try:
            with get_producer() as producer:
                producer.publish(msg, topic, exchange='agv.direct')
        except Exception:
            pass
        else:
            return True

    def set_agv_feedback_cb(self, cb):
        self.agv_feedback_cb = cb

    def set_execution_feedback_cb(self, cb):
        self.execution_feedback_cb = cb

    def set_navigation_approval_cb(self, cb):
        self.navigation_approval_cb = cb

    def set_robot_control_cb(self, cb):
        self.robot_control_cb = cb

    def set_task_abort_cb(self, cb):
        self.task_abort_cb = cb

    def set_task_assignment_cb(self, cb):
        self.task_assignment_cb = cb

    def set_task_runner_command_cb(self, cb):
        self.task_runner_command_cb = cb

    def on_get_agv_update(self):
        agv = dict(self.models.agv)
        agv.update({
            'agv_ip': self.models.agv_ip,
            'skillset_md5': self.models.skillset_md5,
            'downloadables_md5': self.models.downloadables_md5,
        })

        self.send_agv_update(agv)

        if time.time() - self.__last_feedback > self.DISCONNECT_INTERVAL:
            self.__last_feedback = time.time()
            self._send_status('Disconnected', broken=True, code=ErrorCode.DFLEET_DISCONNECTED)
            # trigger sync to determine other issues.
            self._dispatch_live_sync(['agv_ip'])
            # trigger reconnection if required.
            if get_event_loop().readers:
                if self.__reconnecting:
                    self.__reconnecting = False
                else:
                    self.__reconnecting = True
                    raise self.__reset_trigger('Heartbeat timeout.')

        elif time.time() - self.__last_live_sync > 15 * 60:
            # trigger sync to determine agv_ip change.
            self._dispatch_live_sync(['agv_ip'])

    def on_agv_feedback(self, body, message):
        self.__last_feedback = time.time()
        self.__reconnecting = False
        self.__dfleet_run_id = body.get('run_id')

        if not body.get('compatible'):
            self._send_status('Incompatible', broken=True, code=ErrorCode.DFLEET_INCOMPATIBLE)
        elif body.get('downloads'):
            self._send_status('Syncing', broken=True, code=ErrorCode.DFLEET_SYNCING)
            self._dispatch_live_sync(body['downloads'])
        elif self.pending_apply and not self.reloadable:
            error = self.models.downloadables.get('error')
            if error:
                status = 'Failed to apply data: %s' % error
            else:
                status = 'New data will be applied once the current app is stopped.'
            self._send_status('Sync pending:\n%s' % status, broken=bool(error), code=ErrorCode.DFLEET_SYNC_PENDING)
        else:
            self._send_status('Ready', broken=False, code=ErrorCode.NORMAL)

        if self.agv_feedback_cb:
            try:
                self.agv_feedback_cb(body)
            except Exception as ex:
                rospy.logerr('[Fms Comm]: Unhandled exception in agv_feedback callback function: %s', ex)

    def on_execution_feedback(self, body, message):
        if self.execution_feedback_cb:
            try:
                self.execution_feedback_cb(body)
            except Exception as ex:
                rospy.logerr('[Fms Comm]: Unhandled exception in execution_feedback callback function: %s', ex)

    def on_navigation_approval(self, body, message):
        if self.navigation_approval_cb:
            try:
                self.navigation_approval_cb(body)
            except Exception as ex:
                rospy.logerr('[Fms Comm]: Unhandled exception in navigation_approval callback function: %s', ex)

        try:
            if body['start'] == body['end']:
                self.__occupation_approval = (
                    body['approved'] and body['plan'] == body['start'][:1] and
                    body['plan_end'] == body['start'] and body['run_id'] or False,
                    (body['dp'], tuple(body['start'])))
            else:
                self.__occupation_approval = None
        except Exception as ex:
            rospy.logerr('[Fms Comm]: Unhandled exception in occupation_approval callback function: %s', ex)

    def on_robot_control(self, body, message):
        if self.robot_control_cb:
            try:
                self.robot_control_cb(body)
            except Exception as ex:
                rospy.logerr('[Fms Comm]: Unhandled exception in robot_control callback function: %s', ex)

    def on_task_abort(self, body, message):
        if self.task_abort_cb:
            try:
                self.task_abort_cb(body)
            except Exception as ex:
                rospy.logerr('[Fms Comm]: Unhandled exception in task_abort callback function: %s', ex)

    def on_task_assignment(self, body, message):
        if self.task_assignment_cb:
            try:
                self.task_assignment_cb(body)
            except Exception as ex:
                rospy.logerr('[Fms Comm]: Unhandled exception in task_assignment callback function: %s', ex)

    def on_task_runner_command(self, body, message):
        if self.task_runner_command_cb:
            try:
                self.task_runner_command_cb(body)
            except Exception as ex:
                rospy.logerr('[Fms Comm]: Unhandled exception in task_runner_command callback function: %s', ex)

    def _run(self):
        while not rospy.is_shutdown():
            live_sync_job = None
            send_navigation_request_job = None

            # consume job greedily, and perform only the last job
            while True:
                try:
                    job = self._worker_job_queue.get_nowait()
                except Empty:
                    break
                else:
                    if job[0] == 'live_sync':
                        live_sync_job = job
                    elif job[0] == 'send_navigation_request':
                        send_navigation_request_job = job

            work_done = False

            if live_sync_job:
                _, request_time, downloads = live_sync_job
                if self.__last_live_sync < request_time:
                    self._live_sync(downloads)
                    self.__last_live_sync = time.time()
                    work_done = True

            if self.__occupation:
                if (self.is_sync_ready() and self.models.agv['status'] in [
                        'Idle', 'Working', 'Running App', 'Paused', 'Suspended'] and
                        (self.__dfleet_run_id, self.__occupation) != self.__occupation_approval):
                    self._send_navigation_request({
                        'timestamp': time.time(),
                        'ack_only': False,
                        'dp': self.__occupation[0],
                        'start': self.__occupation[1],
                    })
            elif send_navigation_request_job:
                _, msg = send_navigation_request_job
                self._send_navigation_request(msg)
                work_done = True

            if not self._throttle_custom_log:
                n = 5
                while self._custom_log_queue and n:
                    msg = self._custom_log_queue[0]
                    if self._send_custom_log(msg):
                        self._custom_log_queue.popleft()
                        n -= 1
                    else:
                        # send error, disable send for this round
                        n = 0
                    work_done = True
                if not n:
                    self._throttle_custom_log = True

            if not work_done:
                time.sleep(1.0)  # use wall-time because the simulation clock could be not running
                if self._throttle_custom_log:
                    self._throttle_custom_log = False

    def _dispatch_live_sync(self, downloads):
        self._worker_job_queue.put(('live_sync', time.time(), downloads))

    def _live_sync(self, downloads):
        self.__last_live_sync = time.time()

        headers = {
            'Authorization': 'AgvToken %s:%s' % (self.models.agv_uuid, self.models.fms_metadata['token']),
        }
        try:
            params = dict(downloads=','.join(downloads))
        except Exception:
            params = {}

        try:
            with requests.get(self.models.fms_metadata['live_endpoint'], headers=headers,
                    params=params, timeout=(3.05, 7), allow_redirects=False, verify=False, stream=True) as r:
                content = bytes()
                gen = r.iter_content(1024)
                while not rospy.is_shutdown():
                    try:
                        content += next(gen)
                    except StopIteration:
                        break

                if rospy.is_shutdown():
                    return

                content = content.decode('utf-8')

                if r.status_code == 200:
                    try:
                        data = json.loads(content)['data']
                        assert isinstance(data, dict)
                    except Exception:
                        raise RuntimeError('Data format error')
                elif r.is_redirect:
                    # detect http to https redirection
                    raise RuntimeError('Server endpoint has changed.')
                else:
                    try:
                        detail = json.loads(content)['detail']
                    except Exception:
                        detail = r.reason
                    raise RuntimeError('Got (%s) %s' % (r.status_code, detail))

        except requests.exceptions.ConnectionError:
            self._send_status('Sync Error:\nCannot connect to server.', broken=True)
            return

        except Exception as ex:
            self._send_status('Sync Error:\n%s' % ex, broken=True)
            return

        with self._lock:
            if not self._update_downloadables(data):
                return

        # obtain lock to prevent module starting or stopping
        with self.robot.lock:
            if not self.robot.is_reserved():
                self.apply_downloadables()

    def _update_downloadables(self, data):
        downloadables_keys = ['skillset', 'fms_skillset',
            'map_params', 'map_structure', 'station_names',
            'global_params', 'variables', 'task_templates',
            'teleports', 'transition_triggers',
            'agv_home', 'executor_cfg', 'transaction_enabled']
        if self.robot.trackless:
            downloadables_keys += ['ocg']

        for k, v in data.items():
            if k == 'agv_ip':
                self.models.agv_ip = v
            elif k == 'agv_home' or k == 'skillset':
                self.models.downloadables[k] = v
                self.models.downloadables_md5[k] = v
            elif k == 'fms_skillset':
                self.models.downloadables[k] = json.loads(v)
                self.models.downloadables_md5[k] = hashlib.md5(v.encode('utf-8')).hexdigest()
            elif k == 'transition_triggers':
                processed_v = {int(key): value for key, value in v.items()}
                self.models.downloadables[k] = processed_v
                self.models.downloadables_md5[k] = hashlib.md5(json.dumps(processed_v, sort_keys=True).encode('utf-8')).hexdigest()
            elif k in downloadables_keys:
                self.models.downloadables[k] = v
                self.models.downloadables_md5[k] = hashlib.md5(json.dumps(v, sort_keys=True).encode('utf-8')).hexdigest()

        # Remove obsolete downloadables key
        keys = list(self.models.downloadables_md5.keys())
        for k in keys:
            if k not in downloadables_keys:
                self.models.downloadables_md5.pop(k)
                try:
                    data[k] = self.models.downloadables.pop(k)
                except Exception:
                    data[k] = True

        if self.first_synced or self.models.downloadables.get('error'):
            if list(data.keys()) == ['agv_ip']:
                # skip if only refreshing agv_ip
                return

        if set(self.models.downloadables.keys()).issuperset(downloadables_keys):
            self.models.downloadables['error'] = None
            self.reloadable = self.models.check_reloadable(self.models.downloadables, enabled=False, pre_validate=False)
            self.pending_apply = True
            rospy.loginfo('Downloadables reloadable is %s' % self.reloadable)
            with db_auto_reconnect(), transaction.atomic():
                Cache.set_downloadables(self.models.downloadables)
                Cache.set_downloadables_md5(self.models.downloadables_md5)
            return True

    @db_auto_reconnect()
    def apply_downloadables(self):
        with self._lock:
            if not self.pending_apply or self.models.downloadables.get('error'):
                return
            if self.models.from_downloadables():
                if not self.reloadable:
                    Variable.objects.filter(name='/agv05_executor/previous_location').delete()
                self.robot.base.map_tracker.set_map(self.robot.models.graph)  # clear location tracking
                self.pending_apply = False
                self.reloadable = False
                self.first_synced = True
                return True

    @db_auto_reconnect()
    def hot_reload_downloadables(self):
        with self._lock:
            if not self.pending_apply or not self.reloadable or self.models.downloadables.get('error'):
                return
            if self.models.hot_reload_downloadables():
                self.pending_apply = False
                self.reloadable = False
                self.first_synced = True
                return True

    def is_sync_ready(self):
        return self.first_synced and not self.pending_apply and not self._status_broken

    def is_sync_pending(self):
        return not self.first_synced or self.pending_apply

    def get_status(self):
        with self._lock:
            return self._status, self._status_broken, self._status_code

    def handle_in_pipe(self, msg):
        pass

    def out(self, data):
        self.__out_pipe.publish(std_msgs.msg.String(data=json.dumps(data)))

    def resend_status(self):
        with self._lock:
            if not self._status:
                return  # avoid exception due to __out_pipe not ready.
            self.out({
                'id': 'status',
                'value': self._status,
                'broken': self._status_broken,
            })

    def _send_status(self, status, broken, code=None):
        with self._lock:
            self._status = status
            self._status_broken = broken
            if code is not None:
                self._status_code = code
            self.out({
                'id': 'status',
                'value': status,
                'broken': broken,
            })

    def setup_live_app(self):
        with get_connection() as connection:
            exchange = Exchange('agv.direct', 'direct')

            topic = self.__agv_live_app_command_topic % self.signature
            self.__agv_feedback_sub = connection.AutoConsumer(
                Queue(topic, exchange, topic, durable=False, auto_delete=True),
                no_ack=True, callbacks=[self.on_live_app_cmd])

    def on_live_app_cmd(self, body, message):
        if self.live_app_command_cb:
            try:
                self.live_app_command_cb(body)
            except Exception as ex:
                rospy.logerr('[Fms Comm]: Unhandled exception in live app command callback function: %s', ex)

    def set_live_app_cmd_cb(self, cb):
        self.live_app_command_cb = cb

    def send_live_app(self, msg):
        return self._publish(msg, self.__agv_live_app_topic % self.signature)

    def get_map_ocg(self, ocg_pk):
        headers = {
            'Authorization': 'AgvToken %s:%s' % (self.models.agv_uuid, self.models.fms_metadata['token']),
        }
        url = '%sapi/v3/config/maps/ocgs/%s' % (self.models.fms_metadata['dashboard'], ocg_pk)
        try:
            with requests.get(url, headers=headers,
                    timeout=(3.05, 7), allow_redirects=False, verify=False, stream=True) as r:
                content = bytes()
                gen = r.iter_content(1024)
                while not rospy.is_shutdown():
                    try:
                        content += next(gen)
                    except StopIteration:
                        break

                if rospy.is_shutdown():
                    return

                content = content.decode()

                if r.status_code == 200:
                    try:
                        data = json.loads(content)
                        assert isinstance(data, dict)
                        return data
                    except Exception:
                        raise RuntimeError('Data format error')
                elif r.is_redirect:
                    # detect http to https redirection
                    raise RuntimeError('Server endpoint has changed.')
                else:
                    raise RuntimeError('Fail to obtain ocg.')
        except requests.exceptions.ConnectionError:
            rospy.logerr('[Fms Comm]: Get ocg error: Cannot connect to server.')
        except Exception as ex:
            rospy.logerr('[Fms Comm]: Get ocg error: %s' % ex)
