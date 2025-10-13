from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Activity, AgvActivity, db_auto_reconnect, redis_cache
from agv05_webserver.system.tasks.build_agv_statistics import build_agv_statistics
from django.core.cache import cache
from django.db import transaction
from django.utils import timezone
from django.utils.encoding import force_text
from six.moves import queue as Queue
import diagnostic_updater
import rospy
import six
import std_msgs.msg
import threading
import ujson as json


class Diagnostic(object):

    def __init__(self, robot):
        self.robot = robot
        self._agv = None
        self._activity = Activity.POWERED_OFF
        self._downtime = None

        self._ev = threading.Event()
        self._lock = threading.Lock()

        self.__in_pipe = rospy.Subscriber('~downtime_tracker_in', std_msgs.msg.String, self.handle_in_pipe, queue_size=1)
        self.__out_pipe = rospy.Publisher('~downtime_tracker_out', std_msgs.msg.String, latch=True, queue_size=1)

        self._updater = diagnostic_updater.Updater()
        self._updater.setHardwareID('AGV05')
        self._updater.add('Status', self._handle_diagnostic_status)

        self._queue = Queue.Queue()
        self._worker_thread = threading.Thread(target=self._logger)
        self._worker_thread.start()

        self.out({'tracker': None})

    def signal_shutdown(self):
        self._ev.set()

    def update(self, agv=None):
        if agv:
            self._agv = agv
            self._updater.force_update()
        else:
            agv = self._agv
            self._updater.update()

        if not agv:
            return

        # record agv activity
        status = agv['status']
        activity = Activity.POWERED_OFF
        if status == 'Powered Off':
            pass
        elif status == 'Not Ready':
            activity = Activity.NOT_READY
        elif status in ['Running App', 'Running App (Sync Pending)']:
            activity = Activity.RUNNING_APP
        elif status in ['Idle', 'Working', 'Suspended', 'Paused', 'Sync Pending', 'Syncing']:
            if agv['error_code']:
                activity = agv['error_code']
            elif agv['charging']:
                activity = Activity.CHARGING
            elif status == 'Working':
                activity = Activity.WORKING
            else:
                activity = Activity.IDLE

        now = timezone.localtime().replace(microsecond=0)
        self._queue.put((now, activity))

        if self._is_downtime(self._activity) != self._is_downtime(activity):
            # wake thread to send or hide tracker
            self._ev.set()
        self._activity = activity

    def _handle_diagnostic_status(self, stat):
        if not self._agv:
            return stat
        agv = self._agv

        location = agv['location']
        if location:
            try:
                location = '%s | %s | %s' % (
                    location[0] // 100000,
                    location[0] % 100000,
                    '%.1f' % location[1] if self.robot.trackless else
                    ['N', 'E', 'S', 'W'][location[1]])
            except Exception:
                pass
        else:
            location = '-'

        prev_location = agv['prev_location']
        if prev_location:
            try:
                prev_location = '%s | %s | %s' % (
                    prev_location[0] // 100000,
                    prev_location[0] % 100000,
                    '%.1f' % prev_location[1] if self.robot.trackless else
                    ['N', 'E', 'S', 'W'][prev_location[1]])
            except Exception:
                pass
        else:
            prev_location = '-'

        pose = agv['pose']
        if pose:
            try:
                pose = '%(x)s | %(y)s | %(theta)s' % pose
            except Exception:
                pass
        else:
            pose = '-'

        velocity = agv['velocity']
        if velocity:
            try:
                velocity = '%s | %s | %s' % tuple(velocity)
            except Exception:
                pass
        else:
            velocity = '-'

        safety_message = agv['safety_message']
        safety_message = six.ensure_str(safety_message['message'].replace('\n', '\u21b5')) if safety_message else '-'
        user_message = agv['user_message']
        user_message = six.ensure_str(force_text(user_message['message']).replace('\n', '\u21b5')) if user_message else '-'
        fms_status = agv['fms_status']
        fms_status = six.ensure_str(fms_status.replace('\n', '\u21b5')) if fms_status else '-'

        stat.summary(diagnostic_updater.DiagnosticStatus.OK, 'Status OK')
        stat.add('UUID', six.ensure_str(agv['uuid']) if agv['uuid'] else '-')
        stat.add('Name', six.ensure_str(agv['name']) if agv['name'] else '-')
        stat.add('Status', agv['status'])
        stat.add('Action', six.ensure_str(agv['action']) or '-')
        stat.add('Location (map | j | h)', location)
        stat.add('Previous Location (map | j | h)', prev_location)
        stat.add('Pose (x | y | th)', pose)
        stat.add('Motion', agv['motion'] or '-')
        stat.add('Velocity (x | y | th)', velocity)
        stat.add('Dimension Profile', agv['dimension_profile'])
        stat.add('Error Code', agv['error_code'])
        stat.add('Safety Message', safety_message)
        stat.add('User Message', user_message)
        stat.add('Charging', agv['charging'])
        stat.add('Battery (%)', agv['battery'])
        stat.add('Task Counter', agv['task_counter'])
        stat.add('Mileage (m)', agv['mileage'] or '-')
        stat.add('DFleet Status', fms_status)
        return stat

    def _logger(self):
        with db_auto_reconnect():
            p = AgvActivity.objects.last()

        initialized = False
        while not initialized and not rospy.is_shutdown():
            if self._ev.wait(10):
                self._ev.clear()
            while not self._queue.empty():
                t, a = self._queue.get()
                if p:
                    if t <= p.end:
                        continue
                    with db_auto_reconnect():
                        p = AgvActivity(start=p.end, end=t, activity=Activity.POWERED_OFF)
                        p.save()
                p = AgvActivity(start=t, end=t, activity=a)
                initialized = True
                break

        if initialized and not self._is_downtime(a):
            q = AgvActivity.objects.exclude(**self._downtime_kwargs()).last()
            if q:
                self._send_downtime(timezone.localtime(q.end), t)
        else:
            self._ev.set()

        while not rospy.is_shutdown():
            if self._ev.wait(30):
                self._ev.clear()
            with db_auto_reconnect():
                modified = False
                while not self._queue.empty():
                    t, a = self._queue.get()
                    if t <= p.end:
                        continue
                    p.end = t
                    modified = True
                    if a != p.activity:
                        p.save()
                        if not self._is_downtime(a) and self._is_downtime(p.activity):
                            q = AgvActivity.objects.exclude(**self._downtime_kwargs()).last()
                            if q:
                                self._send_downtime(timezone.localtime(q.end), t)

                        elif self._is_downtime(a):
                            with self._lock:
                                self._downtime = None
                            self.out({'tracker': None})

                        p = AgvActivity(start=t, end=t, activity=a)
                        modified = False
                if modified:
                    p.save()

    def _is_downtime(self, ac):
        self._tracker = self.robot.config.downtime_activity_tracker
        if self._tracker == 'robot_controller':
            return Activity.POWERED_OFF <= ac < 100
        elif self._tracker == 'task_runner':
            return Activity.RUNNING_APP <= ac < 100
        return True

    def _downtime_kwargs(self):
        if self._tracker == 'robot_controller':
            return {
                'activity__gte': Activity.POWERED_OFF,
                'activity__lt': 100,
            }
        elif self._tracker == 'task_runner':
            return {
                'activity__gte': Activity.RUNNING_APP,
                'activity__lt': 100,
            }
        return {}

    def _send_downtime(self, start, end):
        downtime = {
            'start': start,
            'end': end,
            'msg': {
                'tracker': self._tracker,
                'text': 'between %s and %s (%s)' % (
                    start.strftime('%Y-%m-%d %H:%M:%S'),
                    end.strftime('%Y-%m-%d %H:%M:%S'),
                    self._get_duration_display(end - start),
                ),
            },
        }
        with self._lock:
            self._downtime = downtime
        self.out(downtime['msg'])

    def _get_duration_display(self, d):
        h = d.seconds // 3600
        m = (d.seconds % 3600) // 60
        s = d.seconds % 60

        if d.days:
            return '%dd, %dh %dm %ds' % (d.days, h, m, s)
        elif h:
            return '%dh %dm %ds' % (h, m, s)
        elif m:
            return '%dm %ds' % (m, s)
        else:
            return '%ds' % s

    def handle_in_pipe(self, msg):
        with self._lock:
            downtime = self._downtime
            if not downtime:
                return
            try:
                msg = json.loads(msg.data)
                if msg['tracker'] != downtime['msg']['tracker']:
                    return
                if msg['text'] != downtime['msg']['text']:
                    return
                new_activity = msg['activity']
            except Exception as ex:
                rospy.logerr('[DowntimeTracker] invalid input: %s', ex)
                return
            self._downtime = None
            self.out({'tracker': None})

        LOCK_TIMEOUT = 10
        changed = False

        while not redis_cache.add('build_agv_statistics_lock', True, LOCK_TIMEOUT):
            rospy.sleep(0.1)

        with db_auto_reconnect(), transaction.atomic():
            p = AgvActivity.objects.select_for_update().filter(end__gt=downtime['start'], end__lte=downtime['end'])
            q = cache.get('AGV_STATISTICS_UPDATE_QUEUE', [])
            for pp in p:
                if pp.activity != new_activity:
                    q.append({
                        'start': pp.start,
                        'end': pp.end,
                        'activity0': pp.activity,
                        'activity': new_activity
                    })
                    changed = True
            if changed:
                cache.set('AGV_STATISTICS_UPDATE_QUEUE', q)
                p.update(activity=new_activity)

        redis_cache.delete('build_agv_statistics_lock')
        if changed:
            build_agv_statistics.apply_async()

    def out(self, data):
        self.__out_pipe.publish(std_msgs.msg.String(data=json.dumps(data)))
