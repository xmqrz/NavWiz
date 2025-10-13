from __future__ import absolute_import
from __future__ import unicode_literals

from celery import shared_task
from datetime import timedelta, datetime
from django.db.models import DurationField, Sum, F
from django.utils import timezone
import pytz
import ujson as json

from agv05_webserver.system.models import Cache, Task, AgvActivity, Activity


@shared_task(ignore_result=True)
def build_dashboard_stats():
    stats = Cache.get_dashboard_stats()

    end = timezone.now()
    end = end.replace(minute=0, second=0, microsecond=0)
    start = end - timedelta(hours=1)

    # get battery and mileage
    battery = 0
    mileage = 0
    import rospy
    import std_srvs.srv
    try:
        get_agv_status = rospy.ServiceProxy('/agv05_executor/get_agv_status', std_srvs.srv.Trigger, persistent=False)
        agv = json.loads(get_agv_status().message)
        battery = agv['battery']
        mileage = agv['mileage']
    except Exception:
        # prevent update if executor not running.
        return

    _update_task_stats(stats, start, end)
    _update_battery_stats(stats, start, end, battery)
    _update_mileage_stats(stats, start, end, mileage)
    _update_activity_stats(stats, start, end)

    Cache.set_dashboard_stats(stats)


def _update_task_stats(stats, start, end):
    tasks = stats.get('tasks', [])
    last_tasks = stats.get('last_week_tasks', [])

    tasks_start = start - timedelta(hours=3)
    new_tasks = __update_task_stats(tasks, tasks_start, end)

    last_week = start - timedelta(days=7)
    last_week_start = last_week - timedelta(hours=3)
    last_week_end = last_week + timedelta(hours=4)
    new_last_tasks = __update_task_stats(last_tasks, last_week_start, last_week_end)

    stats['tasks'] = new_tasks
    stats['last_week_tasks'] = new_last_tasks


def __update_task_stats(tasks, start, end):
    new_tasks = []
    for time, count in tasks:
        t = pytz.utc.localize(datetime.strptime(time, '%Y-%m-%dT%H:%M:%S'))
        if t <= start:
            continue

        if t > end:
            break

        # fill older range
        cur = start + timedelta(hours=1)
        while t > cur and cur <= end:
            new_tasks.append([
                cur.strftime('%Y-%m-%dT%H:%M:%S'),
                Task.objects.completed().filter(created__gt=start, created__lte=cur).count()
            ])
            start = cur
            cur = start + timedelta(hours=1)

        if start == end:
            break

        new_tasks.append([
            t.strftime('%Y-%m-%dT%H:%M:%S'),
            count,
        ])
        start = t

    while start < end:
        n = start
        start = n + timedelta(hours=1)
        new_tasks.append([
            start.strftime('%Y-%m-%dT%H:%M:%S'),
            Task.objects.completed().filter(created__gt=n, created__lte=start).count()
        ])

    assert start == end

    return new_tasks


def _update_battery_stats(stats, start, end, battery):
    percentage = int(5 * round(battery / 5))
    battery = stats.get('battery', [])
    battery_start = end - timedelta(hours=7)
    new_battery = []

    for time, p in battery:
        t = pytz.utc.localize(datetime.strptime(time, '%Y-%m-%dT%H:%M:%S'))
        if t < battery_start:
            continue

        if t >= end:
            break

        new_battery.append([time, p])

    new_battery.append([
        end.strftime('%Y-%m-%dT%H:%M:%S'),
        percentage
    ])

    stats['battery'] = new_battery


def _update_mileage_stats(stats, start, end, mileage):
    mileage = round(mileage)

    mileage_stats = stats.get('mileage', [])
    mileage_start = end - timedelta(hours=6)
    new_mileage = []

    for time, m in mileage_stats:
        t = pytz.utc.localize(datetime.strptime(time, '%Y-%m-%dT%H:%M:%S'))
        if t < mileage_start:
            continue

        if t >= end:
            break

        new_mileage.append([time, m])

    new_mileage.append([
        end.strftime('%Y-%m-%dT%H:%M:%S'),
        mileage
    ])

    stats['mileage'] = new_mileage


def _update_activity_stats(stats, start, end):
    activity_start = end - timedelta(hours=6)
    agv_activity = dict((Activity.title(a), d.total_seconds()) for a, d in AgvActivity.objects.filter(end__gte=activity_start, start__lt=end)
        .values('activity')
        .annotate(duration=Sum(F('end') - F('start'), output_field=DurationField()))
        .values_list('activity', 'duration'))

    overlap_start = AgvActivity.objects.filter(end__gte=activity_start).first()
    if overlap_start and overlap_start.start < activity_start:
        agv_activity[Activity.title(overlap_start.activity)] -= (activity_start - overlap_start.start).total_seconds()
    overlap_end = AgvActivity.objects.filter(start__lt=end).last()
    if overlap_end and overlap_end.end > end:
        agv_activity[Activity.title(overlap_end.activity)] -= (overlap_end.end - end).total_seconds()

    stats['agv_activity'] = agv_activity
