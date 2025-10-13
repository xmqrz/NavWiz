from __future__ import absolute_import
from __future__ import unicode_literals

from celery import shared_task
from django.core.cache import cache
from django.db.models import Count, DurationField, Sum, F
from django.utils import timezone
import ujson as json

from ..models import Task, TaskStatistic


@shared_task(ignore_result=True)
def build_task_statistics(rebuild_all=False):
    now = timezone.localtime()
    d1 = timezone.timedelta(days=1)
    d2 = timezone.timedelta(days=2)

    ts = TaskStatistic.objects.last()
    since = timezone.datetime.combine(ts.date, timezone.datetime.min.time()) + d1 if ts else \
        timezone.datetime(year=2013, month=1, day=1)
    since = timezone.make_aware(since)
    since = min(since, now - d2)

    last_update = cache.get('TASK_STATISTICS_LAST_UPDATE')
    if last_update:
        since = min(since, last_update) if rebuild_all else last_update
    since = timezone.localtime(since).replace(hour=0, minute=0, second=0, microsecond=0)

    date_list = Task.objects.completed(ordered=False).filter(modified__gte=since).datetimes('modified', 'day')
    statuses = ['Completed', 'Aborted', 'Cancelled']

    for d in date_list:
        ts = TaskStatistic()
        ts.date = d
        ts.by_status = {}
        ts.by_tt = {}
        for st in statuses:
            ts.by_status[st] = {'count': 0, 'duration': 0}
            ts.by_tt[st] = []

        tasks_by_status = Task.objects.completed(ordered=False) \
            .filter(modified__gte=d, modified__lt=d + d1) \
            .values('status').annotate(
                count=Count('status'),
                duration=Sum(F('run_end') - F('run_start'), output_field=DurationField()))

        tasks_by_tt = Task.objects.completed(ordered=False) \
            .filter(modified__gte=d, modified__lt=d + d1) \
            .values('status', 'task_template').annotate(
                count=Count('task_template'),
                duration=Sum(F('run_end') - F('run_start'), output_field=DurationField()))

        for tbs in tasks_by_status:
            ts.by_status[Task.Status(tbs['status']).name] = {
                'count': tbs['count'],
                'duration': int(tbs['duration'].total_seconds()) if tbs['duration'] else 0,
            }

        for tbtt in tasks_by_tt:
            ts.by_tt[Task.Status(tbtt['status']).name].append({
                'task_template': tbtt['task_template'],
                'count': tbtt['count'],
                'duration': int(tbtt['duration'].total_seconds()) if tbtt['duration'] else 0,
            })

        ts.by_status = json.dumps(ts.by_status)
        ts.by_tt = json.dumps(ts.by_tt)
        ts.save()

    cache.set('TASK_STATISTICS_LAST_UPDATE', now)
