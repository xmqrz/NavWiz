from __future__ import absolute_import
from __future__ import unicode_literals

from celery import shared_task
from collections import defaultdict
from django.core.cache import cache
from django.db import transaction
from django.db.models import Count, DurationField, Sum, F
from django.utils import timezone
import requests
import ujson as json

from ..models import AgvActivity, AgvStatistic, Cache, ExecutorMode, redis_cache


LOCK_TIMEOUT = 5 * 60

UNIX_EPOCH_DATE = timezone.datetime.utcfromtimestamp(0).date()


@shared_task(ignore_result=True)
def build_agv_statistics(rebuild_all=False):
    # ensure mutex with AgvActivityUpdateView and downtime tracker
    if not redis_cache.add('build_agv_statistics_lock', True, LOCK_TIMEOUT):
        return

    try:
        updates = _build_agv_statistics(rebuild_all)
        return _fms_sync_agv_statistics(updates)
    finally:
        redis_cache.delete('build_agv_statistics_lock')


def _build_agv_statistics(rebuild_all=False):
    now = timezone.localtime()
    today = now.date()
    zero = timezone.ZERO
    d1 = timezone.timedelta(days=1)
    d3 = timezone.timedelta(days=3)
    sss = {}

    ss = AgvStatistic.objects.last()
    since = timezone.datetime.combine(ss.date, timezone.datetime.min.time()) + d1 if ss else \
        timezone.datetime(year=2019, month=1, day=1)
    since = timezone.make_aware(since)
    since = min(since, now - d3)

    last_update = cache.get('AGV_STATISTICS_UPDATE_TILL')
    if last_update:
        since = min(since, last_update) if rebuild_all else last_update
    since = timezone.localtime(since).replace(hour=0, minute=0, second=0, microsecond=0)

    # read last known mileage and task counter
    last_extra = None
    if ss:
        try:
            last_extra = dict(json.loads(ss.by_activity)['extra'])
            last_extra_date = ss.date
        except Exception:
            pass

    # read current mileage and task counter
    import rospy
    import std_srvs.srv
    try:
        get_agv_status = rospy.ServiceProxy('/agv05_executor/get_agv_status', std_srvs.srv.Trigger, persistent=False)
        agv = json.loads(get_agv_status().message)
        extra = {
            'mileage': agv['mileage'],
            'task_counter': agv['task_counter'],
        }
    except Exception:
        extra = None

    # process update queue
    with transaction.atomic():
        updates = defaultdict(lambda: defaultdict(int))
        q = cache.get('AGV_STATISTICS_UPDATE_QUEUE', [])
        for p in q:
            start = timezone.localtime(p['start'])
            if start >= since:
                continue

            start0 = start.replace(hour=0, minute=0, second=0, microsecond=0)
            end = timezone.localtime(p['end'])
            if end >= since:
                end = since
                delta_days = (end - start0).days
            else:
                delta_days = (end - start0).days + 1

            ac0 = str(p['activity0'])
            ac = str(p['activity'])

            for i in range(delta_days):
                d = start0 + timezone.timedelta(days=i)
                duration = (min(end, d + d1) - max(start, d)).total_seconds()
                updates[d][ac0] -= duration
                updates[d][ac] += duration

        for d, v in updates.items():
            try:
                ss = AgvStatistic.objects.get(date=d.date())
                by_activity = json.loads(ss.by_activity)
                for ac, duration in v.items():
                    by_activity[ac] = by_activity.get(ac, 0) + duration
                    if not by_activity[ac]:
                        del by_activity[ac]
                ss.by_activity = json.dumps(by_activity)
            except Exception:
                continue
            ss.save()
            sss[ss.date] = by_activity

        cache.delete('AGV_STATISTICS_UPDATE_QUEUE')

    date_list = list(AgvActivity.objects.filter(end__gte=since).datetimes('end', 'day'))

    o9 = AgvActivity.objects.filter(end__gte=since).first()  # overnight - activity split into two or more days
    if not o9:
        cache.set('AGV_STATISTICS_LAST_UPDATE', now)
        return sss

    # generate new statistics since last update
    delta_days = (date_list[-1] - since).days + 1

    j = 0
    for i in range(delta_days):
        d = since + timezone.timedelta(days=i)
        ss = AgvStatistic()
        ss.date = d.date()

        # populate agv activity statistic
        if j < len(date_list) and d == date_list[j]:
            by_activity = dict(AgvActivity.objects.filter(end__gte=d, end__lt=d + d1)
                .values('activity')
                .annotate(duration=Sum(F('end') - F('start'), output_field=DurationField()))
                .values_list('activity', 'duration'))

            if o9.start < d:
                by_activity[o9.activity] -= d - o9.start

            o9 = AgvActivity.objects.filter(end__gte=d + d1).first()
            if o9 and o9.start < d + d1:
                duration = d1 - max(zero, o9.start - d)
                by_activity[o9.activity] = by_activity.get(o9.activity, zero) + duration

            by_activity = {k: int(v.total_seconds()) for k, v in by_activity.items()}
            j += 1
        else:
            if o9.start >= d + d1:
                continue
            duration = d1 - max(zero, o9.start - d)
            by_activity = {o9.activity: int(duration.total_seconds())}

        # populate mileage and task counter
        if ss.date == today and extra:
            if extra['mileage'] is None:
                if last_extra and last_extra_date <= ss.date:
                    extra['mileage'] = last_extra.get('mileage')
                else:
                    try:
                        ss0 = AgvStatistic.objects.get(date=ss.date)
                        extra['mileage'] = json.loads(ss0.by_activity)['extra']['mileage']
                    except Exception:
                        pass
            by_activity['extra'] = extra
        elif last_extra and last_extra_date <= ss.date <= today:
            by_activity['extra'] = last_extra
        else:
            try:
                ss0 = AgvStatistic.objects.get(date=ss.date)
                by_activity['extra'] = json.loads(ss0.by_activity)['extra']
            except Exception:
                pass

        ss.by_activity = json.dumps(by_activity)
        ss.save()
        sss[ss.date] = by_activity

    cache.set('AGV_STATISTICS_UPDATE_TILL', date_list[-1])
    cache.set('AGV_STATISTICS_LAST_UPDATE', now)
    return sss


def _fms_sync_agv_statistics(updates):
    if not _is_fms_mode():
        return
    try:
        endpoint, headers = _get_fms_endpoint_and_headers()
    except Exception:
        return

    last_sync = cache.get('AGV_STATISTICS_SYNC_TILL')
    if not last_sync or last_sync['Authorization'] != headers['Authorization']:
        today = timezone.localdate()
        month = today.replace(day=1)
        if today.day <= 10:
            month = (month - timezone.timedelta(days=1)).replace(day=1)
        last_sync = {
            'Authorization': headers['Authorization'],
            'agv_statistic': month,
            'agv_activity': month,
        }
    last_ss = last_sync['agv_statistic']
    last_ac = last_sync['agv_activity']

    # collect agv statistic
    sss = list(updates.items())
    qs = AgvStatistic.objects.filter(date__gte=last_ss).exclude(date__in=updates.keys())
    for ss in qs:
        try:
            by_activity = json.loads(ss.by_activity)
        except Exception:
            continue
        sss.append((ss.date, by_activity))
    sss.sort()

    # collect agv activity csv export
    from agv05_webserver.app.api.config.agv_activity import AgvActivityConfigViewSet
    v = AgvActivityConfigViewSet()
    v.request = type(str('Request'), (), {'data': {}})()

    months = set(d.replace(day=1) for d in updates)
    month = last_ac
    today = timezone.localdate()
    while month:
        months.add(month)
        month = v._get_next_month(month)
        if month > today:
            month = None

    acs = []
    for month in months:
        v.request.data = {
            'year': month.strftime('%Y'),
            'month': month.strftime('%m')
        }
        try:
            csv = v.download(v.request).getvalue().decode()
        except Exception:
            continue
        acs.append((month, csv))
    acs.sort()

    data = json.dumps({
        'uploads': {
            'agv_statistic': [(_datestamp(d), by_activity) for (d, by_activity) in sss],
            'agv_activity': [(_datestamp(d), csv) for (d, csv) in acs],
        },
    })

    # reset 'build_agv_statistics_lock' expiry proportionally to the size of data
    timeout_scale = len(data) // 1e7 + 1
    redis_cache.set('build_agv_statistics_lock', True, LOCK_TIMEOUT * timeout_scale)

    # push data to fms
    try:
        r = requests.put(endpoint, headers=headers, data=data, timeout=(3.05, 21 * timeout_scale),
                allow_redirects=False, verify=False)
        success = r.status_code == 200
    except Exception:
        success = False

    if success:
        if sss:
            last_sync['agv_statistic'] = sss[-1][0]
        if acs:
            last_sync['agv_activity'] = acs[-1][0]
    else:
        if sss:
            last_sync['agv_statistic'] = sss[0][0]
        if acs:
            last_sync['agv_activity'] = acs[0][0]

    if sss or acs:
        cache.set('AGV_STATISTICS_SYNC_TILL', last_sync)
    return success


def _is_fms_mode():
    return Cache.get_executor_mode() == ExecutorMode.DFleet.value


def _get_fms_endpoint_and_headers():
    agv_uuid = Cache.get_agv_uuid()
    fms_metadata = Cache.get_fms_metadata()

    endpoint = fms_metadata['live_endpoint']
    headers = {
        'Authorization': 'AgvToken %s:%s' % (agv_uuid, fms_metadata['token']),
        'Content-Type': 'application/json',
    }
    return endpoint, headers


def _datestamp(d):
    return (d - UNIX_EPOCH_DATE).total_seconds()
