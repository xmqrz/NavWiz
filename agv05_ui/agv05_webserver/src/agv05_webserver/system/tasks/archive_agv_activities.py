from __future__ import absolute_import
from __future__ import unicode_literals

from celery import shared_task
from django.conf import settings as django_settings
from django.core.cache import cache
from django.core.files import File
from django.utils import timezone
import codecs
import os
import sys

from ..models import AgvActivity


ARCHIVE_FOLDER = os.path.join('archive-x' if django_settings.TRACKLESS else 'archive', 'activities')


@shared_task(ignore_result=True)
def archive_agv_activities():
    last_end = timezone.now() - timezone.timedelta(days=7)
    last_update = cache.get('AGV_STATISTICS_UPDATE_TILL')
    if not last_update:
        return
    last_end = min(last_end, timezone.localtime(last_update).replace(hour=0, minute=0, second=0, microsecond=0))
    activities = AgvActivity.objects.filter(end__lt=last_end).order_by('end')
    activity_count = activities.count() - 10  # keep at least 10 most recent entries
    file = None

    for i in range(0, activity_count, 100):
        for a in activities[0:min(100, activity_count - i)]:
            end = timezone.localtime(a.end)
            dir = os.path.join(django_settings.MEDIA_ROOT, ARCHIVE_FOLDER, end.strftime('%Y/'))
            filename = os.path.join(dir, end.strftime('%m') + '.csv')

            if file is None or file.name != filename:
                if file is not None:
                    file.close()

                if os.path.exists(filename):
                    file = File(codecs.open(filename, 'a', 'utf-8'))
                else:
                    try:
                        os.makedirs(dir)
                    except OSError:
                        pass

                    file = File(codecs.open(filename, 'w', 'utf-8'))
                    file.write('\ufeff')  # Unicode BOM
                    file.write('"Start Time","End Time","Duration","Activity","Activity Code"\n')

            start = timezone.make_naive(a.start)
            end = timezone.make_naive(a.end)
            duration = a.end - a.start
            activity = a.get_activity_display()
            code = a.activity

            try:
                file.write('"%s","%s","%s","%s","%s"\n' % (
                    start, end, duration, activity, code))
            except Exception:
                sys.stderr.write('Failed to write row to file.\n')

            a.delete()

    if file is not None:
        file.close()
