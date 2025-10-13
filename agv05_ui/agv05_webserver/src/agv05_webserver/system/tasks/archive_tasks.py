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
import ujson as json

from ..models import Task


@shared_task(ignore_result=True)
def archive_tasks():
    last_mod = timezone.now() - timezone.timedelta(days=7)
    last_update = cache.get('TASK_STATISTICS_LAST_UPDATE')
    if not last_update:
        return
    last_mod = min(last_mod, timezone.localtime(last_update).replace(hour=0, minute=0, second=0, microsecond=0))
    tasks = Task.objects.completed().filter(modified__lt=last_mod).reverse().select_related('owner')
    task_count = tasks.count() - 10  # keep at least 10 most recent entries
    file = None

    for i in range(0, task_count, 100):
        for task in tasks[0:min(100, task_count - i)]:
            mod = timezone.localtime(task.modified)
            dir = os.path.join(django_settings.MEDIA_ROOT, 'archive/tasks/', mod.strftime('%Y/'))
            filename = os.path.join(dir, mod.strftime('%m') + '.csv')

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
                    file.write('"Name","Template","Params","Status","Progress","Creation Time","Scheduled Start Time","Start Time","End Time","Duration","Owner"\n')

            name = _csv_escape(task.name)
            template = _csv_escape(task.task_template)

            try:
                params = _csv_escape(' | '.join(['%s: %s' % p for p in json.loads(task.params).items()]))
            except Exception:
                params = ''

            try:
                status = task.get_status_display()
            except Exception:
                status = ''

            progress = _csv_escape(task.progress)
            created = timezone.make_naive(task.created) if task.created else ''
            start_after = timezone.make_naive(task.start_after) if task.start_after else ''
            run_start = timezone.make_naive(task.run_start) if task.run_start else ''
            run_end = timezone.make_naive(task.run_end) if task.run_end else ''
            duration = task.run_end - task.run_start if task.run_start and task.run_end and task.run_end >= task.run_start else ''
            owner = task.owner if task.owner else ''

            try:
                file.write('"%s","%s","%s","%s","%s","%s","%s","%s","%s","%s","%s"\n' % (
                    name, template, params, status, progress, created, start_after, run_start, run_end, duration, owner))
            except Exception:
                sys.stderr.write('Failed to write row to file.\n')

            task.delete()

    if file is not None:
        file.close()


def _csv_escape(s):
    return s.replace('"', '""')
