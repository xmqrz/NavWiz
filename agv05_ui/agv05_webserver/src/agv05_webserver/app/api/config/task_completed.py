from __future__ import absolute_import
from __future__ import unicode_literals

from datetime import date
from django.core.files.storage import default_storage
from django.http import StreamingHttpResponse
from django.utils import timezone
from rest_framework import serializers
from rest_framework import status, viewsets
from rest_framework.decorators import action
from rest_framework.response import Response
from sendfile import sendfile
import gzip
import itertools
# couldn't use ujson here as it doesn't accept argument 'cls' (for DjangoJSONEncoder)
import json
import os

from ...serializers import TaskCompletedSerializer, TaskCompletedArchiveSerializer
from ..mixin import Permission
from agv05_webserver.system.models import Task


# TODO: move this to api folder (parent folder)
class TaskCompletedConfigViewSet(viewsets.ModelViewSet):
    permission_classes = Permission('system.view_completed_tasks')
    serializer_class = TaskCompletedSerializer
    queryset = Task.objects.completed().select_related('owner')

    def _get_next_month(self, date):
        if date.month == 12:
            return date.replace(year=date.year + 1, month=1, day=1)
        else:
            return date.replace(month=date.month + 1, day=1)

    def _print_header(self):
        yield '\ufeff'  # Unicode BOM
        yield '"Name","Template","Params","Status","Progress","Creation Time","Scheduled Start Time","Start Time","End Time","Duration","Owner"\n'

    def _print_row(self, tasks):
        for task in tasks:
            name = self._csv_escape(task.name)
            template = self._csv_escape(task.task_template)

            try:
                params = self._csv_escape(' | '.join(
                    ['%s: %s' % p for p in json.loads(task.params).items()]))
            except Exception:
                params = ''

            try:
                status = task.get_status_display()
            except Exception:
                status = ''

            progress = self._csv_escape(task.progress)
            created = timezone.make_naive(task.created) if task.created else ''
            start_after = timezone.make_naive(
                task.start_after) if task.start_after else ''
            run_start = timezone.make_naive(
                task.run_start) if task.run_start else ''
            run_end = timezone.make_naive(task.run_end) if task.run_end else ''
            duration = task.run_end - \
                task.run_start if task.run_start and task.run_end and task.run_end >= task.run_start else ''
            owner = task.owner if task.owner else ''

            yield '"%s","%s","%s","%s","%s","%s","%s","%s","%s","%s","%s"\n' % (
                name, template, params, status, progress, created, start_after, run_start, run_end, duration, owner)

    def _csv_escape(self, s):
        return s.replace('"', '""')

    @action(detail=False, methods=['POST'],
            permission_classes=Permission('system.view_completed_tasks'),
            serializer_class=TaskCompletedSerializer)
    def download(self, request, *args, **kwargs):
        year = int(request.data.get('year', date.today().year))
        month_number = int(request.data.get('month', date.today().month))
        month = date(year=year, month=month_number, day=1)

        lookup = {
            'modified__gte': month,
            'modified__lt': self._get_next_month(month),
        }
        qs = self.get_queryset().filter(**lookup).reverse()

        file_path = os.path.join('archive/tasks', month.strftime('%Y/%m.csv'))
        attachment_filename = month.strftime('%Y_%m.csv')

        if not qs:
            abs_path = os.path.join(default_storage.location, file_path)
            return sendfile(self.request, abs_path, attachment=True, attachment_filename=attachment_filename)

        if default_storage.exists(file_path):
            response_gen = itertools.chain(
                default_storage.open(file_path), self._print_row(qs))
        else:
            response_gen = itertools.chain(
                self._print_header(), self._print_row(qs))

        response = StreamingHttpResponse(response_gen, content_type="text/csv")
        response['Content-Disposition'] = 'attachment; filename="%s"' % attachment_filename
        return response

    def _read_files(self, start_date, end_date):
        try:
            folder = os.path.join(os.environ['ROS_HOME'], 'diagnostics')
        except Exception:
            folder = os.path.join(os.environ['HOME'], '.ros', 'diagnostics')

        d = start_date
        while d <= end_date:
            filename = os.path.join(folder, d.strftime('%Y%m%d.csv'))
            d += timezone.timedelta(days=1)
            try:
                with gzip.open(filename + '.1.gz') as f:
                    line = f.readline()
                    if line.startswith(b'\xef\xbb\xbf'):  # Remove Unicode BOM
                        line = line[3:]
                    yield line
                    for line in f:
                        yield line
                yield b'\n'
                yield b'\n'
            except Exception:
                try:
                    with open(filename, 'rb') as f:
                        line = f.readline()
                        if line.startswith(b'\xef\xbb\xbf'):  # Remove Unicode BOM
                            line = line[3:]
                        yield line
                        for line in f:
                            yield line
                    yield b'\n'
                    yield b'\n'
                except Exception:
                    pass

    def _filter_lines(self, f, start, end):
        prefix_format = '"%Y-%m-%d %H:%M:%S"'
        prefix_len = len(end.strftime(prefix_format))
        started = False
        header = ''

        for line in f:
            if line.startswith(b'"Timestamp"'):
                if started:
                    yield line
                else:
                    header = line
                continue

            try:
                prefix = line[:prefix_len].decode('latin')
                t = timezone.datetime.strptime(prefix, prefix_format)
            except Exception:
                pass
            else:
                if t > end:
                    return
                if not started and t >= start:
                    started = True
                    yield '\ufeff'  # Unicode BOM
                    yield header

            if started:
                yield line

    @action(detail=True, methods=['POST'], url_path='download-diagnostics',
            permission_classes=Permission(
                'system.view_completed_tasks', 'system.view_log_files'),
            serializer_class=serializers.Serializer)
    def download_diagnostics(self, request, *args, **kwargs):
        task = self.get_object()
        if not task.run_start or not task.run_end:
            return Response({"error": 'Either the start or end time of the task is empty.'}, status=status.HTTP_400_BAD_REQUEST)
        start = timezone.make_naive(task.run_start)
        end = timezone.make_naive(task.run_end)
        attachment_filename = '%s_%s.csv' % (
            end.strftime('%Y%m%d_%H%M%S'), task.name)

        if end - start > timezone.timedelta(days=1):
            return Response({"error": 'Not extracting diagnostics for task spanning more than 24 hours.'}, status=status.HTTP_400_BAD_REQUEST)

        f = self._read_files(start.date(), end.date())
        response_gen = self._filter_lines(f, start, end)

        try:
            peek = next(response_gen)
        except Exception:
            return Response({"error": 'There is no diagnostic record found within the specified time frame.'}, status=status.HTTP_400_BAD_REQUEST)

        response_gen = itertools.chain([peek], response_gen)
        response = StreamingHttpResponse(response_gen, content_type="text/csv")
        response['Content-Disposition'] = 'attachment; filename="%s"' % attachment_filename
        return response


class TaskCompletedArchiveConfigViewSet(viewsets.ViewSet):
    permission_classes = Permission('system.view_completed_tasks')
    serializer_class = TaskCompletedArchiveSerializer

    def list(self, request, *args, **kwargs):
        queryset = self.get_queryset()
        serializer = self.serializer_class(queryset, many=True)
        return Response(serializer.data)

    def get_queryset(self):
        year = int(self.request.query_params.get('year', date.today().year))
        in_db = [d.date() for d in self.get_task_queryset(
            year).datetimes('modified', 'month')]
        qs = []
        for i in range(1, 13):
            month = date(year=year, month=i, day=1)
            exists = (month in in_db or
                      default_storage.exists(os.path.join('archive/tasks', month.strftime('%Y/%m.csv'))))
            qs.append(
                {'month_number': i, 'month': month.strftime('%B'), 'exists': exists})
        return qs

    def get_task_queryset(self, year):
        return Task.objects.completed().select_related('owner').filter(modified__year=year)
