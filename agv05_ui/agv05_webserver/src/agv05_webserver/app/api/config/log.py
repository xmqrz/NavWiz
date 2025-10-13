from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.tasks import bundle_log
from celery.result import AsyncResult
from collections import OrderedDict
from distutils.version import LooseVersion
from django.utils import timezone
from functools import cmp_to_key
from rest_framework import viewsets, status
from rest_framework.decorators import action
from rest_framework.generics import GenericAPIView
from rest_framework.response import Response
from rest_framework.reverse import reverse
from sendfile import sendfile
import os
import subprocess

from ..mixin import CustomErrorMixin, Permission
from ...serializers import LogRequestSerializer, LogDownloadSerializer

MAIN_LOG = '/var/log/navwiz'
PERIPHERAL_LOG = '/var/lib/navwiz/.ros/log'


class LogConfigViewSet(CustomErrorMixin, GenericAPIView, viewsets.ViewSet):
    permission_classes = Permission('system.view_log_files')

    def list(self, request, *args, **kwargs):
        return Response(OrderedDict([
            ('request', reverse('app:config-api:log-request', request=request)),
            ('download', reverse('app:config-api:log-download', request=request)),
            ('main-logs', reverse('app:config-api:log-main-logs', [''], request=request)),
            ('peripheral-logs', reverse('app:config-api:log-peripheral-logs', [''], request=request)),
        ]))

    @action(detail=False, methods=['POST'], serializer_class=LogRequestSerializer)
    def request(self, request, *args, **kwargs):
        serializer = LogRequestSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        start_date = serializer.validated_data['start_date']
        end_date = serializer.validated_data['end_date']

        result = bundle_log.apply_async(args=[start_date, end_date], expires=120)

        return Response({
            'request_id': result.id,
            'status': 'PENDING',
        })

    @action(detail=False, methods=['PUT', 'POST'], serializer_class=LogDownloadSerializer)
    def download(self, request, *args, **kwargs):
        serializer = LogDownloadSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        request_id = serializer.validated_data['request_id']

        result = AsyncResult(str(request_id))

        if result.successful():
            data = result.result
            filepath = data['filepath']

            if os.path.exists(filepath):
                if request.method == 'PUT':
                    return Response({
                        'request_id': result.id,
                        'status': 'READY',
                    })

                # Let nginx start serve file before we remove it.
                subprocess.Popen('sleep 3 && sudo rm %s' % filepath, shell=True)

                return sendfile(request, filepath, attachment=True, attachment_filename=data['filename'])
            else:
                return Response({
                    'request_id': result.id,
                    'status': 'FAILURE',
                })
        else:
            if result.state == 'EXECUTING':
                return Response({
                    'request_id': result.id,
                    'status': result.state,
                    'size': result.info['size'],
                })
            return Response({
                'request_id': result.id,
                'status': result.state,
            })

    @action(detail=False, methods=['GET'], url_path='main-logs/(?P<log_path>.*)')
    def main_logs(self, request, log_path=None, *args, **kwargs):
        dir_url_name = 'app:config-api:log-main-logs'
        file_url_name = 'main-log-download'
        return self._log_dir_response(request, MAIN_LOG, log_path, dir_url_name, file_url_name)

    @action(detail=False, methods=['GET'], url_path='peripheral-logs/(?P<log_path>.*)')
    def peripheral_logs(self, request, log_path=None, *args, **kwargs):
        dir_url_name = 'app:config-api:log-peripheral-logs'
        file_url_name = 'peripheral-log-download'
        return self._log_dir_response(request, PERIPHERAL_LOG, log_path, dir_url_name, file_url_name)

    def _log_dir_response(self, request, base_dir, log_path, dir_url_name, file_url_name):
        if not log_path:
            log_path = ''

        directory = os.path.join(base_dir, log_path)
        if not os.path.isdir(directory):
            self._custom_error('Invalid directory.', status_code=status.HTTP_404_NOT_FOUND)

        result = []
        try:
            for filename in os.listdir(directory):
                filepath = os.path.join(directory, filename)
                url_path = os.path.join(log_path, filename)
                stat = os.stat(filepath)
                modified_time = timezone.datetime.fromtimestamp(stat.st_mtime, timezone.get_current_timezone())
                if os.path.isdir(filepath):
                    url = reverse(dir_url_name, [url_path + os.sep], request=request)
                    result.append(OrderedDict((
                        ('name', filename + os.sep),
                        ('timestamp', modified_time),
                        ('size', '-'),
                        ('url', url),
                        ('is_directory', True),
                    )))
                else:
                    filesize = self._sizeof_fmt(stat.st_size)
                    url = reverse(file_url_name, [url_path], request=request)

                    result.append(OrderedDict((
                        ('name', filename),
                        ('timestamp', modified_time),
                        ('size', filesize),
                        ('url', url),
                        ('is_directory', False),
                    )))
        except Exception:
            pass

        raw_url = reverse(file_url_name, [log_path], request=request)
        result = sorted(result, key=cmp_to_key(loose_version_compare))

        up_path = os.path.dirname(log_path[:-1])

        if up_path != log_path:
            url = reverse(dir_url_name, [up_path], request=request)
            result.insert(0, OrderedDict((
                ('name', '../'),
                ('timestamp', ''),
                ('size', '-'),
                ('url', url),
                ('is_directory', True),
            )))

        return Response(OrderedDict((
            ('raw_url', raw_url),
            ('result', result),
        )))

    def _sizeof_fmt(self, num):
        for unit in ('B', 'KB', 'MB', 'GB', 'TB'):
            if abs(num) < 1024.0:
                return '%3.1f %s' % (num, unit)
            num /= 1024.0
        return '%3.1f %s' % (num, 'PB')


def loose_version_compare(a, b):
    if a['is_directory'] != b['is_directory']:
        if a['is_directory']:
            return -1
        else:
            return 1

    a = a['name']
    b = b['name']
    try:
        # Compare using LooseVersion
        v1 = LooseVersion(a)
        v2 = LooseVersion(b)
        if v1 > v2:
            return 1
        elif v1 < v2:
            return -1
        else:
            return 0
    except Exception:
        # Fallback to normal string comparison if LooseVersion fails
        if a > b:
            return 1
        elif a < b:
            return -1
        else:
            return 0
