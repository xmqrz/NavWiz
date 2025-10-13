from __future__ import absolute_import
from __future__ import unicode_literals

from datetime import date
from django.conf import settings as django_settings
from django.core.cache import cache
from django.core.files.storage import default_storage
from django.db import transaction
from django.http import StreamingHttpResponse, Http404
from django.utils import timezone
from rest_framework import permissions, viewsets, status
from rest_framework.decorators import action
from rest_framework.response import Response
from sendfile import sendfile
import itertools
import os
import time
import ujson as json

from agv05_webserver.system.models import Activity, AgvActivity, Variable, redis_cache
from agv05_webserver.system.tasks.build_agv_statistics import build_agv_statistics
from ...serializers import AgvActivityArchiveSerializer, AgvActivityConfigSerializer, AgvActivityChoicesSerializer, AgvActivityViewSerializer
from ..mixin import CustomErrorMixin, Permission
from .mixin import VariableEndpointMixin
from .license_void import LicenseVoidMixin

ARCHIVE_FOLDER = os.path.join(
    'archive-x' if django_settings.TRACKLESS else 'archive', 'activities')


class AgvActivityConfigViewSet(LicenseVoidMixin, CustomErrorMixin, VariableEndpointMixin, viewsets.ModelViewSet):
    permission_classes = (Permission(
        'system.view_agv_activities'), permissions.DjangoModelPermissions)
    serializer_class = AgvActivityViewSerializer
    queryset = AgvActivity.objects.order_by('-end')

    def update(self, request, *args, **kwargs):
        if not request.user.has_perm('system.change_agvactivity'):
            self.permission_denied(request)

        LOCK_TIMEOUT = 10
        changed = False
        new_activity = request.data.get('activity')

        counter = 0
        while not redis_cache.add('build_agv_statistics_lock', True, LOCK_TIMEOUT):
            time.sleep(0.1)
            counter += 1
            if counter >= 100:
                err_msg = 'The operation cannot be completed as the statistics are being computed. Please try again in a while.'
                return Response({'detail': err_msg}, status=status.HTTP_400_BAD_REQUEST)

        with transaction.atomic():
            instance = self.get_object()
            serializer = self.get_serializer(
                instance, data=request.data, partial=True)
            serializer.is_valid(raise_exception=True)

            if instance.activity != new_activity:
                q = cache.get('AGV_STATISTICS_UPDATE_QUEUE', [])
                q.append({
                    'start': instance.start,
                    'end': instance.end,
                    'activity0': instance.activity,
                    'activity': new_activity
                })
                cache.set('AGV_STATISTICS_UPDATE_QUEUE', q)

                instance.activity = new_activity
                instance.save()
                changed = True

        redis_cache.delete('build_agv_statistics_lock')
        if changed:
            build_agv_statistics.apply_async()
        return Response(serializer.data)

    @action(detail=False, methods=['GET'], url_path='get-activity-choices',
            permission_classes=Permission('system.view_agv_activities'),
            serializer_class=AgvActivityChoicesSerializer)
    def get_activity_choices(self, request, *args, **kwargs):
        return Response(json.dumps(Activity.choices()))

    @action(detail=False, methods=['GET', 'POST'],
            permission_classes=Permission(
                'system.view_agv_activities', 'system.change_agv_activities_config'),
            serializer_class=AgvActivityConfigSerializer)
    def configuration(self, request, *args, **kwargs):
        if request.method == 'GET':
            return self._get_config(request, *args, **kwargs)
        else:
            return self._update_config(request, *args, **kwargs)

    def _get_config(self, request, *args, **kwargs):
        name = Variable.AGV_ACTIVITY_CONFIG
        try:
            v = Variable.objects.get(name=name)
        except Exception:
            v = Variable(
                name=name,
                modified=timezone.now() - timezone.timedelta(days=1),
            )
            v.value = json.dumps({
                'labels': {k: 'Safety Trigger' for k, v in Activity.choices() if 100 <= k < 200},
            })
        v.activities = json.dumps(Activity.choices())

        serializer = self.get_serializer(v)
        return Response(serializer.data)

    def _update_config(self, request, *args, **kwargs):
        return self._variable_endpoint(Variable.AGV_ACTIVITY_CONFIG, request, *args, **kwargs)

    def _print_header(self):
        yield '\ufeff'  # Unicode BOM
        yield '"Start Time","End Time","Duration","Activity","Activity Code"\n'

    def _print_row(self, activities):
        for a in activities:
            start = timezone.make_naive(a.start)
            end = timezone.make_naive(a.end)
            duration = a.end - a.start
            activity = a.get_activity_display()
            code = a.activity

            yield '"%s","%s","%s","%s","%s"\n' % (start, end, duration, activity, code)

    def _get_next_month(self, date):
        if date.month == 12:
            return date.replace(year=date.year + 1, month=1, day=1)
        else:
            return date.replace(month=date.month + 1, day=1)

    @action(detail=False, methods=['POST'],
            permission_classes=Permission('system.view_agv_activities'),
            serializer_class=AgvActivityViewSerializer)
    def download(self, request, *args, **kwargs):
        year = int(request.data.get('year', date.today().year))
        month_number = int(request.data.get('month', date.today().month))
        month = date(year=year, month=month_number, day=1)

        lookup = {
            'start__gte': month,
            'start__lt': self._get_next_month(month),
        }
        qs = self.get_queryset().filter(**lookup).reverse()

        file_path = os.path.join(ARCHIVE_FOLDER, month.strftime('%Y/%m.csv'))
        attachment_filename = month.strftime('%Y_%m.csv')

        if not qs and hasattr(request, 'META'):  # avoid sendfile if fake call
            abs_path = os.path.join(default_storage.location, file_path)
            return sendfile(request, abs_path, attachment=True, attachment_filename=attachment_filename)

        if default_storage.exists(file_path):
            response_gen = itertools.chain(
                default_storage.open(file_path), self._print_row(qs))
        elif not qs:
            raise Http404()
        else:
            response_gen = itertools.chain(
                self._print_header(), self._print_row(qs))

        response = StreamingHttpResponse(response_gen, content_type="text/csv")
        response['Content-Disposition'] = 'attachment; filename="%s"' % attachment_filename
        return response


class AgvActivityArchiveConfigViewSet(LicenseVoidMixin, viewsets.ViewSet):
    permission_classes = Permission('system.view_agv_activities')
    serializer_class = AgvActivityArchiveSerializer

    def list(self, request, *args, **kwargs):
        queryset = self.get_queryset()
        serializer = self.serializer_class(queryset, many=True)
        return Response(serializer.data)

    def get_queryset(self):
        year = int(self.request.query_params.get('year', date.today().year))
        in_db = [d.date() for d in self.get_agv_activity_queryset(
            year).datetimes('start', 'month')]
        qs = []
        for i in range(1, 13):
            month = date(year=year, month=i, day=1)
            exists = (month in in_db or
                      default_storage.exists(os.path.join(ARCHIVE_FOLDER, month.strftime('%Y/%m.csv'))))
            qs.append(
                {'month_number': i, 'month': month.strftime('%B'), 'exists': exists})
        return qs

    def get_agv_activity_queryset(self, year):
        return AgvActivity.objects.order_by('-end').filter(start__year=year)
