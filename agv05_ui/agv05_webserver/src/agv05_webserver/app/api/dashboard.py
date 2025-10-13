from __future__ import absolute_import
from __future__ import unicode_literals

from django.utils import timezone
from datetime import timedelta
from rest_framework import permissions, status, viewsets
from rest_framework.decorators import action
from rest_framework.reverse import reverse
from rest_framework.response import Response
import subprocess

from agv05_webserver.system.models import Cache, Task
from agv05_webserver.system.views.dashboard import dashboard_params
from .mixin import Permission, CustomErrorMixin
from .version import _get_version

NAVWIZ_HOME = '/var/lib/navwiz/'


def _get_uptime():
    try:
        uptime_output = subprocess.check_output(
            ['uptime', '-p'],
            universal_newlines=True
        ).strip()
        return uptime_output.replace('up ', '') \
            .replace(' days', 'd') \
            .replace(' day', 'd') \
            .replace(' hours', 'h') \
            .replace(' hour', 'h') \
            .replace(' minutes', 'm') \
            .replace(' minute', 'm') \
            .replace(',', '')
    except Exception:
        raise
        return '-'


def _get_today_task_statistic():
    now = timezone.now()
    start_of_day = now.replace(hour=0, minute=0, second=0, microsecond=0)
    end_of_day = start_of_day + timedelta(days=1)
    today_task = Task.objects.completed().filter(created__gte=start_of_day, created__lt=end_of_day)
    assigned = today_task.count()
    completed = today_task.filter(status=Task.Status.Completed.value).count()
    return (assigned, completed)


class DashboardViewSet(CustomErrorMixin, viewsets.ViewSet):
    permission_classes = (permissions.AllowAny,)

    # this is actually a `retrieve` operation portrayed as `list` so that
    # it can be included in the router's urls.
    def list(self, request, *args, **kwargs):
        if not request.user.has_perm('system.view_system_panel'):
            self.permission_denied(request)
        assigned_task, completed_task = _get_today_task_statistic()
        resp = _get_version()
        resp['assigned_task'] = assigned_task
        resp['completed_task'] = completed_task
        resp['uptime'] = _get_uptime()
        resp['skillset'] = Cache.get_models_skillset()
        resp['skillset_md5'] = Cache.get_models_skillset_md5()
        resp['skillset_img'] = '%s?size=128x128&set=set1&bgset=bg1' % reverse(
            'hashicon:index',
            [resp['skillset_md5']],
            request=request) if resp['skillset_md5'] else ''
        resp['stats'] = Cache.get_dashboard_stats()
        return Response(resp)

    @action(detail=False, methods=['GET'], permission_classes=Permission('system.view_panel'))
    def entry(self, request, *args, **kwargs):
        params = dashboard_params()
        if not params:
            self._custom_error('Dashboard does not exist.', status_code=status.HTTP_404_NOT_FOUND)
        entry = reverse(
            'system:dashboard',
            request=request,
            kwargs={
                'asset': params['entry'],
            }
        )
        return Response({
            'entry': entry
        })
