from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import TaskStatistic
from django.core.cache import cache
from rest_framework import viewsets

from ..pagination import MonthPagination, YearPagination
from ..serializers import TaskStatisticSerializer
from .mixin import Permission


class TaskStatisticViewSet(viewsets.ReadOnlyModelViewSet):
    permission_classes = Permission('system.view_completed_tasks')
    queryset = TaskStatistic.objects.all()
    serializer_class = TaskStatisticSerializer
    pagination_class = MonthPagination

    def _get_extra(self):
        return {
            'last_update': cache.get('TASK_STATISTICS_LAST_UPDATE'),
        }


class TaskCompletedYearPagination(viewsets.ReadOnlyModelViewSet):
    permission_classes = Permission('system.view_completed_tasks')
    queryset = TaskStatistic.objects.all()
    serializer_class = TaskStatisticSerializer
    pagination_class = YearPagination
