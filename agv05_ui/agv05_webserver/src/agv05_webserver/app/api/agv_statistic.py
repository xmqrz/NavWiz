from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import AgvStatistic
from django.core.cache import cache
from rest_framework import viewsets

from ..pagination import MonthPagination, YearPagination
from ..serializers import AgvStatisticSerializer
from .mixin import Permission


class AgvStatisticViewSet(viewsets.ReadOnlyModelViewSet):
    permission_classes = Permission('system.view_agv_activities')
    queryset = AgvStatistic.objects.all()
    serializer_class = AgvStatisticSerializer
    pagination_class = MonthPagination

    def _get_extra(self):
        d = {'last_update': cache.get('AGV_STATISTICS_LAST_UPDATE')}
        res0 = self.queryset.filter(date__lt=self.paginator.month).last()
        if res0:
            d['result0'] = AgvStatisticSerializer(res0).data
        return d


class AgvActivitiesYearPagination(viewsets.ReadOnlyModelViewSet):
    permission_classes = Permission('system.view_agv_activities')
    queryset = AgvStatistic.objects.all()
    serializer_class = AgvStatisticSerializer
    pagination_class = YearPagination
