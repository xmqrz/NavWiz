from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from rest_framework import permissions, status, viewsets
from rest_framework.response import Response
import std_srvs.srv
import rospy
import ujson as json

from ..serializers import AgvSerializer
from .mixin import CustomErrorMixin


class AgvViewSet(CustomErrorMixin, viewsets.ViewSet):
    permission_classes = (permissions.IsAuthenticated,)

    agv05_executor = getattr(django_settings, 'AGV05_EXECUTOR')

    # this is actually a `retrieve` operation portrayed as `list` so that
    # it can be included in the router's urls.
    def list(self, request, *args, **kwargs):
        try:
            serializer = AgvSerializer(self._get_status())
        except Exception:
            self._custom_error('Executor not running.', status_code=status.HTTP_404_NOT_FOUND)
        return Response(serializer.data)

    def _get_status(self):
        get_agv_status = rospy.ServiceProxy(self.agv05_executor + '/get_agv_status', std_srvs.srv.Trigger, persistent=False)
        return json.loads(get_agv_status().message)
