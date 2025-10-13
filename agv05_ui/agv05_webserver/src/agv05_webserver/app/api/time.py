from __future__ import absolute_import
from __future__ import unicode_literals

from django.utils import timezone
from rest_framework import permissions, viewsets
from rest_framework.response import Response


class TimeViewSet(viewsets.ViewSet):
    permission_classes = (permissions.AllowAny,)

    def list(self, request, *args, **kwargs):
        now = timezone.now()
        tz = timezone.get_current_timezone()
        return Response({
            'now': now.astimezone(tz).isoformat(),
            'tz': tz.zone,
        })
