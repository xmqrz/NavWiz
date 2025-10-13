from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, TaskTemplate
from rest_framework import viewsets
from rest_framework.response import Response
from rest_framework.decorators import action
from rest_framework.reverse import reverse

from ..serializers import TaskTemplateSerializer


class TaskTemplateViewSet(viewsets.ReadOnlyModelViewSet):
    queryset = TaskTemplate.objects.all()
    serializer_class = TaskTemplateSerializer

    def list(self, request, *args, **kwargs):
        return Response({
            'active': reverse('app:api:tasktemplate-active', request=request),
        })

    @action(detail=False)
    def active(self, request, *args, **kwargs):
        results = [{
            'id': tt['id'],
            'name': tt['name'],
            'params': tt['params'],
            'outcomes': tt['outcomes'],
        } for tt in Cache.get_task_templates() if tt['is_top_level']]

        return Response({
            'results': results,
        })
