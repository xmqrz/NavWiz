from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from rest_framework import exceptions, permissions, viewsets
from rest_framework.response import Response
import agv05_executive_msgs.srv
import rospy
import ujson as json

from ..serializers import TaskTemplateVariableSerializer


class VariableViewSet(viewsets.ViewSet):
    permission_classes = (permissions.IsAuthenticated,)
    serializer_class = TaskTemplateVariableSerializer
    lookup_field = 'name'
    agv05_executor = getattr(django_settings, 'AGV05_EXECUTOR')

    def list(self, request, *args, **kwargs):
        try:
            get_variable = rospy.ServiceProxy(
                self.agv05_executor + '/get_variable', agv05_executive_msgs.srv.GetVariable, persistent=False)
            res = get_variable()
            variable_list = json.loads(res.value)
        except Exception:
            raise exceptions.APIException('Robot controller not started.')

        return Response({
            'results': variable_list,
        })

    def retrieve(self, request, *args, **kwargs):
        name = kwargs['name']
        try:
            get_variable = rospy.ServiceProxy(
                self.agv05_executor + '/get_variable', agv05_executive_msgs.srv.GetVariable, persistent=False)
            res = get_variable(name)
        except Exception:
            raise exceptions.NotFound('Variable not found.')

        serializer = TaskTemplateVariableSerializer({
            'name': name,
            'type': res.type,
            'value': json.loads(res.value),
        })
        return Response(serializer.data)

    def update(self, request, *args, **kwargs):
        name = kwargs['name']
        serializer = TaskTemplateVariableSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        try:
            set_variable = rospy.ServiceProxy(
                self.agv05_executor + '/set_variable', agv05_executive_msgs.srv.SetVariable, persistent=False)
            res = set_variable(name, json.dumps(serializer.validated_data['value']))
        except Exception:
            raise exceptions.NotFound('Variable not found.')

        if not res.success:
            raise exceptions.ValidationError({'detail': 'Invalid value.'})

        serializer = TaskTemplateVariableSerializer({
            'name': name,
            'type': res.type,
            'value': json.loads(res.value),
        })
        return Response(serializer.data)
