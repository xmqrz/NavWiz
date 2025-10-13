from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache
from django.conf import settings as django_settings
from rest_framework import exceptions, permissions, viewsets
from rest_framework.response import Response
import agv05_executive_msgs.srv
import rospy
import ujson as json

from ..serializers import RegisterSerializer


class RegisterViewSet(viewsets.ViewSet):
    permission_classes = (permissions.IsAuthenticated,)
    serializer_class = RegisterSerializer
    lookup_field = 'name'
    lookup_value_regex = '[A-Z0-9]+'
    agv05_executor = getattr(django_settings, 'AGV05_EXECUTOR')

    def list(self, request, *args, **kwargs):
        try:
            skillset = json.loads(Cache.get_models_skillset())
            register_list = skillset['register_list']
        except Exception:
            raise exceptions.APIException('Robot controller not started.')

        return Response({
            'results': register_list,
        })

    def retrieve(self, request, *args, **kwargs):
        name = kwargs['name']
        try:
            get_register = rospy.ServiceProxy(self.agv05_executor + '/get_register', agv05_executive_msgs.srv.GetRegister, persistent=False)
            res = get_register(name)
        except Exception:
            raise exceptions.NotFound('Register not found.')

        serializer = RegisterSerializer({
            'name': name,
            'value': res.value,
        })
        return Response(serializer.data)

    def update(self, request, *args, **kwargs):
        name = kwargs['name']
        serializer = RegisterSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        try:
            set_register = rospy.ServiceProxy(self.agv05_executor + '/set_register', agv05_executive_msgs.srv.SetRegister, persistent=False)
            res = set_register(name, serializer.validated_data['value'])
        except Exception:
            raise exceptions.NotFound('Register not found.')

        serializer.validated_data['name'] = name
        return Response(serializer.data)
