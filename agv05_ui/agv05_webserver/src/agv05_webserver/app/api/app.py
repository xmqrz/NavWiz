from __future__ import absolute_import
from __future__ import unicode_literals

from distutils.util import strtobool
from django.conf import settings as django_settings
from django.core.exceptions import PermissionDenied
from django.http import Http404
from django.utils.encoding import force_text
from rest_framework import permissions, viewsets
from rest_framework.decorators import action
from rest_framework.exceptions import NotAuthenticated
from rest_framework.response import Response
from rest_framework.reverse import reverse
import agv05_executive_msgs.srv
import logging
import rospy
import std_msgs.msg
import ujson as json

from ..serializers import AppSerializer, HwAppSerializer
from agv05_webserver.system.models import Cache


logger = logging.getLogger(__name__)


class AppViewSet(viewsets.ViewSet):
    permission_classes = (permissions.AllowAny,)
    serializer_class = AppSerializer
    agv05_executor = getattr(django_settings, 'AGV05_EXECUTOR')
    lookup_value_regex = '[\\w._-]+'

    def list(self, request, *args, **kwargs):
        app_descriptions = Cache.get_models_app_descriptions() or []
        serializer = HwAppSerializer(app_descriptions, many=True)
        return Response({
            'task-runner-command': reverse('app:api:app-task-runner-command', request=request),
            'hw-app-descriptions': serializer.data,
        })

    @action(methods=['post'], detail=False, permission_classes=(permissions.AllowAny,), url_path='task-runner-command')
    def task_runner_command(self, request, *args, **kwargs):
        try:
            cmd = request.data['command']
            if cmd == 'pause':
                pause = request.data['pause']
                if not isinstance(pause, bool):
                    pause = strtobool(pause)

                if pause and not request.user.has_perm('app.pause_task_runner'):
                    self.permission_denied(request)
                elif not pause and not request.user.has_perm('app.resume_task_runner'):
                    self.permission_denied(request)

                self._send_task_runner_command({
                    'command': 'pause',
                    'pause': pause,
                })
                return Response({'result': True, 'message': ''})
        except (PermissionDenied, NotAuthenticated):
            raise
        except Exception as ex:
            logger.error('Exception: %s', ex)

        return Response({'result': False, 'message': 'Request error.'})

    def _send_task_runner_command(self, data):
        pub = rospy.Publisher(self.agv05_executor + '/task_runner_command', std_msgs.msg.String, queue_size=1)
        pub.publish(std_msgs.msg.String(data=json.dumps(data)))

    def create(self, request, *args, **kwargs):
        serializer = AppSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        operation = serializer.validated_data['operation']
        app_id = serializer.validated_data['app_id']

        if operation == AppSerializer.Operation.START_APP:
            if app_id == 'task-runner':
                perm = 'app.start_task_runner'
            else:
                perm = 'app.start_app'
        else:
            if app_id == 'task-runner':
                perm = 'app.stop_task_runner'
            else:
                perm = 'app.stop_app'

        if not request.user.has_perm(perm):
            self.permission_denied(request)

        success = False
        try:
            set_app_running = rospy.ServiceProxy(self.agv05_executor + '/set_module_running', agv05_executive_msgs.srv.SetModuleRunning, persistent=False)
            res = set_app_running(operation, app_id)
            if res.success:
                success = True
            error_msg = res.message
        except Exception as ex:
            error_msg = force_text(ex)

        if not success:
            if operation == AppSerializer.Operation.START_APP:
                if app_id == 'task-runner':
                    message = 'Failed to start task runner.'
                else:
                    message = 'Failed to start app.'
            else:
                if app_id == 'task-runner':
                    message = 'Failed to stop task runner.'
                else:
                    message = 'Failed to stop app.'
            if error_msg:
                message += '\n\nError: %s' % error_msg
            return Response({'result': False, 'message': message})

        return Response({'result': True})

    def retrieve(self, request, pk=None, *args, **kwargs):
        app_descriptions = Cache.get_models_app_descriptions() or []
        app_detail = next((a for a in app_descriptions if a['id'] == pk), None)

        if app_detail is None:
            raise Http404('Invalid app id.')

        serializer = HwAppSerializer(app_detail)
        return Response(serializer.data)
