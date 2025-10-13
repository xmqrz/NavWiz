from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, ExecutorMode, Task
from django.conf import settings as django_settings
from django.contrib.auth import get_user_model
from django.utils import six
from django.utils.encoding import force_text
from rest_framework import exceptions, mixins, permissions, status, viewsets
from rest_framework.decorators import action
from rest_framework.response import Response
import agv05_executive_msgs.srv
import requests
import rospy
import ujson as json

from .mixin import CustomErrorMixin
from ..serializers import FmsTaskSerializer, TaskSerializer


User = get_user_model()


class TaskViewSet(CustomErrorMixin, mixins.CreateModelMixin, viewsets.ReadOnlyModelViewSet):
    permission_classes = (permissions.DjangoModelPermissionsOrAnonReadOnly,)
    serializer_class = TaskSerializer
    queryset = Task.objects.all()
    lookup_value_regex = '[0-9]+'
    agv05_executor = getattr(django_settings, 'AGV05_EXECUTOR')

    def get_queryset(self):
        queryset = super(TaskViewSet, self).get_queryset()
        status = self.request.query_params.get('status', None)
        if status is not None:
            try:
                status = Task.Status[status].value
            except Exception:
                queryset = queryset.none()
            else:
                queryset = queryset.filter(status=status)
        return queryset.select_related('owner')

    def check_permissions(self, request):
        super(TaskViewSet, self).check_permissions(request)
        self.user_instance = request.user if request.user.is_authenticated \
            else User.objects.get(username='Guest')
        self.user_instance_superset = User.objects.get(username='agv_panel') \
            if request.user.username == 'agv_panel_pin_protected' else None

    @action(detail=False)
    def running(self, request, *args, **kwargs):
        self.queryset = Task.objects.running().select_related('owner')
        return self.list(request, *args, **kwargs)

    @action(detail=False)
    def completed(self, request, *args, **kwargs):
        self.queryset = Task.objects.completed().select_related('owner')
        return self.list(request, *args, **kwargs)

    @action(detail=False)
    def templates(self, request, *args, **kwargs):
        data = {}
        try:
            skillset = json.loads(Cache.get_models_skillset())
            data['skill_descriptions'] = skillset['skill_descriptions']
        except Exception:
            pass

        try:
            data['task_templates'] = Cache.get_task_templates()
        except Exception:
            pass

        return Response(data)

    def perform_create(self, serializer):
        if Task.objects.running(ordered=False).count() >= 100:
            raise exceptions.Throttled(detail='Task queue is full.')

        if self._is_fms_mode():
            return self._fms_perform_create(serializer)

        template_id = serializer.validated_data.get('task_template_id')
        template_name = serializer.validated_data.get('task_template')
        tt = Cache.get_task_template_if_allowed(self.user_instance, template_id=template_id, template_name=template_name)

        if tt:
            serializer.validated_data['task_template_id'] = tt['id']
            serializer.validated_data['task_template'] = tt['name']
        elif tt is False:
            self.permission_denied(self.request, 'Task template is not available.')
        else:
            raise exceptions.ValidationError({'detail': 'Task template is not available.'})

        if not serializer.validated_data['name']:
            serializer.validated_data['name'] = tt['name']

        if not isinstance(serializer.validated_data['params'], six.string_types):
            serializer.validated_data['params'] = json.dumps(serializer.validated_data['params'])

        serializer.validated_data['status'] = Task.Status.Suspended.value if tt['create_suspended'] else Task.Status.Pending.value
        serializer.validated_data['owner'] = self.user_instance
        super(TaskViewSet, self).perform_create(serializer)

        try:
            create_task = rospy.ServiceProxy(self.agv05_executor + '/create_task', agv05_executive_msgs.srv.TriggerTask, persistent=False)
            res = create_task(serializer.data['id'])
        except Exception:
            pass

    def _fms_perform_create(self, serializer):
        try:
            endpoint, headers = self._get_fms_endpoint_and_headers()
        except Exception:
            self._custom_error('Error pairing with DFleet', status_code=status.HTTP_502_BAD_GATEWAY)

        data = json.dumps(serializer.validated_data)
        try:
            r = requests.post(endpoint, headers=headers, data=data, timeout=(3.05, 21), allow_redirects=False, verify=False)
            if r.is_redirect:
                self._custom_error('DFleet endpoint has changed', status_code=status.HTTP_502_BAD_GATEWAY)
            elif r.ok:
                data = json.loads(r.content)
                serializer.validated_data['id'] = None
                serializer.validated_data['fms_task_id'] = data['id']
                serializer.validated_data['status'] = Task.Status[data['status'].capitalize()]
                serializer.validated_data['progress'] = data['progress']
                serializer.validated_data['sequence'] = data['sequence']
                serializer.validated_data['owner'] = None
            else:
                detail = json.loads(r.content)
                self._custom_error(detail, status_code=r.status_code)

        except requests.exceptions.ConnectionError:
            self._custom_error('Error connecting to DFleet', status_code=status.HTTP_502_BAD_GATEWAY)

    @action(methods=['post'], detail=True, permission_classes=(permissions.AllowAny,))
    def abort(self, request, *args, **kwargs):
        self.queryset = Task.objects.running()
        instance = self.get_object()

        if not (self.request.user.has_perm('system.abort_task') or
                self.request.user.has_perm('system.abort_own_task') and (
                instance.owner_id == self.user_instance.id or
                self.user_instance_superset and
                instance.owner_id == self.user_instance_superset.id)):
            self.permission_denied(self.request)

        if self._is_fms_mode():
            self._fms_abort(instance)

        instance.abort()

        success = False
        error_msg = ''
        try:
            abort_task = rospy.ServiceProxy(self.agv05_executor + '/abort_task', agv05_executive_msgs.srv.TriggerTask, persistent=False)
            res = abort_task(instance.id)
            if res.success:
                success = True
            error_msg = res.message
        except Exception as ex:
            error_msg = force_text(ex)

        if not success:
            return Response({'result': False, 'message': 'Error: %s' % error_msg})
        return Response({'result': True, 'message': error_msg})

    def _fms_abort(self, instance):
        try:
            endpoint, headers = self._get_fms_endpoint_and_headers()
        except Exception:
            return
        else:
            endpoint = '%s/%s/abort' % (endpoint, instance.fms_task_id)

        try:
            r = requests.post(endpoint, headers=headers, timeout=(3.05, 21), allow_redirects=False, verify=False)
            if r.is_redirect:
                self._custom_error('DFleet endpoint has changed', status_code=status.HTTP_502_BAD_GATEWAY)
            elif not r.ok and r.status_code != status.HTTP_404_NOT_FOUND:
                detail = json.loads(r.content)
                self._custom_error(detail, status_code=r.status_code)

        except requests.exceptions.ConnectionError:
            return

    @action(methods=['post'], detail=True, permission_classes=(permissions.AllowAny,))
    def cancel(self, request, *args, **kwargs):
        if self._is_fms_mode():
            success, error_msg = self._fms_cancel()
        else:
            success, error_msg = self._cancel()

        if not success:
            return Response({'result': False, 'message': 'Error: %s' % error_msg})
        return Response({'result': True, 'message': error_msg})

    def _cancel(self):
        self.queryset = Task.objects.running()
        instance = self.get_object()

        if not (self.request.user.has_perm('system.cancel_task') or
                self.request.user.has_perm('system.cancel_own_task') and (
                instance.owner_id == self.user_instance.id or
                self.user_instance_superset and
                instance.owner_id == self.user_instance_superset.id)):
            self.permission_denied(self.request)

        instance.cancel()

        success = False
        error_msg = ''
        try:
            cancel_task = rospy.ServiceProxy(self.agv05_executor + '/change_task', agv05_executive_msgs.srv.TriggerTask, persistent=False)
            res = cancel_task(instance.id)
            if res.success:
                success = True
            error_msg = res.message
        except Exception as ex:
            error_msg = force_text(ex)

        return success, error_msg

    def _fms_cancel(self):
        if not self.request.user.has_perm('system.cancel_task'):
            self.permission_denied(self.request)

        serializer = FmsTaskSerializer(data=self.request.data)
        serializer.is_valid(raise_exception=True)

        try:
            endpoint, headers = self._get_fms_endpoint_and_headers()
        except Exception:
            self._custom_error('Error pairing with DFleet', status_code=status.HTTP_502_BAD_GATEWAY)
        else:
            endpoint = '%s/%s/cancel' % (endpoint, serializer.validated_data['fms_task_id'])

        try:
            r = requests.post(endpoint, headers=headers, timeout=(3.05, 21), allow_redirects=False, verify=False)
            if r.is_redirect:
                self._custom_error('DFleet endpoint has changed', status_code=status.HTTP_502_BAD_GATEWAY)
            elif not r.ok:
                detail = json.loads(r.content)
                self._custom_error(detail, status_code=r.status_code)

        except requests.exceptions.ConnectionError:
            self._custom_error('Error connecting to DFleet', status_code=status.HTTP_502_BAD_GATEWAY)

        return True, ''

    @action(methods=['post'], detail=True, permission_classes=(permissions.AllowAny,))
    def prioritize(self, request, *args, **kwargs):
        if self._is_fms_mode():
            success, error_msg = self._fms_prioritize()
        else:
            success, error_msg = self._prioritize()

        if not success:
            return Response({'result': False, 'message': 'Error: %s' % error_msg})
        return Response({'result': True, 'message': error_msg})

    def _prioritize(self):
        self.queryset = Task.objects.running()
        instance = self.get_object()

        if not (self.request.user.has_perm('system.prioritize_task') or
                self.request.user.has_perm('system.prioritize_own_task') and (
                instance.owner_id == self.user_instance.id or
                self.user_instance_superset and
                instance.owner_id == self.user_instance_superset.id)):
            self.permission_denied(self.request)

        instance.prioritize()

        success = False
        error_msg = ''
        try:
            prioritize_task = rospy.ServiceProxy(self.agv05_executor + '/change_task', agv05_executive_msgs.srv.TriggerTask, persistent=False)
            res = prioritize_task(instance.id)
            if res.success:
                success = True
            error_msg = res.message
        except Exception as ex:
            error_msg = force_text(ex)

        return success, error_msg

    def _fms_prioritize(self):
        if not self.request.user.has_perm('system.prioritize_task'):
            self.permission_denied(self.request)

        serializer = FmsTaskSerializer(data=self.request.data)
        serializer.is_valid(raise_exception=True)

        try:
            endpoint, headers = self._get_fms_endpoint_and_headers()
        except Exception:
            self._custom_error('Error pairing with DFleet', status_code=status.HTTP_502_BAD_GATEWAY)
        else:
            endpoint = '%s/%s/prioritize' % (endpoint, serializer.validated_data['fms_task_id'])

        try:
            r = requests.post(endpoint, headers=headers, timeout=(3.05, 21), allow_redirects=False, verify=False)
            if r.is_redirect:
                self._custom_error('DFleet endpoint has changed', status_code=status.HTTP_502_BAD_GATEWAY)
            elif not r.ok:
                detail = json.loads(r.content)
                self._custom_error(detail, status_code=r.status_code)

        except requests.exceptions.ConnectionError:
            self._custom_error('Error connecting to DFleet', status_code=status.HTTP_502_BAD_GATEWAY)

        return True, ''

    @action(methods=['post'], detail=True, permission_classes=(permissions.AllowAny,))
    def suspend(self, request, *args, **kwargs):
        if self._is_fms_mode():
            success, error_msg = self._fms_suspend()
        else:
            success, error_msg = self._suspend()

        if not success:
            return Response({'result': False, 'message': 'Error: %s' % error_msg})
        return Response({'result': True, 'message': error_msg})

    def _suspend(self):
        self.queryset = Task.objects.running()
        instance = self.get_object()

        if not (self.request.user.has_perm('system.suspend_task') or
                self.request.user.has_perm('system.suspend_own_task') and (
                instance.owner_id == self.user_instance.id or
                self.user_instance_superset and
                instance.owner_id == self.user_instance_superset.id)):
            self.permission_denied(self.request)

        instance.suspend()

        success = False
        error_msg = ''
        try:
            suspend_task = rospy.ServiceProxy(self.agv05_executor + '/change_task', agv05_executive_msgs.srv.TriggerTask, persistent=False)
            res = suspend_task(instance.id)
            if res.success:
                success = True
            error_msg = res.message
        except Exception as ex:
            error_msg = force_text(ex)

        return success, error_msg

    def _fms_suspend(self):
        if not self.request.user.has_perm('system.suspend_task'):
            self.permission_denied(self.request)

        serializer = FmsTaskSerializer(data=self.request.data)
        serializer.is_valid(raise_exception=True)

        try:
            endpoint, headers = self._get_fms_endpoint_and_headers()
        except Exception:
            self._custom_error('Error pairing with DFleet', status_code=status.HTTP_502_BAD_GATEWAY)
        else:
            endpoint = '%s/%s/suspend' % (endpoint, serializer.validated_data['fms_task_id'])

        try:
            r = requests.post(endpoint, headers=headers, timeout=(3.05, 21), allow_redirects=False, verify=False)
            if r.is_redirect:
                self._custom_error('DFleet endpoint has changed', status_code=status.HTTP_502_BAD_GATEWAY)
            elif not r.ok:
                detail = json.loads(r.content)
                self._custom_error(detail, status_code=r.status_code)

        except requests.exceptions.ConnectionError:
            self._custom_error('Error connecting to DFleet', status_code=status.HTTP_502_BAD_GATEWAY)

        return True, ''

    @action(methods=['post'], detail=True, permission_classes=(permissions.AllowAny,))
    def resume(self, request, *args, **kwargs):
        if self._is_fms_mode():
            success, error_msg = self._fms_resume()
        else:
            success, error_msg = self._resume()

        if not success:
            return Response({'result': False, 'message': 'Error: %s' % error_msg})
        return Response({'result': True, 'message': error_msg})

    def _resume(self):
        self.queryset = Task.objects.running()
        instance = self.get_object()

        if not (self.request.user.has_perm('system.resume_task') or
                self.request.user.has_perm('system.resume_own_task') and (
                instance.owner_id == self.user_instance.id or
                self.user_instance_superset and
                instance.owner_id == self.user_instance_superset.id)):
            self.permission_denied(self.request)

        instance.resume()

        success = False
        error_msg = ''
        try:
            resume_task = rospy.ServiceProxy(self.agv05_executor + '/create_task', agv05_executive_msgs.srv.TriggerTask, persistent=False)
            res = resume_task(instance.id)
            if res.success:
                success = True
            error_msg = res.message
        except Exception as ex:
            error_msg = force_text(ex)

        return success, error_msg

    def _fms_resume(self):
        if not self.request.user.has_perm('system.resume_task'):
            self.permission_denied(self.request)

        serializer = FmsTaskSerializer(data=self.request.data)
        serializer.is_valid(raise_exception=True)

        try:
            endpoint, headers = self._get_fms_endpoint_and_headers()
        except Exception:
            self._custom_error('Error pairing with DFleet.', status_code=status.HTTP_502_BAD_GATEWAY)
        else:
            endpoint = '%s/%s/resume' % (endpoint, serializer.validated_data['fms_task_id'])

        try:
            r = requests.post(endpoint, headers=headers, timeout=(3.05, 21), allow_redirects=False, verify=False)
            if r.is_redirect:
                self._custom_error('DFleet endpoint has changed', status_code=status.HTTP_502_BAD_GATEWAY)
            elif not r.ok:
                detail = json.loads(r.content)
                self._custom_error(detail, status_code=r.status_code)

        except requests.exceptions.ConnectionError:
            self._custom_error('Error connecting to DFleet', status_code=status.HTTP_502_BAD_GATEWAY)

        return True, ''

    def _is_fms_mode(self):
        return Cache.get_executor_mode() == ExecutorMode.DFleet.value

    def _get_fms_endpoint_and_headers(self):
        agv_uuid = Cache.get_agv_uuid()
        fms_metadata = Cache.get_fms_metadata()

        endpoint = fms_metadata['task_endpoint']
        headers = {
            'Authorization': 'AgvToken %s:%s' % (agv_uuid, fms_metadata['token']),
            'Content-Type': 'application/json',
        }
        return endpoint, headers
