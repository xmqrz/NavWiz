from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, ExecutorMode
from django.contrib.auth import get_user_model
from rest_framework import permissions, status, viewsets
from rest_framework.decorators import action
from rest_framework.response import Response
import requests
import ujson as json

from ..serializers import TransactionSerializer, TransactionResumeSerializer
from .mixin import CustomErrorMixin


User = get_user_model()


class TransactionViewSet(CustomErrorMixin, viewsets.ViewSet):
    permission_classes = (permissions.IsAuthenticated,)
    serializer_class = TransactionSerializer

    def check_permissions(self, request):
        super(TransactionViewSet, self).check_permissions(request)
        self.user_instance = request.user if request.user.is_authenticated \
            else User.objects.get(username='Guest')
        self.user_instance_superset = User.objects.get(username='agv_panel') \
            if request.user.username == 'agv_panel_pin_protected' else None

    @action(methods=['post'], detail=True, permission_classes=(permissions.AllowAny,))
    def abort(self, request, *args, **kwargs):
        if not self.request.user.has_perm('system.abort_transaction'):
            self.permission_denied(self.request)

        if not self._is_fms_mode():
            self._custom_error('Not in DFleet mode.', status_code=status.HTTP_404_NOT_FOUND)
        transaction_id = kwargs['pk']

        try:
            endpoint, headers = self._get_fms_endpoint_and_headers()
        except Exception:
            return
        else:
            endpoint = '%s/%s/abort' % (endpoint, transaction_id)

        try:
            r = requests.post(endpoint, headers=headers, timeout=(3.05, 21), allow_redirects=False, verify=False)
            if r.is_redirect:
                self._custom_error('DFleet endpoint has changed', status_code=status.HTTP_502_BAD_GATEWAY)
            elif not r.ok and r.status_code != status.HTTP_404_NOT_FOUND:
                detail = json.loads(r.content)
                self._custom_error(detail, status_code=r.status_code)

        except requests.exceptions.ConnectionError:
            self._custom_error('Error connecting to DFleet', status_code=status.HTTP_502_BAD_GATEWAY)

        return Response({'result': True, 'message': ''})

    @action(methods=['post'], detail=True, permission_classes=(permissions.AllowAny,))
    def cancel(self, request, *args, **kwargs):
        if not self.request.user.has_perm('system.cancel_transaction'):
            self.permission_denied(self.request)

        if not self._is_fms_mode():
            self._custom_error('Not in DFleet mode.', status_code=status.HTTP_404_NOT_FOUND)
        transaction_id = kwargs['pk']

        try:
            endpoint, headers = self._get_fms_endpoint_and_headers()
        except Exception:
            return
        else:
            endpoint = '%s/%s/cancel' % (endpoint, transaction_id)

        try:
            r = requests.post(endpoint, headers=headers, timeout=(3.05, 21), allow_redirects=False, verify=False)
            if r.is_redirect:
                self._custom_error('DFleet endpoint has changed', status_code=status.HTTP_502_BAD_GATEWAY)
            elif not r.ok and r.status_code != status.HTTP_404_NOT_FOUND:
                detail = json.loads(r.content)
                self._custom_error(detail, status_code=r.status_code)

        except requests.exceptions.ConnectionError:
            self._custom_error('Error connecting to DFleet', status_code=status.HTTP_502_BAD_GATEWAY)

        return Response({'result': True, 'message': ''})

    @action(methods=['post'], detail=True, permission_classes=(permissions.AllowAny,))
    def resume(self, request, *args, **kwargs):
        if not self.request.user.has_perm('system.resume_transaction'):
            self.permission_denied(self.request)

        if not self._is_fms_mode():
            self._custom_error('Not in DFleet mode.', status_code=status.HTTP_404_NOT_FOUND)
        transaction_id = kwargs['pk']

        try:
            endpoint, headers = self._get_fms_endpoint_and_headers()
        except Exception:
            return
        else:
            endpoint = '%s/%s/resume' % (endpoint, transaction_id)

        serializer = TransactionResumeSerializer(data=request.data)

        if not serializer.is_valid():
            return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)

        data = json.dumps(serializer.validated_data)

        try:
            r = requests.post(endpoint, headers=headers, data=data,
                              timeout=(3.05, 21), allow_redirects=False, verify=False)
            if r.is_redirect:
                self._custom_error('DFleet endpoint has changed', status_code=status.HTTP_502_BAD_GATEWAY)
            elif not r.ok and r.status_code != status.HTTP_404_NOT_FOUND:
                detail = json.loads(r.content)
                self._custom_error(detail, status_code=r.status_code)

        except requests.exceptions.ConnectionError:
            self._custom_error('Error connecting to DFleet', status_code=status.HTTP_502_BAD_GATEWAY)

        return Response({'result': True, 'message': ''})

    def _get_fms_endpoint_and_headers(self):
        agv_uuid = Cache.get_agv_uuid()
        fms_metadata = Cache.get_fms_metadata()

        endpoint = fms_metadata['transaction_endpoint']
        headers = {
            'Authorization': 'AgvToken %s:%s' % (agv_uuid, fms_metadata['token']),
            'Content-Type': 'application/json',
        }
        return endpoint, headers

    def _is_fms_mode(self):
        return Cache.get_executor_mode() == ExecutorMode.DFleet.value
