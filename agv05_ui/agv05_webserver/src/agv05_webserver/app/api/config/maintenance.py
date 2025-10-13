from __future__ import absolute_import
from __future__ import unicode_literals

from collections import defaultdict
from dateutil.relativedelta import relativedelta
from django.shortcuts import redirect
from django.utils import timezone
from rest_framework import exceptions, viewsets, status
from rest_framework.decorators import action
from rest_framework.renderers import JSONRenderer
from rest_framework.response import Response
import agv05_msgs.srv
import requests
import rospy
import ujson as json

from agv05_webserver.system.models import Variable, gen_totp, val_totp, val_totp2
from ...serializers import AssemblyInfoSerializer, \
    PreventiveMaintenanceSerializer, ServiceLogsSerializer
from ..mixin import Permission


class AssemblyInfoConfigViewSet(viewsets.ViewSet):
    permission_classes = (
        Permission('system.view_system_panel'),
        Permission('system.change_assembly_info', methods='POST'),
    )
    serializer_class = AssemblyInfoSerializer

    variable_edit = [Variable.AGV_MODEL, Variable.SERIAL_NUMBER, Variable.MANUFACTURE_DATE]
    variable_date = [Variable.MANUFACTURE_DATE]

    def list(self, request, *args, **kwargs):
        initial = defaultdict(lambda: None,
            Variable.shared_qs().filter(pk__in=self.variable_edit).values_list('name', 'value'))
        for k in self.variable_date:
            if k in initial:
                d = Variable.parse_date(initial[k])
                if d:
                    initial[k] = d
        initial['mileage'] = int(Variable.get_mileage())
        try:
            parts = json.loads(Variable.shared_qs().get(pk=Variable.AGV_PARTS).value)
            assert isinstance(parts, list)
        except Exception:
            parts = []
        initial['parts'] = parts
        serializer = AssemblyInfoSerializer(initial)
        return Response(serializer.data)

    def create(self, request, *args, **kwargs):
        return self._populate(request.data)

    @action(detail=False, methods=['POST'])
    def sync(self, request, *args, **kwargs):
        endpoint = 'https://hub.dfautomation.com/api/v1/products/by-machine-id/%s?device_key=%s' % gen_totp()
        try:
            r = requests.get(endpoint, timeout=(3.05, 21))
            if r.status_code == 404:
                return Response({'detail': 'Product not found.'}, status=status.HTTP_400_BAD_REQUEST)
            elif r.status_code == 422:
                return Response({'detail': 'Device clock out of sync.'}, status=status.HTTP_400_BAD_REQUEST)
            r.raise_for_status()
        except requests.exceptions.ConnectionError:
            return Response({'detail': 'Cannot connect to DF Hub.'}, status=status.HTTP_400_BAD_REQUEST)
        except Exception:
            return Response({'detail': 'Error response from DF Hub.'}, status=status.HTTP_400_BAD_REQUEST)

        try:
            data = json.loads(r.content)
        except Exception:
            return Response({'detail': 'Invalid response from DF Hub.'}, status=status.HTTP_400_BAD_REQUEST)
        return self._populate(data)

    @action(detail=False, methods=['GET'])
    def link(self, request, *args, **kwargs):
        return redirect('https://hub.dfautomation.com/w/products/m/%s' % gen_totp()[0])

    def _populate(self, data):
        serializer = AssemblyInfoSerializer(data=data)
        serializer.is_valid(raise_exception=True)

        if not val_totp(serializer.validated_data['maker_key']):
            raise exceptions.ValidationError({'maker_key': ['Invalid key.']})

        if not val_totp2(serializer.validated_data['maker_key_2']):
            raise exceptions.ValidationError({'maker_key_2': ['Invalid key.']})

        if serializer.validated_data.get('overwrite_mileage'):
            mileage = serializer.validated_data['mileage']
            try:
                srv = rospy.ServiceProxy('/reset_mileage', agv05_msgs.srv.ResetMileage, persistent=False)
                res = srv(mileage)
            except Exception:
                Variable.set_mileage(mileage)
        else:
            serializer.validated_data['mileage'] = int(Variable.get_mileage())

        for pk in self.variable_edit:
            Variable.shared_qs().update_or_create(pk=pk, defaults={
                'value': serializer.validated_data[pk] if pk not in self.variable_date else serializer.data[pk],
            })

        if 'parts' in serializer.validated_data:
            Variable.shared_qs().update_or_create(pk=Variable.AGV_PARTS, defaults={
                'value': json.dumps(serializer.validated_data['parts']),
            })

        return Response(serializer.data)


class PreventiveMaintenanceConfigViewSet(viewsets.ViewSet):
    permission_classes = (
        Permission('system.view_system_panel'),
        Permission('system.change_preventive_maintenance', methods='POST'),
    )
    serializer_class = PreventiveMaintenanceSerializer

    variable_edit = [Variable.NEXT_PM_DUE, Variable.NEXT_PM_MILEAGE]
    variable_date = [Variable.NEXT_PM_DUE]

    def list(self, request, *args, **kwargs):
        initial = defaultdict(lambda: None,
            Variable.shared_qs().filter(pk__in=self.variable_edit).values_list('name', 'value'))
        for k in self.variable_date:
            if k in initial:
                d = Variable.parse_date(initial[k])
                if d:
                    initial[k] = d
        initial['mileage'] = int(Variable.get_mileage())
        serializer = PreventiveMaintenanceSerializer(initial)
        return Response(serializer.data)

    def create(self, request, *args, **kwargs):
        serializer = PreventiveMaintenanceSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        if not val_totp(serializer.validated_data['maker_key']):
            raise exceptions.ValidationError({'maker_key': ['Invalid key.']})

        today = timezone.localdate()
        next_pm_due = serializer.validated_data[Variable.NEXT_PM_DUE]
        if next_pm_due <= today:
            raise exceptions.ValidationError({Variable.NEXT_PM_DUE: ['Next P.M. Due Date must be after today.']})

        elif next_pm_due > today + relativedelta(months=15):
            raise exceptions.ValidationError({Variable.NEXT_PM_DUE: ['Next P.M. Due Date must not be more than 15 months from now.']})

        mileage = int(Variable.get_mileage())
        if serializer.validated_data[Variable.NEXT_PM_MILEAGE] > mileage + 20000000:
            raise exceptions.ValidationError({Variable.NEXT_PM_MILEAGE: ['Next P.M. Mileage must not be more than 20,000,000m from the current mileage.']})
        serializer.validated_data['mileage'] = mileage

        for pk in self.variable_edit:
            Variable.shared_qs().update_or_create(pk=pk, defaults={
                'value': serializer.validated_data[pk] if pk not in self.variable_date else serializer.data[pk],
            })

        return Response(serializer.data)


class ServiceLogConfigViewSet(viewsets.ViewSet):
    permission_classes = (
        Permission('system.view_system_panel'),
        Permission('system.change_service_log', methods='POST'),
    )
    serializer_class = ServiceLogsSerializer

    def list(self, request, *args, **kwargs):
        try:
            slogs = json.loads(Variable.shared_qs().get(pk=Variable.SERVICE_LOGS).value)
            assert isinstance(slogs, list)
        except Exception:
            slogs = []
        serializer = ServiceLogsSerializer({'service_logs': slogs})
        return Response(serializer.data)

    def create(self, request, *args, **kwargs):
        return self._populate(request.data)

    @action(detail=False, methods=['POST'])
    def sync(self, request, *args, **kwargs):
        endpoint = 'https://hub.dfautomation.com/api/v1/service-logs/by-machine-id/%s?device_key=%s' % gen_totp()
        try:
            r = requests.get(endpoint, timeout=(3.05, 21))
            if r.status_code == 404:
                return Response({'detail': 'Product not found.'}, status=status.HTTP_400_BAD_REQUEST)
            elif r.status_code == 422:
                return Response({'detail': 'Device clock out of sync.'}, status=status.HTTP_400_BAD_REQUEST)
            r.raise_for_status()
        except requests.exceptions.ConnectionError:
            return Response({'detail': 'Cannot connect to DF Hub.'}, status=status.HTTP_400_BAD_REQUEST)
        except Exception:
            return Response({'detail': 'Error response from DF Hub.'}, status=status.HTTP_400_BAD_REQUEST)

        try:
            data = json.loads(r.content)
        except Exception:
            return Response({'detail': 'Invalid response from DF Hub.'}, status=status.HTTP_400_BAD_REQUEST)
        return self._populate(data)

    def _populate(self, data):
        serializer = ServiceLogsSerializer(data=data)
        serializer.is_valid(raise_exception=True)

        if not val_totp(serializer.validated_data['maker_key']):
            raise exceptions.ValidationError({'maker_key': ['Invalid key.']})

        Variable.shared_qs().update_or_create(pk=Variable.SERVICE_LOGS, defaults={
            # handles DateTimeField using DjangoJSONEncoder
            'value': JSONRenderer().render(serializer.validated_data['service_logs'])
        })

        return Response(serializer.data)
