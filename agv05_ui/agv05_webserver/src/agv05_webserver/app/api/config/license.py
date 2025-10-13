from __future__ import absolute_import
from __future__ import unicode_literals

from rest_framework import exceptions, viewsets, status
from rest_framework.decorators import action
from rest_framework.response import Response
import requests
import ujson as json

from agv05_webserver.system.models import Variable, myfp, ret_key, val_lic
from ...serializers import LicenseInfoSerializer, \
    LicenseOfflineSerializer, LicenseOnlineSerializer, LicenseRequestSerializer
from ..mixin import Permission
from .software_update import get_current_version

HUB_ENDPOINT = 'https://hub.dfautomation.com/api/v1/licenses'


def update_license():
    data = {}
    data['fingerprint'] = myfp()[0]
    try:
        data['license_key'] = Variable.shared_qs().get(pk=Variable.LICENSE_KEY).value
    except Exception:
        pass
    data['version'] = get_current_version()

    endpoint = '%s/renewal' % HUB_ENDPOINT
    try:
        r = requests.post(endpoint, json=data, timeout=(3.05, 21))
        r.raise_for_status()
        data = json.loads(r.content)
    except Exception:
        return

    key = data['license_key']
    r = val_lic(key)
    if r.features:
        Variable.shared_qs().update_or_create(pk=Variable.LICENSE_KEY, defaults={'value': key})
        Variable.shared_qs().filter(pk=Variable.LICENSE_RET).delete()

    rv_list = json.dumps(data['revocations'])
    Variable.shared_qs().update_or_create(pk=Variable.LICENSE_RW, defaults={'value': rv_list})


class LicenseConfigViewSet(viewsets.ViewSet):
    permission_classes = Permission('system.view_system_panel', 'system.change_license')
    serializer_class = LicenseOfflineSerializer

    @action(detail=False, permission_classes=Permission('system.view_help_content'))
    def info(self, request, *args, **kwargs):
        serializer = LicenseInfoSerializer(val_lic())
        return Response(dict(serializer.data, navwiz_version=get_current_version()))

    def list(self, request, *args, **kwargs):
        update_license()
        req, machine_id = myfp()
        try:
            ret = Variable.shared_qs().get(pk=Variable.LICENSE_RET).value
        except Exception:
            ret = ''
        serializer = LicenseRequestSerializer({
            'machine_id': machine_id,
            'license_req': req,
            'license_ret': ret,
        })
        return Response(serializer.data)

    def create(self, request, *args, **kwargs):
        serializer = LicenseOfflineSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        key = serializer.validated_data['license_key']
        return self._update_key(key)

    @action(methods=['post'], detail=False, serializer_class=LicenseOnlineSerializer)
    def online(self, request, *args, **kwargs):
        serializer = LicenseOnlineSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        endpoint = '%s/activation' % HUB_ENDPOINT
        data = {
            'activation_key': serializer.validated_data['activation_key'],
            'fingerprint': myfp()[0],
            'version': get_current_version(),
        }

        try:
            r = requests.post(endpoint, json=data, timeout=(3.05, 21))
            if r.status_code == 404:
                return Response({'activation_key': ['Invalid activation key.']}, status=status.HTTP_400_BAD_REQUEST)
            elif r.status_code == 422:
                return Response(json.loads(r.content)['errors'], status=status.HTTP_400_BAD_REQUEST)
            r.raise_for_status()
        except requests.exceptions.ConnectionError:
            return Response({'detail': 'Cannot connect to license server.'}, status=status.HTTP_400_BAD_REQUEST)
        except Exception:
            return Response({'detail': 'Error response from license server.'}, status=status.HTTP_400_BAD_REQUEST)

        try:
            key = json.loads(r.content)['license_key']
        except Exception:
            return Response({'detail': 'Invalid response from license server.'}, status=status.HTTP_400_BAD_REQUEST)
        return self._update_key(key)

    def _update_key(self, key):
        r = val_lic(key)
        if not r.features:
            msg = 'The license key is invalid.'
            raise exceptions.ValidationError({'detail': msg})

        Variable.shared_qs().update_or_create(pk=Variable.LICENSE_KEY, defaults={'value': key})
        Variable.shared_qs().filter(pk=Variable.LICENSE_RET).delete()

        serializer = LicenseInfoSerializer(r)
        return Response(serializer.data)

    @action(methods=['post'], detail=False, url_path='return', url_name='return', serializer_class=None)
    def ret(self, request, *args, **kwargs):
        key = ret_key()
        if not key:
            return Response({'detail': 'License missing.'}, status=status.HTTP_404_NOT_FOUND)

        endpoint = '%s/revocation' % HUB_ENDPOINT
        data = {'revocation_key': key}

        try:
            r = requests.post(endpoint, json=data, timeout=(3.05, 21))
            r.raise_for_status()
        except requests.exceptions.ConnectionError:
            return Response({'detail': 'Cannot connect to license server.'}, status=status.HTTP_400_BAD_REQUEST)
        except Exception:
            return Response({'detail': 'Error response from license server.'}, status=status.HTTP_400_BAD_REQUEST)
        return Response({})
