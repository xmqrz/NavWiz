from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_msgs.msg import HardwareInfo
from agv05_webserver.system.models import Cache, Variable
from capabilities.srv import GetCapabilitySpec
from django.conf import settings as django_settings
from rest_framework import permissions, viewsets
from rest_framework.response import Response
import rospy
import subprocess
try:
    from yaml import full_load
except Exception:
    from yaml import load as full_load

NAVWIZ_HOME = '/var/lib/navwiz/'


def _get_version():
    variable_info = [Variable.AGV_MODEL, Variable.SERIAL_NUMBER,
        Variable.MANUFACTURE_DATE, Variable.NEXT_PM_DUE, Variable.NEXT_PM_MILEAGE]

    info = dict(Variable.shared_qs().filter(pk__in=variable_info).values_list('name', 'value'))

    try:
        next_pm_mileage = float(info[Variable.NEXT_PM_MILEAGE])
    except Exception:
        next_pm_mileage = 0

    mileage = round(Variable.get_mileage())

    return {
        'agv_model': info.get(Variable.AGV_MODEL, '-'),
        'serial_number': info.get(Variable.SERIAL_NUMBER, '-'),
        'manufacture_date': info.get(Variable.MANUFACTURE_DATE, '-'),
        'next_pm_due': info.get(Variable.NEXT_PM_DUE, '2020-08-01'),
        'next_pm_mileage': next_pm_mileage,
        'current_mileage': mileage,
        'system_version': '%s (%s)' % (Cache.get_models_version(),
            ('trackless' if django_settings.TRACKLESS else 'tracked')),
        'plugin_version': '%s' % _get_plugin_version(),
        'hw_info': _get_hw_info(),
    }


def _get_hw_info():
    info = []
    try:
        entries = rospy.wait_for_message('/hardware_info', HardwareInfo, timeout=0.1).entries
        for e in entries:
            info.append((e.name.replace('_', ' ').title(), e.value))
    except Exception:
        pass
    return info


def _get_plugin_version():
    version = None
    try:
        version = subprocess.check_output('cat %s/.navwiz_plugin/*.mount/version 2>/dev/null || true' % NAVWIZ_HOME, shell=True, universal_newlines=True)
    except Exception:
        pass

    if not version:
        version = '%s.x.x' % _get_plugin_api_version()

    return version


def _get_plugin_api_version():
    try:
        get_capability_spec = rospy.ServiceProxy('/capability_server/get_capability_spec', GetCapabilitySpec, persistent=False)
        data = full_load(get_capability_spec('agv05_capabilities/MobileRobot').capability_spec.content)
        return data['version']
    except Exception as ex:
        return 1


class VersionViewSet(viewsets.ViewSet):
    permission_classes = (permissions.AllowAny,)

    # this is actually a `retrieve` operation portrayed as `list` so that
    # it can be included in the router's urls.
    def list(self, request, *args, **kwargs):
        return Response({'info': _get_version()})
