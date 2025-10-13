from __future__ import absolute_import
from __future__ import unicode_literals

from django.shortcuts import redirect
from rest_framework.reverse import reverse
from rest_framework import permissions, viewsets

from .agv import AgvConfigViewSet
from .agv_activity import AgvActivityConfigViewSet
from .agv_activity import AgvActivityConfigViewSet, AgvActivityArchiveConfigViewSet
from .datetime import DateTimeConfigViewSet
from .diagnostic import DiagnosticConfigViewSet
from .log import LogConfigViewSet
from .hardware_plugin import HardwarePluginViewSet
from .maintenance import AssemblyInfoConfigViewSet, PreventiveMaintenanceConfigViewSet, ServiceLogConfigViewSet
from .map import MapConfigViewSet
from .map_changeset import MapChangesetConfigViewSet
from .network import NetworkViewSet
from .parameter import ParameterConfigViewSet
from .audio import AudioConfigViewSet, AudioFileDownloadView
from .laser import LaserConfigViewSet
from .io import IoConfigViewSet
from .robot_config import RobotConfigViewSet
from .search import SearchViewSet
from .software_update import SoftwareUpdateConfigViewSet
from .software_patch import SoftwarePatchConfigViewSet
from .task_completed import TaskCompletedConfigViewSet, TaskCompletedArchiveConfigViewSet
from .task_template import TaskTemplateConfigViewSet
from .user import UserConfigViewSet
from .group import GroupConfigViewSet
from .webhook import WebhookViewSet
from .permission import PermissionConfigViewSet
from .backup_restore import BackupConfigViewSet
from .system_monitor import SystemMonitorConfigViewSet
from .license import LicenseConfigViewSet


class ConfigViewSet(viewsets.ViewSet):
    permission_classes = (permissions.AllowAny,)

    def list(self, request, *args, **kwargs):
        return redirect('%s/' % reverse('app:api:config-list', request=request))
