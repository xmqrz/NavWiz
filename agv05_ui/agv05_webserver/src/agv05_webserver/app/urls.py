from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.conf import settings as django_settings
from django.conf.urls import include, url
from django.views.generic import TemplateView
from rest_framework import routers

from .api import (
    AgvActivitiesYearPagination, AgvViewSet, AppViewSet, AgvStatisticViewSet, BootViewSet, DashboardViewSet,
    LogViewSet, MapViewSet, RegisterViewSet, TaskViewSet, TaskStatisticViewSet,
    TaskCompletedYearPagination, TaskTemplateViewSet, TimeViewSet, TransactionViewSet,
    VariableViewSet, VersionViewSet, WhiteLabelViewSet, WhiteLabelFileDownload, WifiViewSet,
)

from .api.config import (
    AgvActivityConfigViewSet, AgvActivityArchiveConfigViewSet,
    AssemblyInfoConfigViewSet, DiagnosticConfigViewSet, LogConfigViewSet,
    PreventiveMaintenanceConfigViewSet, ServiceLogConfigViewSet, HardwarePluginViewSet,
    AgvConfigViewSet, ConfigViewSet, MapConfigViewSet, MapChangesetConfigViewSet, NetworkViewSet,
    ParameterConfigViewSet, LaserConfigViewSet, AudioConfigViewSet, AudioFileDownloadView,
    RobotConfigViewSet, SearchViewSet, SoftwareUpdateConfigViewSet, SoftwarePatchConfigViewSet, TaskCompletedConfigViewSet,
    TaskCompletedArchiveConfigViewSet, TaskTemplateConfigViewSet, UserConfigViewSet,
    WebhookViewSet, IoConfigViewSet,
    BackupConfigViewSet,
    PermissionConfigViewSet, GroupConfigViewSet, SystemMonitorConfigViewSet, DateTimeConfigViewSet,
    LicenseConfigViewSet
)


router = routers.DefaultRouter(trailing_slash=False)
router.register(r'agv', AgvViewSet, 'agv')
router.register(r'agv-activities-year-pagination', AgvActivitiesYearPagination)
router.register(r'agv-statistics', AgvStatisticViewSet)
router.register(r'apps', AppViewSet, 'app')
router.register(r'boot', BootViewSet, 'boot')
router.register(r'dashboard', DashboardViewSet, 'dashboard')
router.register(r'logs', LogViewSet, 'log')
router.register(r'maps', MapViewSet)
router.register(r'registers', RegisterViewSet, 'register')
router.register(r'tasks', TaskViewSet)
router.register(r'task-statistics', TaskStatisticViewSet)
router.register(r'task-completed-year-pagination', TaskCompletedYearPagination)
router.register(r'task-templates', TaskTemplateViewSet)
router.register(r'time', TimeViewSet, 'time')
router.register(r'transactions', TransactionViewSet, 'transaction')
router.register(r'variables', VariableViewSet, 'variable')
router.register(r'version', VersionViewSet, 'version')
router.register(r'white-label', WhiteLabelViewSet, 'white-label')
router.register(r'wifi', WifiViewSet, 'wifi')


# config api
router.register(r'config', ConfigViewSet, 'config')
config_router = routers.DefaultRouter(trailing_slash=False)
config_router.register(r'agv-activities', AgvActivityConfigViewSet, 'agv-activities')
config_router.register(r'agv-activities-archive', AgvActivityArchiveConfigViewSet, 'agv-activities-archive')
config_router.register(r'assembly-info', AssemblyInfoConfigViewSet, 'assembly-info')
config_router.register(r'date-time', DateTimeConfigViewSet, 'date-time')
config_router.register(r'diagnostics', DiagnosticConfigViewSet, 'diagnostics')
config_router.register(r'log', LogConfigViewSet, 'log')
config_router.register(r'preventive-maintenance', PreventiveMaintenanceConfigViewSet, 'preventive-maintenance')
config_router.register(r'service-logs', ServiceLogConfigViewSet, 'service-logs')
config_router.register(r'hardware-plugin', HardwarePluginViewSet, 'hardware-plugin')
config_router.register(r'agv', AgvConfigViewSet, 'agv')
config_router.register(r'network', NetworkViewSet, 'network')
config_router.register(r'robot-config', RobotConfigViewSet, 'robot-config')
config_router.register(r'search', SearchViewSet, 'search')
config_router.register(r'software-update', SoftwareUpdateConfigViewSet, 'software-update')
config_router.register(r'software-patch', SoftwarePatchConfigViewSet, 'software-patch')
config_router.register(r'task-completed', TaskCompletedConfigViewSet, 'task-completed')
config_router.register(r'task-completed-archive', TaskCompletedArchiveConfigViewSet, 'task-completed-archive')
config_router.register(r'task-templates', TaskTemplateConfigViewSet, 'task-templates')
config_router.register(r'users', UserConfigViewSet, 'users')
config_router.register(r'groups', GroupConfigViewSet, 'groups')
config_router.register(r'webhooks', WebhookViewSet, 'webhooks')
config_router.register(r'parameters', ParameterConfigViewSet, 'parameters')
config_router.register(r'audio', AudioConfigViewSet, 'audio')
config_router.register(r'lasers', LaserConfigViewSet, 'lasers')
config_router.register(r'io', IoConfigViewSet, 'io')
config_router.register(r'system-monitor', SystemMonitorConfigViewSet, 'system-monitor')
config_router.register(r'license', LicenseConfigViewSet, 'license')

if not django_settings.TRACKLESS:
    config_router.register(r'maps', MapConfigViewSet, 'maps')
    config_router.register(r'maps/(?P<map_pk>[0-9]+)/changesets', MapChangesetConfigViewSet, 'map-changesets')
    config_router.register(r'backup', BackupConfigViewSet, 'backup')
    config_router.register(r'permissions', PermissionConfigViewSet, 'permissions')

sub_urlpatterns = OrderedDict()

sub_urlpatterns['api-manual'] = [
    url(
        r'^api/v3/config/audio/download/(?P<field_name>[\w]+)/file/(?P<file_name>.+)$',
        AudioFileDownloadView.as_view(),
        name='audio-download',
    ),
    url(
        r'^api/v3/white-label/download/(?P<file_name>.+)$',
        WhiteLabelFileDownload.as_view(),
        name='white-label-download',
    ),
]

sub_urlpatterns['main'] = [
    url(r'^$', TemplateView.as_view(template_name='app/index.html'), name='index'),
]
sub_urlpatterns['api'] = [
    url(r'^api/v3/config/', include(config_router.urls, namespace='config-api')),
    url(r'^api/v3/', include(router.urls, namespace='api')),
]

urlpatterns = [vv for v in sub_urlpatterns.values() for vv in v]
