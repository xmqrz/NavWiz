from __future__ import absolute_import
from __future__ import unicode_literals

from django.contrib.auth.models import Group, Permission
from rest_framework.decorators import action

from agv05_webserver.app.api.config.permission import PermissionConfigViewSet as AppPermissionConfigViewSet


class PermissionConfigViewSet(AppPermissionConfigViewSet):

    def generate_serialzier_class(self):
        return super(PermissionConfigViewSet, self).generate_serialzier_class(system_codenames=[
            'view_panel', 'view_system_panel',
            'view_agv_activities', 'view_completed_tasks', 'view_map_quality',
            'view_log_files', 'view_help_content',
            'change_agv',
            'change_map', 'delete_map',
            'change_mapocg', 'delete_mapocg',
            'add_mapchangeset', 'change_mapchangeset',
            'change_parameter', 'change_protected_parameter', 'delete_parameter',
            'add_tasktemplate', 'change_tasktemplate', 'delete_tasktemplate',
            'add_webhook', 'change_webhook', 'delete_webhook',
            'backup_restore',
            'change_network',
            'change_datetime',
            'change_agvactivity', 'change_agv_activities_config',
            'change_license',
            'update_software',
        ])

    @action(detail=False, methods=['POST'])
    def reset(self, request, *args, **kwargs):
        group_anon, created = Group.objects.get_or_create(name='Guest')
        group_cb, created = Group.objects.get_or_create(name='Call Button')
        group_user, created = Group.objects.get_or_create(name='User')
        group_sv, created = Group.objects.get_or_create(name='Supervisor')
        group_builtins = Group.objects.builtins()

        default_group_mapping = {
            # systemx
            'view_map_quality': [group_sv],
            'change_mapocg': [group_sv],
            'delete_mapocg': [group_sv],
            'add_mapchangeset': [group_sv],
            'change_mapchangeset': [group_sv],
        }

        for perm in Permission.objects.filter(codename__in=default_group_mapping.keys()):
            perm.group_set.remove(*group_builtins)
            perm.group_set.add(*default_group_mapping.get(perm.codename))

        return super(PermissionConfigViewSet, self).reset(request, *args, **kwargs)
