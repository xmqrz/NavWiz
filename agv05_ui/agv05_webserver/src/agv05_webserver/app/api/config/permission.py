from __future__ import absolute_import
from __future__ import unicode_literals

from django.contrib.auth import get_user_model
from django.contrib.auth.models import Group, Permission
from rest_framework import viewsets
from rest_framework.decorators import action
from rest_framework.response import Response

from ...serializers import permissionlistserializer_factory
from ..mixin import CustomErrorMixin, Permission as DRFPermission, NoPaginationMixin
from .license_void import LicenseVoidMixin

User = get_user_model()


class PermissionConfigViewSet(
    NoPaginationMixin,
    CustomErrorMixin,
    LicenseVoidMixin,
    viewsets.ViewSet
):
    permission_classes = DRFPermission('system.view_system_panel', 'auth.change_permission')

    def list(self, request, *args, **kwargs):
        # Note: show detail (similar to retrieve)
        serializer = self.get_serializer(request.data)
        return Response(serializer.to_representation(serializer.get_initial()))

    def get_serializer(self, *args, **kwargs):
        serializer_class = self.generate_serialzier_class()
        serializer = serializer_class(*args, **kwargs)
        return serializer

    def generate_serialzier_class(self, **kwargs):
        change_change_permission = self.request.user.has_perm('system.change_change_permission')
        serializer_class = permissionlistserializer_factory(change_change_permission=change_change_permission, **kwargs)

        return serializer_class

    def create(self, request):
        # Note: perform_update (similar to update method)
        serializer = self.get_serializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        serializer.save()

        return self.list(request)

    @action(detail=False, methods=['POST'])
    def reset(self, request, *args, **kwargs):
        user_agv_panel, created = User.objects.get_or_create(username='agv_panel')
        user_agv_panel_pp, created = User.objects.get_or_create(username='agv_panel_pin_protected')

        group_anon, created = Group.objects.get_or_create(name='Guest')
        group_cb, created = Group.objects.get_or_create(name='Call Button')
        group_user, created = Group.objects.get_or_create(name='User')
        group_sv, created = Group.objects.get_or_create(name='Supervisor')
        group_builtins = Group.objects.builtins()

        default_user_mapping = {
            # agv boot
            'start_agv05': [user_agv_panel_pp],
            'stop_agv05': [user_agv_panel_pp],
            'soft_reboot': [user_agv_panel, user_agv_panel_pp],
            'hard_reboot': [user_agv_panel, user_agv_panel_pp],
            'hot_reload': [],
            'poweroff': [user_agv_panel, user_agv_panel_pp],
            # agv panel
            'start_task_runner': [user_agv_panel, user_agv_panel_pp],
            'stop_task_runner': [user_agv_panel, user_agv_panel_pp],
            'pause_task_runner': [user_agv_panel, user_agv_panel_pp],
            'resume_task_runner': [user_agv_panel, user_agv_panel_pp],
            'start_app': [user_agv_panel, user_agv_panel_pp],
            'stop_app': [user_agv_panel, user_agv_panel_pp],
            'show_panel_side_menu': [user_agv_panel, user_agv_panel_pp],
            'show_panel_call_buttons': [user_agv_panel, user_agv_panel_pp],
            'show_panel_running_tasks': [user_agv_panel, user_agv_panel_pp],
            'show_panel_completed_tasks': [user_agv_panel, user_agv_panel_pp],
            'show_panel_all_modules': [user_agv_panel, user_agv_panel_pp],
            'show_panel_live_map': [user_agv_panel, user_agv_panel_pp],
            'show_panel_live_camera': [],
            'show_panel_health_status': [],
            'show_panel_live_monitor': [user_agv_panel, user_agv_panel_pp],
            'show_panel_live_manipulation': [user_agv_panel, user_agv_panel_pp],
            'show_system_clock': [user_agv_panel, user_agv_panel_pp],
            'show_system_icon': [],
            'show_popup_safety': [user_agv_panel, user_agv_panel_pp],
            'show_popup_user_own': [user_agv_panel, user_agv_panel_pp],
            'show_popup_user_public': [user_agv_panel, user_agv_panel_pp],
            'show_popup_user_private': [user_agv_panel, user_agv_panel_pp],
            # task operation
            'add_task': [user_agv_panel, user_agv_panel_pp],
            'abort_own_task': [user_agv_panel, user_agv_panel_pp],
            'abort_task': [user_agv_panel, user_agv_panel_pp],
            'cancel_own_task': [user_agv_panel, user_agv_panel_pp],
            'cancel_task': [user_agv_panel, user_agv_panel_pp],
            'prioritize_own_task': [user_agv_panel, user_agv_panel_pp],
            'prioritize_task': [user_agv_panel, user_agv_panel_pp],
            'suspend_own_task': [user_agv_panel, user_agv_panel_pp],
            'suspend_task': [user_agv_panel, user_agv_panel_pp],
            'resume_own_task': [user_agv_panel, user_agv_panel_pp],
            'resume_task': [user_agv_panel, user_agv_panel_pp],
            'use_tasktemplate': [],
            # transaction
            'abort_transaction': [user_agv_panel, user_agv_panel_pp],
            'cancel_transaction': [user_agv_panel, user_agv_panel_pp],
            'resume_transaction': [user_agv_panel, user_agv_panel_pp],
            # wifi
            'wifi_connect': [user_agv_panel, user_agv_panel_pp],
            'wifi_adhoc': [user_agv_panel, user_agv_panel_pp],
        }

        default_group_mapping = {
            # agv boot
            'start_agv05': [group_user, group_sv],
            'stop_agv05': [group_user, group_sv],
            'soft_reboot': [group_user, group_sv],
            'hard_reboot': [group_user, group_sv],
            'hot_reload': [group_user, group_sv],
            'poweroff': [group_user, group_sv],
            # agv panel
            'start_task_runner': [group_user, group_sv],
            'stop_task_runner': [group_user, group_sv],
            'pause_task_runner': [group_user, group_sv],
            'resume_task_runner': [group_user, group_sv],
            'start_app': [group_user, group_sv],
            'stop_app': [group_user, group_sv],
            'show_panel_side_menu': [group_user, group_sv],
            'show_panel_call_buttons': [group_cb, group_user, group_sv],
            'show_panel_running_tasks': [group_anon, group_cb, group_user, group_sv],
            'show_panel_completed_tasks': [group_anon, group_cb, group_user, group_sv],
            'show_panel_all_modules': [group_user, group_sv],
            'show_panel_live_map': [group_user, group_sv],
            'show_panel_live_camera': [group_sv],
            'show_panel_health_status': [group_sv],
            'show_panel_live_monitor': [group_user, group_sv],
            'show_panel_live_manipulation': [group_user, group_sv],
            'show_system_clock': [group_anon, group_cb, group_user, group_sv],
            'show_system_icon': [group_anon, group_user, group_sv],
            'show_popup_safety': [group_user, group_sv],
            'show_popup_user_own': [group_cb, group_user, group_sv],
            'show_popup_user_public': [group_user, group_sv],
            'show_popup_user_private': [group_sv],
            # task operation
            'add_task': [group_cb, group_user, group_sv],
            'abort_own_task': [group_cb, group_user, group_sv],
            'abort_task': [group_user, group_sv],
            'cancel_own_task': [group_cb, group_user, group_sv],
            'cancel_task': [group_user, group_sv],
            'prioritize_own_task': [group_user, group_sv],
            'prioritize_task': [group_user, group_sv],
            'suspend_own_task': [group_user, group_sv],
            'suspend_task': [group_user, group_sv],
            'resume_own_task': [group_user, group_sv],
            'resume_task': [group_user, group_sv],
            'use_tasktemplate': [group_sv],
            # transaction
            'abort_transaction': [group_cb, group_user, group_sv],
            'cancel_transaction': [group_cb, group_user, group_sv],
            'resume_transaction': [group_cb, group_user, group_sv],
            # system
            'view_panel': [group_anon, group_cb, group_user, group_sv],
            'view_system_panel': [group_sv],
            'view_agv_activities': [group_user, group_sv],
            'view_completed_tasks': [group_user, group_sv],
            'view_log_files': [group_sv],
            'view_help_content': [group_user, group_sv],
            'change_agv': [group_sv],
            'add_map': [group_sv],
            'change_map': [group_sv],
            'delete_map': [group_sv],
            'add_mapchangeset': [group_sv],
            'change_mapchangeset': [group_sv],
            'change_parameter': [group_sv],
            'change_protected_parameter': [],
            'delete_parameter': [group_sv],
            'add_tasktemplate': [group_sv],
            'change_tasktemplate': [group_sv],
            'delete_tasktemplate': [group_sv],
            'add_webhook': [group_sv],
            'edit_webhook': [group_sv],
            'delete_webhook': [group_sv],
            'backup_restore': [group_sv],
            'change_network': [group_sv],
            'change_datetime': [group_sv],
            'change_agvactivity': [group_sv],
            'change_agv_activities_config': [group_sv],
            'change_license': [group_sv],
            'update_software': [group_sv],
            # user
            'view_users': [group_sv],
            'add_user': [group_sv],
            'change_user': [group_sv],
            'delete_user': [group_sv],
            'change_own_password': [group_user, group_sv],
            'add_group': [group_sv],
            'change_group': [group_sv],
            'delete_group': [group_sv],
            # 'change_permission': [group_sv],  # won't update this permission.
            # wifi
            'wifi_connect': [group_user, group_sv],
            'wifi_adhoc': [group_user, group_sv],
        }

        for perm in Permission.objects.filter(codename__in=default_user_mapping.keys()):
            perm.user_set.set(default_user_mapping.get(perm.codename))

        for perm in Permission.objects.filter(codename__in=default_group_mapping.keys()):
            perm.group_set.remove(*group_builtins)
            perm.group_set.add(*default_group_mapping.get(perm.codename))

        return self.list(request)
