from __future__ import unicode_literals

from django.contrib.auth.hashers import check_password, make_password
from django.db import models
from six import python_2_unicode_compatible


# Create your models here.

@python_2_unicode_compatible
class Variable(models.Model):
    name = models.CharField(max_length=127, primary_key=True)
    value = models.TextField(blank=True, default='')
    created = models.DateTimeField(auto_now_add=True)
    modified = models.DateTimeField(auto_now=True)

    class Meta:
        # extra permissions
        permissions = (
            # boot operations
            ('start_agv05', 'Can start AGV05 controller'),
            ('stop_agv05', 'Can stop AGV05 controller'),
            ('soft_reboot', 'Can perform soft reboot'),
            ('hard_reboot', 'Can perform hard reboot'),
            ('hot_reload', 'Can perform hot reload'),
            ('poweroff', 'Can perform poweroff'),
            # panel
            ('start_task_runner', 'Can start Task Runner'),
            ('stop_task_runner', 'Can stop Task Runner'),
            ('pause_task_runner', 'Can pause Task Runner'),
            ('resume_task_runner', 'Can resume Task Runner'),
            ('start_app', 'Can start app'),
            ('stop_app', 'Can stop app'),
            ('show_panel_all_modules', 'Show all modules on panel'),
            ('show_panel_call_buttons', 'Show call buttons on panel'),
            ('show_panel_completed_tasks', 'Show completed tasks on panel'),
            ('show_panel_health_status', 'Show health status on panel'),
            ('show_panel_live_monitor', 'Show live monitor tools on panel'),
            ('show_panel_live_manipulation', 'Show live manipulation tools on panel'),
            ('show_panel_live_camera', 'Show live camera on panel'),
            ('show_panel_live_map', 'Show live map on panel'),
            ('show_panel_running_tasks', 'Show running tasks on panel'),
            ('show_panel_side_menu', 'Show side menu on panel'),
            ('show_system_clock', 'Show system clock'),
            ('show_system_icon', 'Show link to ConfigPanel'),
            ('show_popup_safety', 'Show safety message popup'),
            ('show_popup_user_own', 'Show own\'s user message popup'),
            ('show_popup_user_public', 'Show public user message popup'),
            ('show_popup_user_private', 'Show private user message popup'),
            # wifi
            ('wifi_adhoc', 'Can on and off adhoc'),
            ('wifi_connect', 'Can connect and disconnect wifi'),
        )

    def __str__(self):
        return self.name


# AGV panel pin

def set_agv_panel_pin(pin):
    Variable.objects.update_or_create(name='agv_panel_pin', defaults={
        'value': make_password(pin)
    })


def check_agv_panel_pin(pin):
    variable, created = Variable.objects.get_or_create(name='agv_panel_pin', defaults={
        'value': make_password('123456')
    })
    return check_password(pin, variable.value)
