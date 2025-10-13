# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.contrib.auth.management import create_permissions
from django.contrib.contenttypes.management import create_contenttypes
from django.db import models, migrations


def create_panel_user_and_feature_group(apps, schema_editor):
    User = apps.get_model('auth', 'User')
    Group = apps.get_model('auth', 'Group')
    Permission = apps.get_model('auth', 'Permission')
    auth = apps.get_app_config('auth')
    app = apps.get_app_config('app')

    auth.models_module = True  # a trick to get the next two function executed.
    create_contenttypes(auth, verbosity=0)
    create_permissions(auth, verbosity=0)

    app.models_module = True  # a trick to get the next two function executed.
    create_contenttypes(app, verbosity=0)
    create_permissions(app, verbosity=0)

    user_agv_panel, created = User.objects.get_or_create(username='agv_panel')
    user_agv_panel_pp, created = User.objects.get_or_create(username='agv_panel_pin_protected')

    group_anon, created = Group.objects.get_or_create(name='Guest')
    group_user, created = Group.objects.get_or_create(name='User')
    group_sv, created = Group.objects.get_or_create(name='Supervisor')

    # Permission objects
    start_agv05 = Permission.objects.get(content_type__app_label='app', codename='start_agv05')
    stop_agv05 = Permission.objects.get(content_type__app_label='app', codename='stop_agv05')
    soft_reboot = Permission.objects.get(content_type__app_label='app', codename='soft_reboot')
    hard_reboot = Permission.objects.get(content_type__app_label='app', codename='hard_reboot')
    poweroff = Permission.objects.get(content_type__app_label='app', codename='poweroff')
    show_system_icon = Permission.objects.get(content_type__app_label='app', codename='show_system_icon')

    user_agv_panel.user_permissions.add(soft_reboot, hard_reboot, poweroff)
    user_agv_panel_pp.user_permissions.add(start_agv05, stop_agv05, soft_reboot, hard_reboot, poweroff)

    group_anon.permissions.add(show_system_icon)
    group_user.permissions.add(start_agv05, stop_agv05, soft_reboot, hard_reboot, poweroff, show_system_icon)
    group_sv.permissions.add(start_agv05, stop_agv05, soft_reboot, hard_reboot, poweroff, show_system_icon)


class Migration(migrations.Migration):

    dependencies = [
        ('app', '0001_initial'),
        ('auth', '0007_alter_validators_add_error_messages'),
    ]

    operations = [
        migrations.RunPython(create_panel_user_and_feature_group),
    ]
