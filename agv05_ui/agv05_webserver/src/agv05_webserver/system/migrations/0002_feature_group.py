# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.contrib.auth.management import create_permissions
from django.contrib.contenttypes.management import create_contenttypes
from django.db import models, migrations


def create_feature_group(apps, schema_editor):
    Group = apps.get_model('auth', 'Group')
    Permission = apps.get_model('auth', 'Permission')
    auth = apps.get_app_config('auth')
    system = apps.get_app_config('system')

    auth.models_module = True  # a trick to get the next two function executed.
    create_contenttypes(auth, verbosity=0)
    create_permissions(auth, verbosity=0)

    system.models_module = True  # a trick to get the next two function executed.
    create_contenttypes(system, verbosity=0)
    create_permissions(system, verbosity=0)

    group_anon, created = Group.objects.get_or_create(name='Guest')
    group_user, created = Group.objects.get_or_create(name='User')
    group_sv, created = Group.objects.get_or_create(name='Supervisor')

    # Permission objects
    change_permission = Permission.objects.get(content_type__app_label='auth', codename='change_permission')
    add_user = Permission.objects.get(content_type__app_label='auth', codename='add_user')
    change_user = Permission.objects.get(content_type__app_label='auth', codename='change_user')
    delete_user = Permission.objects.get(content_type__app_label='auth', codename='delete_user')
    view_users = Permission.objects.get(content_type__app_label='system', codename='view_users')
    add_mapchangeset = Permission.objects.get(content_type__app_label='system', codename='add_mapchangeset')
    change_mapchangeset = Permission.objects.get(content_type__app_label='system', codename='change_mapchangeset')
    delete_mapchangeset = Permission.objects.get(content_type__app_label='system', codename='delete_mapchangeset')
    add_parameter = Permission.objects.get(content_type__app_label='system', codename='add_parameter')
    change_parameter = Permission.objects.get(content_type__app_label='system', codename='change_parameter')
    delete_parameter = Permission.objects.get(content_type__app_label='system', codename='delete_parameter')
    add_tasktemplate = Permission.objects.get(content_type__app_label='system', codename='add_tasktemplate')
    change_tasktemplate = Permission.objects.get(content_type__app_label='system', codename='change_tasktemplate')
    delete_tasktemplate = Permission.objects.get(content_type__app_label='system', codename='delete_tasktemplate')
    backup_restore = Permission.objects.get(content_type__app_label='system', codename='backup_restore')
    view_system_panel = Permission.objects.get(content_type__app_label='system', codename='view_system_panel')

    group_user.permissions.add(view_system_panel)
    group_sv.permissions.add(view_system_panel,
        view_users, add_user, change_user, delete_user, change_permission,
        add_mapchangeset, change_mapchangeset, delete_mapchangeset,
        add_parameter, change_parameter, delete_parameter,
        add_tasktemplate, change_tasktemplate, delete_tasktemplate,
        backup_restore)


class Migration(migrations.Migration):

    dependencies = [
        ('system', '0001_initial'),
        ('auth', '0007_alter_validators_add_error_messages'),
    ]

    operations = [
        migrations.RunPython(create_feature_group),
    ]
