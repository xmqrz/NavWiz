# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.conf import settings as django_settings
from django.contrib.auth.hashers import make_password
from django.db import migrations


def create_anonymous_user_and_group(apps, schema_editor):
    User = apps.get_model('auth', 'User')
    Group = apps.get_model('auth', 'Group')

    anon_user_name = getattr(django_settings, 'ANONYMOUS_USER_NAME', 'Guest')
    anon_group_name = getattr(django_settings, 'ANONYMOUS_GROUP_NAME', 'Guest')

    # create anonymous group
    anon_group, created = Group.objects.get_or_create(name=anon_group_name)

    # create anonymous user
    anon_user, created = User.objects.get_or_create(username=anon_user_name, defaults={
        'password': make_password(None)
    })
    anon_user.groups.add(anon_group)


class Migration(migrations.Migration):

    dependencies = [
        ('auth', '0007_alter_validators_add_error_messages'),
    ]

    operations = [
        migrations.RunPython(create_anonymous_user_and_group),
    ]
