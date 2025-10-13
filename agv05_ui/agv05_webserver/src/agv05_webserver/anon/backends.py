"""
Model backend that enables permissions for AnonymousUsers.

I wanted it to be as simple as possible so anonymous users just forward their permission checks
to some fixed group model.

To control which group will represent anonymous user you use ANONYMOUS_GROUP_NAME setting in
settings file.

You need to enable this backend by setting AUTHENTICATION_BACKENDS. Append this backend after
Django's default 'django.contrib.auth.ModelBackend'.

Reference:
- https://djangosnippets.org/snippets/2594/
- https://github.com/django-guardian/django-guardian/blob/devel/guardian/backends.py
"""

from __future__ import unicode_literals

from django.conf import settings as django_settings
from django.contrib.auth import get_user_model
from django.contrib.auth.models import Group, Permission


def get_anon_group_name():
    return getattr(django_settings, 'ANONYMOUS_GROUP_NAME', 'Guest')


def get_anon_user_name():
    return getattr(django_settings, 'ANONYMOUS_USER_NAME', 'Guest')


class AnonymousUserBackend(object):

    # required method in auth backend
    def authenticate(self, username=None, password=None):
        return None

    # required method in auth backend
    def get_user(self, user_id):
        return None

    def get_user_permissions(self, user_obj, obj=None):
        if not user_obj.is_anonymous or obj is not None:
            return set()
        if not hasattr(user_obj, '_user_perm_cache'):
            User = get_user_model()
            anon_user_name = get_anon_user_name()
            try:
                anon_user = User.objects.get(username=anon_user_name)
            except User.DoesNotExist:
                anon_user = User(username=anon_user_name)
                anon_user.set_unusable_password()
                anon_user.save()
            perms = Permission.objects.filter(user=anon_user)
            perms = perms.values_list('content_type__app_label', 'codename').order_by()
            user_obj._user_perm_cache = set('%s.%s' % (ct, name) for ct, name in perms)
        return user_obj._user_perm_cache

    def get_group_permissions(self, user_obj, obj=None):
        if not user_obj.is_anonymous or obj is not None:
            return set()
        if not hasattr(user_obj, '_group_perm_cache'):
            anon_group_name = get_anon_group_name()
            anon_group, created = Group.objects.get_or_create(name=anon_group_name)
            perms = Permission.objects.filter(group=anon_group)
            perms = perms.values_list('content_type__app_label', 'codename').order_by()
            user_obj._group_perm_cache = set('%s.%s' % (ct, name) for ct, name in perms)
        return user_obj._group_perm_cache

    def get_all_permissions(self, user_obj, obj=None):
        if not user_obj.is_anonymous or obj is not None:
            return set()
        if not hasattr(user_obj, '_perm_cache'):
            user_obj._perm_cache = self.get_user_permissions(user_obj)
            user_obj._perm_cache.update(self.get_group_permissions(user_obj))
        return user_obj._perm_cache

    def has_perm(self, user_obj, perm, obj=None):
        if not user_obj.is_active and not user_obj.is_anonymous:
            return False
        return perm in self.get_all_permissions(user_obj)

    def has_module_perm(self, user_obj, app_label):
        if not user_obj.is_active and not user_obj.is_anonymous:
            return False
        for perm in self.get_all_permissions(user_obj):
            if perm[:perm.index('.')] == app_label:
                return True
        return False
