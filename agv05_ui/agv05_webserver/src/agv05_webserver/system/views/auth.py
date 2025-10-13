from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.conf import settings as django_settings
from django.contrib.auth import get_user_model, authenticate, login, logout
from django.utils.decorators import method_decorator
from django.views.decorators.csrf import csrf_protect, ensure_csrf_cookie
from rest_framework import status, views, permissions
from rest_framework.response import Response

User = get_user_model()


class LoginView(views.APIView):
    permission_classes = (permissions.AllowAny,)

    @method_decorator(csrf_protect, name='dispatch')
    def post(self, request, format=None):
        data = request.data

        username = data.get('username', None)
        password = data.get('password', None)

        user = authenticate(username=username, password=password)

        if user is not None:
            if user.is_active:
                login(request, user)
                return Response(get_user_info(user))

        # TODO: python i18n required!!
        return Response({'error': 'Invalid username or password.'}, status=status.HTTP_400_BAD_REQUEST)


class LogoutView(views.APIView):
    permission_classes = (permissions.IsAuthenticated,)

    @method_decorator(csrf_protect, name='dispatch')
    def get(self, request, format=None):
        logout(request)
        user = User.objects.get(username='Guest')
        return Response(get_user_info(user))


class AuthView(views.APIView):
    permission_classes = (permissions.AllowAny,)

    @method_decorator(ensure_csrf_cookie)
    def get(self, request, format=None):
        try:
            user = request.user if request.user.is_authenticated else User.objects.get(username='Guest')
            return Response(get_user_info(user))
        except Exception:
            return Response({'error': 'Error while checking authentication'}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)
        return Response({'status': 'ok'})


def get_user_info(user):
    login = bool(user.is_authenticated) and user.username != 'Guest'
    permissions = get_permissions(user)
    group = user.groups.first()
    username_with_pin = None
    perms_with_pin = set()
    if user.username == 'agv_panel':
        user_with_pin = User.objects.get(username='agv_panel_pin_protected')
        username_with_pin = user_with_pin.username
        perms_with_pin = get_permissions(user_with_pin)

    env = {
        'TRACKLESS': django_settings.TRACKLESS,
        'DYNAMIC_PATH_PLANNING': django_settings.DYNAMIC_PATH_PLANNING,
        'OMNI_DRIVE': django_settings.OMNI_DRIVE,
        'CSRFNAME': django_settings.CSRF_COOKIE_NAME,
    }

    return OrderedDict([
        ('login', login),
        ('is_superuser', user.is_superuser and user.is_active),
        ('username', user.username),
        ('group', group.name if group else None),
        ('permissions', list(permissions)),
        ('username_with_pin', username_with_pin),
        ('permissions_with_pin', list(perms_with_pin)),
        ('env', env)
    ])


def get_permissions(user):
    permissions = None
    if user.is_superuser or not user.is_active:
        return []

    return user.get_all_permissions()
