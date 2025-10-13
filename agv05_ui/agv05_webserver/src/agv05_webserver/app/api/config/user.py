from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.app.models import set_agv_panel_pin
from collections import OrderedDict
from django.contrib.auth import get_user_model, update_session_auth_hash
from django.contrib.auth.models import Group
from itertools import chain
from rest_framework import permissions, status, viewsets, serializers
from rest_framework.authtoken.models import Token
from rest_framework.decorators import action
from rest_framework.response import Response
from rest_framework.reverse import reverse

from ...serializers import (
    UserListSerializer,
    UserSerializer,
    UserUpdateSerializer,
    UserPassUpdateSerializer,
    ProtectionPinSerializer,
)
from ..mixin import Permission
from .license_void import LicenseVoidMixin


User = get_user_model()


def own_get_queryset():
    return User.objects.exclude(
        username__in=['Guest', 'agv_panel', 'agv_panel_pin_protected']
    ).order_by('-is_active', 'username', 'id')


class UserOwnProtectedMixin(object):

    def get_queryset(self):
        object_list = super(UserOwnProtectedMixin, self).get_queryset()
        return object_list.exclude(pk=self.request.user.pk)


class UserConfigViewSet(LicenseVoidMixin, UserOwnProtectedMixin, viewsets.ModelViewSet):
    permission_classes = (Permission('system.view_system_panel', 'system.view_users'), permissions.DjangoModelPermissions)
    queryset = User.objects.filter(is_staff=False).exclude(
        username__in=['Guest', 'agv_panel', 'agv_panel_pin_protected']
    ).order_by('-is_active', 'username', 'id')
    serializer_class = UserSerializer
    update_serializer_class = UserUpdateSerializer
    list_serializer_class = UserListSerializer

    def get_serializer_class(self):
        if self.action == 'list':
            return self.list_serializer_class
        if self.action == 'update':
            return self.update_serializer_class
        return super(UserConfigViewSet, self).get_serializer_class()

    def list(self, request, *args, **kwargs):
        response = super(UserConfigViewSet, self).list(request, *args, **kwargs)
        return Response(OrderedDict(chain([
            ('available-groups', reverse('app:config-api:users-available-groups', request=request)),
        ], response.data.items())))

    @action(
        detail=True,
        methods=['POST'],
        url_path='change-password',
        serializer_class=UserPassUpdateSerializer
    )
    def change_password(self, request, *args, **kwargs):
        instance = self.get_object()
        serializer = self.get_serializer(instance, data=request.data)
        serializer.is_valid(raise_exception=True)
        self.perform_update(serializer)

        return Response(serializer.data)

    @change_password.mapping.get
    def get_change_password(self, request, *args, **kwargs):
        return super(UserConfigViewSet, self).retrieve(request, *args, **kwargs)

    @action(
        detail=False,
        methods=['POST'],
        url_path='change-password',
        serializer_class=UserPassUpdateSerializer,
        permission_classes=Permission('system.change_own_password'),
        get_queryset=own_get_queryset
    )
    def change_own_password(self, request, *args, **kwargs):
        instance = request.user
        serializer = self.get_serializer(instance, data=request.data)
        serializer.is_valid(raise_exception=True)
        self.perform_update(serializer)
        update_session_auth_hash(request, serializer.instance)

        return Response(serializer.data)

    @change_own_password.mapping.get
    def get_change_own_password(self, request, *args, **kwargs):
        self.kwargs['pk'] = request.user.pk
        return super(UserConfigViewSet, self).retrieve(request, *args, **kwargs)

    @action(
        detail=False,
        methods=['POST'],
        serializer_class=ProtectionPinSerializer,
        url_path="change-protection-pin"
    )
    def change_protection_pin(self, request):
        serializer = self.get_serializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        try:
            set_agv_panel_pin(serializer.validated_data['pin'])
            return Response({'result': True})
        except Exception as e:
            return Response({"error": "Failed to change protection pin."}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)

    @action(detail=False, methods=['GET'], url_path="available-groups")
    def available_groups(self, request, *args, **kwargs):
        return Response(Group.objects.exclude(name='Guest').values('id', 'name'))

    @action(detail=True, methods=['POST'], url_path="auth-token", serializer_class=serializers.Serializer)
    def auth_token(self, request, pk=None):
        user = self.get_object()

        # regenerate token
        try:
            user.auth_token.delete()
            user.auth_token.key = None
            user.auth_token.save()
            token = user.auth_token
        except Exception:
            token, _ = Token.objects.get_or_create(user=user)

        return Response(token.key)

    @auth_token.mapping.get
    def get_auth_token(self, request, *args, **kwargs):
        token, _ = Token.objects.get_or_create(user=self.get_object())
        return Response(token.key)

    @action(
        detail=False,
        methods=['POST'],
        url_path="auth-token",
        serializer_class=serializers.Serializer,
        permission_classes=Permission('system.change_own_password'),
        get_queryset=own_get_queryset
    )
    def own_auth_token(self, request, pk=None):
        user = request.user

        # regenerate token
        try:
            user.auth_token.delete()
            user.auth_token.key = None
            user.auth_token.save()
            token = user.auth_token
        except Exception:
            token, _ = Token.objects.get_or_create(user=user)

        return Response(token.key)

    @own_auth_token.mapping.get
    def get_own_auth_token(self, request, *args, **kwargs):
        token, _ = Token.objects.get_or_create(user=request.user)
        return Response(token.key)

    def perform_create(self, serializer):
        instance = serializer.save()

    def perform_update(self, serializer):
        instance = serializer.save()

    def perform_destroy(self, instance):
        instance.delete()
