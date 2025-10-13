from __future__ import absolute_import
from __future__ import unicode_literals

from django.contrib.auth.models import Group
from rest_framework import permissions, viewsets
from rest_framework.decorators import action
from rest_framework.response import Response

from ...serializers import GroupSerializer
from ..mixin import Permission
from .license_void import LicenseVoidMixin


class GroupConfigViewSet(LicenseVoidMixin, viewsets.ModelViewSet):
    permission_classes = (Permission('system.view_system_panel', 'system.view_users'), permissions.DjangoModelPermissions)
    serializer_class = GroupSerializer
    queryset = Group.objects.all()

    @action(detail=False, methods=['GET'], url_path='builtin-groups')
    def builtin_groups(self, request, *args, **kwargs):
        queryset = Group.objects.builtins()
        serializer = GroupSerializer(queryset, many=True)
        return Response(serializer.data)

    def partial_update(self, request, *args, **kwargs):
        try:
            group = self.get_object()
        except Group.DoesNotExist:
            return Response({"error": "Group not found."}, status=404)
        group.name = request.data.get('name')
        group.save()

        serializer = GroupSerializer(group)
        return Response(serializer.data)
