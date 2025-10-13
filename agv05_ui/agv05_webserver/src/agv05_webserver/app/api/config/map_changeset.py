from __future__ import absolute_import
from __future__ import unicode_literals

from rest_framework import permissions, viewsets

from agv05_webserver.system.models import MapChangeset
from ...serializers import MapChangesetSerializer, MapChangesetSummarySerializer
from ..mixin import Permission
from .license_void import LicenseVoidMixin


class MapChangesetConfigViewSet(LicenseVoidMixin, viewsets.ReadOnlyModelViewSet):
    permission_classes = (Permission('system.view_system_panel'), permissions.DjangoModelPermissions)
    queryset = MapChangeset.objects.all()
    serializer_class = MapChangesetSerializer

    def get_queryset(self):
        pk = self.kwargs.get('map_pk')  # Retrieve the pk from the URL
        return self.queryset.filter(map__pk=pk)

    def get_serializer(self, *args, **kwargs):
        if kwargs.get('many'):
            kwargs.setdefault('context', self.get_serializer_context())
            return MapChangesetSummarySerializer(*args, **kwargs)

        return super(MapChangesetConfigViewSet, self).get_serializer(*args, **kwargs)
