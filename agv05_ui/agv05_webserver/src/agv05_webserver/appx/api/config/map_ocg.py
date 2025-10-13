from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from itertools import chain
from rest_framework import viewsets, status
from rest_framework.decorators import action
from rest_framework.response import Response
from sendfile import sendfile

from agv05_webserver.app.api.mixin import CustomErrorMixin
from agv05_webserver.app.api.config.fms_void import FmsVoidMixin
from agv05_webserver.app.api.config.license_void import LicenseVoidMixin
from agv05_webserver.system.models import Map
from agv05_webserver.systemx.models import MapOcg
from agv05_webserver.app.api.mixin import Permission, AltModelPermissions
from ...serializers import MapOcgListSerializer, MapOcgSerializer


class MapOcgConfigViewSet(FmsVoidMixin, LicenseVoidMixin, CustomErrorMixin, viewsets.ModelViewSet):
    permission_classes = (
        Permission('system.view_system_panel'),
        AltModelPermissions,
        Permission('systemx.change_mapocg', methods='GET'),
    )
    serializer_class = MapOcgSerializer

    def get_serializer(self, *args, **kwargs):
        if kwargs.get('many'):
            kwargs.setdefault('context', self.get_serializer_context())
            return MapOcgListSerializer(*args, **kwargs)

        return super(MapOcgConfigViewSet, self).get_serializer(*args, **kwargs)

    def get_queryset(self):
        pk = self.kwargs.get('map_pk')  # Retrieve the pk from the URL
        return MapOcg.objects.filter(map__pk=pk)

    def _add_map_name(self, response):
        pk = self.kwargs.get('map_pk')  # Retrieve the pk from the URL
        map_name = ''
        try:
            map = Map.objects.get(pk=pk)
            map_name = str(map)
        except Map.DoesNotExist:
            self._custom_error(
                'Map not found.',
                status_code=status.HTTP_404_NOT_FOUND
            )

        return Response(OrderedDict(chain([
            ('map_display_name', map_name),
        ], response.data.items())))

    def list(self, request, *args, **kwargs):
        response = super(MapOcgConfigViewSet, self).list(request, *args, **kwargs)
        return self._add_map_name(response)

    def retrieve(self, request, *args, **kwargs):
        response = super(MapOcgConfigViewSet, self).retrieve(request, *args, **kwargs)
        return self._add_map_name(response)

    @action(detail=True, methods=['GET'])
    def png(self, request, *args, **kwargs):
        instance = self.get_object()

        if not instance.png_file or not instance.png_file.path:
            self._custom_error(
                'File not found.',
                status_code=status.HTTP_404_NOT_FOUND
            )
        return sendfile(self.request, instance.png_file.path, attachment=True)

    def perform_update(self, serializer):
        kwargs = {
            'is_autosaved': False,
        }
        if serializer.validated_data.get('clear_name'):
            kwargs['name'] = None
        instance = serializer.save(**kwargs)

    def perform_create(self, serializer):
        pk = self.kwargs.get('map_pk')  # Retrieve the pk from the URL
        try:
            map = Map.objects.get(pk=pk)
            kwargs = {
                'is_autosaved': False,
                "map": map
            }
            if 'clear_name' in serializer.validated_data:
                del serializer.validated_data['clear_name']
            instance = serializer.save(**kwargs)
        except Map.DoesNotExist:
            self._custom_error(
                'Map not found.',
                status_code=status.HTTP_404_NOT_FOUND
            )
