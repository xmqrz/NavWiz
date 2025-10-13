from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from itertools import chain
from django.utils import timezone
from rest_framework import permissions, viewsets, status
from rest_framework.decorators import action
from rest_framework.response import Response
from rest_framework.reverse import reverse

from agv05_webserver.system.models import Map, MapChangeset, MapAnnotation, Variable
from ..mixin import CustomErrorMixin, Permission
from .fms_void import FmsVoidMixin
from .license_void import LicenseVoidMixin
from .mixin import VariableEndpointMixin
from ...serializers import (
    MapEditSerializer,
    MapListSerializer,
    MapLayoutSerializer,
    MapAnnotationSerializer,
    MapActiveSerializer,
    MapParamSerializer,
    VariableSerializer
)


class MapConfigViewSet(CustomErrorMixin, FmsVoidMixin, LicenseVoidMixin, VariableEndpointMixin, viewsets.ModelViewSet):
    permission_classes = (Permission('system.view_system_panel'), permissions.DjangoModelPermissions)
    queryset = Map.objects.all()
    serializer_class = MapEditSerializer
    list_serializer_class = MapListSerializer
    map_changeset = MapChangeset

    def list(self, request, *args, **kwargs):
        response = super(MapConfigViewSet, self).list(request, *args, **kwargs)
        return Response(OrderedDict(chain([
            ('active', reverse('app:config-api:maps-active', request=request)),
            ('teleport', reverse('app:config-api:maps-teleport', request=request)),
            ('transition-trigger', reverse('app:config-api:maps-transition-trigger', request=request)),
            ('param', reverse('app:config-api:maps-param', request=request)),
        ], response.data.items())))

    def get_serializer(self, *args, **kwargs):
        if kwargs.get('many'):
            kwargs.setdefault('context', self.get_serializer_context())
            return self.list_serializer_class(*args, **kwargs)

        return super(MapConfigViewSet, self).get_serializer(*args, **kwargs)

    @action(detail=True, methods=['POST'], serializer_class=MapLayoutSerializer, permission_classes=Permission('system.view_system_panel', 'system.add_mapchangeset'))
    def layout(self, request, *args, **kwargs):
        serializer = self.get_serializer(data=request.data)
        instance = self.get_object()
        serializer.is_valid(raise_exception=True)
        if not serializer.validated_data.get('overwrite'):
            m = self.map_changeset.objects.filter(map=instance).first()
            modified = request.data.get('modified')
            if m is not None and str(modified) != str(m.created):
                self._custom_error('Your changes are not saved. The content has been changed while you are editing it.', status_code=status.HTTP_409_CONFLICT)
        serializer.save(
            map=instance,
            author=request.user,
        )
        return Response(OrderedDict(chain([
            ('name', str(instance)),
            ('id', instance.id),
        ], serializer.data.items())))

    @layout.mapping.get
    def get_layout(self, request, *args, **kwargs):
        instance = self.get_object()
        m = self.map_changeset.objects.filter(map=instance).first()
        if m is None:
            m = self.map_changeset(
                map=instance,
                created=timezone.now() - timezone.timedelta(days=1)
            )
        return Response(OrderedDict(chain([
            ('name', str(instance)),
            ('id', instance.id),
        ], self.get_serializer(m).data.items())))

    @action(detail=True, methods=['POST'], serializer_class=MapAnnotationSerializer, permission_classes=Permission('system.view_system_panel', 'system.add_mapchangeset'))
    def annotation(self, request, *args, **kwargs):
        instance = self.get_object()
        m = MapAnnotation.objects.filter(map=instance).first()
        serializer = self.get_serializer(m, data=request.data)
        serializer.is_valid(raise_exception=True)
        if m is not None and not serializer.validated_data.get('overwrite'):
            modified = serializer.validated_data.get('modified')
            if str(modified) != str(m.modified):
                self._custom_error('Your changes are not saved. The content has been changed while you are editing it.', status_code=status.HTTP_409_CONFLICT)
        serializer.save(
            map=instance,
        )
        return Response(OrderedDict(chain([
            ('name', str(instance)),
            ('id', instance.id),
        ], serializer.data.items())))

    @annotation.mapping.get
    def get_annotation(self, request, *args, **kwargs):
        instance = self.get_object()
        a = MapAnnotation.objects.filter(map=instance).first()
        if a is None:
            a = {
                'modified': timezone.now() - timezone.timedelta(days=1)
            }
        return Response(OrderedDict(chain([
            ('name', str(instance)),
            ('id', instance.id),
        ], self.get_serializer(a).data.items())))

    @action(detail=False, methods=['POST'], serializer_class=MapActiveSerializer, permission_classes=Permission('system.view_system_panel', 'system.change_map'))
    def active(self, *args, **kwargs):
        return self._variable_endpoint(Variable.ACTIVE_MAP, *args, **kwargs)

    @active.mapping.get
    def get_active(self, *args, **kwargs):
        return self._variable_get_endpoint(Variable.ACTIVE_MAP, *args, **kwargs)

    @action(detail=False, methods=['POST'], serializer_class=VariableSerializer, permission_classes=Permission('system.view_system_panel', 'system.change_map'))
    def teleport(self, *args, **kwargs):
        return self._variable_endpoint(Variable.TELEPORT, *args, **kwargs)

    @teleport.mapping.get
    def get_teleport(self, *args, **kwargs):
        return self._variable_get_endpoint(Variable.TELEPORT, *args, **kwargs)

    @action(detail=False, methods=['POST'], serializer_class=VariableSerializer, url_path='transition-trigger', permission_classes=Permission('system.view_system_panel', 'system.change_map'))
    def transition_trigger(self, *args, **kwargs):
        return self._variable_endpoint(Variable.TRANSITION_TRIGGER, *args, **kwargs)

    @transition_trigger.mapping.get
    def get_transition_trigger(self, *args, **kwargs):
        return self._variable_get_endpoint(Variable.TRANSITION_TRIGGER, *args, **kwargs)

    @action(detail=False, methods=['POST'], serializer_class=MapParamSerializer, permission_classes=Permission('system.view_system_panel', 'system.change_map'))
    def param(self, *args, **kwargs):
        return self._variable_endpoint(Variable.MAP_PARAM, *args, **kwargs)

    @param.mapping.get
    def get_param(self, *args, **kwargs):
        return self._variable_get_endpoint(Variable.MAP_PARAM, *args, **kwargs)

    def clone_map(self, instance, from_id):
        changeset = self.map_changeset.objects.filter(map=from_id).first()
        self.map_changeset.objects.create(
            metadata=changeset.metadata,
            structure=changeset.structure,
            stations=changeset.stations,
            map=instance,
            author=self.request.user)
        mapannotation = MapAnnotation.objects.filter(map=from_id).first()
        MapAnnotation.objects.create(
            annotations=mapannotation.annotations,
            map=instance)

    def perform_create(self, serializer):
        instance = serializer.save()
        try:
            from_id = self.request.GET['from']
            self.clone_map(instance, int(from_id))
        except Exception:
            pass
