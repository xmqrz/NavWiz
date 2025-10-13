from __future__ import absolute_import
from __future__ import unicode_literals

from django.http import Http404
from rest_framework.decorators import action

from agv05_webserver.system.models import MapAnnotation
from agv05_webserver.systemx.models import MapOcg, MapChangeset
from agv05_webserver.app.api.mixin import Permission
from agv05_webserver.app.api.config.map import MapConfigViewSet as AppMapConfigViewSet
from ...serializers import (
    MapLayoutSerializer,
    MapListSerializer
)


class MapConfigViewSet(AppMapConfigViewSet):
    map_changeset = MapChangeset
    list_serializer_class = MapListSerializer

    @action(detail=True, methods=['POST'], serializer_class=MapLayoutSerializer, permission_classes=Permission('system.view_system_panel', 'system.add_mapchangeset'))
    def layout(self, *args, **kwargs):
        return super(MapConfigViewSet, self).layout(*args, **kwargs)

    @layout.mapping.get
    def get_layout(self, *args, **kwargs):
        return super(MapConfigViewSet, self).get_layout(*args, **kwargs)

    def clone_map(self, instance, from_id, changeset):
        ocg = changeset.ocg
        clone_ocg = None
        if ocg is not None:
            clone_ocg = MapOcg.objects.create(
                is_autosaved=ocg.is_autosaved,
                metadata=ocg.metadata,
                map=instance)
            clone_ocg.png_file.save('raw.png', ocg.png_file)
        self.map_changeset.objects.create(
            metadata=changeset.metadata,
            structure=changeset.structure,
            stations=changeset.stations,
            map=instance,
            author=self.request.user,
            ocg=clone_ocg)
        mapannotation = MapAnnotation.objects.filter(map=from_id).first()
        MapAnnotation.objects.create(
            annotations=mapannotation.annotations,
            map=instance)

    def perform_create(self, serializer):
        # only allow clone in trackless mode.
        try:
            from_id = int(self.request.GET['from'])
        except Exception:
            raise Http404()
        changeset = self.map_changeset.objects.filter(map=from_id).first()
        if not changeset:
            raise Http404()

        instance = serializer.save()
        try:
            self.clone_map(instance, from_id, changeset)
        except Exception:
            pass
