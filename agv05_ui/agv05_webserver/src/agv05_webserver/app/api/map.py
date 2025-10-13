from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, Map
from django.conf import settings as django_settings
from rest_framework import permissions, viewsets
from rest_framework.response import Response
from rest_framework.decorators import action
from rest_framework.reverse import reverse
import base64

from ..serializers import MapSerializer

if django_settings.TRACKLESS:
    from agv05_webserver.systemx.models import Cache


class MapViewSet(viewsets.ReadOnlyModelViewSet):
    queryset = Map.objects.all()
    serializer_class = MapSerializer

    def list(self, request, *args, **kwargs):
        return Response({
            'active': reverse('app:api:map-active', request=request),
        })

    @action(detail=False)
    def active(self, request, *args, **kwargs):
        if not django_settings.TRACKLESS:
            results = [{
                'name': m.get('name', ''),
                'structure': m,
            } for m in Cache.get_map_structure()]

        else:
            results = [{
                'name': m.get('name', ''),
                'structure': m,
                'ocg': {
                    'resolution': ocg.info.resolution,
                    'width': ocg.info.width,
                    'height': ocg.info.height,
                    'x0': ocg.info.origin.position.x,
                    'y0': ocg.info.origin.position.y,
                    'data': ('data:image/png;base64,' +
                        base64.b64encode(ocg.data.tobytes()).decode()),
                },
            } for m, ocg in zip(Cache.get_map_structure(), Cache.get_ocg())]

        return Response({
            'results': results,
        })
