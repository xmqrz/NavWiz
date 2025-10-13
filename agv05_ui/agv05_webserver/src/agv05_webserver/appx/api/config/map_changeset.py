from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.app.api.config.map_changeset import MapChangesetConfigViewSet as AppMapChangesetConfigViewSet
from agv05_webserver.systemx.models import MapChangeset
from ...serializers import MapChangesetSerializer


class MapChangesetConfigViewSet(AppMapChangesetConfigViewSet):
    queryset = MapChangeset.objects.all()
    serializer_class = MapChangesetSerializer
