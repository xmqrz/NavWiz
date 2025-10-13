from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf.urls import include, url

from ..app.urls import router, config_router, sub_urlpatterns
from .api.config import (
    MapConfigViewSet,
    MapChangesetConfigViewSet,
    MapQualityConfigViewSet,
    MapQualityMediaConfigViewSet,
    MapOcgConfigViewSet,
    BackupConfigViewSet,
    PermissionConfigViewSet,
)

config_router.register(r'maps', MapConfigViewSet, 'maps')
config_router.register(r'maps/(?P<map_pk>[0-9]+)/changesets', MapChangesetConfigViewSet, 'map-changesets')
config_router.register(r'maps/(?P<map_pk>[0-9]+)/ocgs', MapOcgConfigViewSet, 'map-ocgs')
config_router.register(r'map-quality', MapQualityConfigViewSet, 'map-quality')
config_router.register(
    r'map-quality/media/(?P<filename>[0-9]{8}_[0-9]{6}_[a-z]{3,6}\.(png|json))$',
    MapQualityMediaConfigViewSet,
    'map-quality-media'
)
config_router.register(r'backup', BackupConfigViewSet, 'backup')
config_router.register(r'permissions', PermissionConfigViewSet, 'permissions')

# re-register for new added url
sub_urlpatterns['api'] = [
    url(r'^api/v3/config/', include(config_router.urls, namespace='config-api')),
    url(r'^api/v3/', include(router.urls, namespace='api')),
]

urlpatterns = [vv for v in sub_urlpatterns.values() for vv in v]
