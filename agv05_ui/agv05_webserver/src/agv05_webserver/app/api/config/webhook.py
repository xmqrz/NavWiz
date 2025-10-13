from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Webhook
from rest_framework import permissions, viewsets

from ...serializers import WebhookSerializer, WebhookEditSerializer
from ..mixin import Permission
from .license_void import LicenseVoidMixin


class WebhookViewSet(LicenseVoidMixin, viewsets.ModelViewSet):
    permission_classes = (Permission('system.view_system_panel'), permissions.DjangoModelPermissions)
    queryset = Webhook.objects.all()

    def get_serializer_class(self):
        if self.action in ['create', 'retrieve', 'update', 'partial_update']:
            return WebhookEditSerializer
        return WebhookSerializer

    def retrieve(self, request, *args, **kwargs):
        if not request.user.has_perm('system.change_webhook'):
            self.permission_denied(request)
        return super(WebhookViewSet, self).retrieve(request, *args, **kwargs)
