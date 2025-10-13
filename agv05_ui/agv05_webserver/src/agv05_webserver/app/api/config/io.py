from __future__ import absolute_import
from __future__ import unicode_literals

from rest_framework import viewsets

from agv05_webserver.system.models import Variable
from ...serializers import VariableSerializer
from ..mixin import CustomErrorMixin, Permission
from .mixin import VariableEndpointMixin
from .license_void import LicenseVoidMixin


class IoConfigViewSet(LicenseVoidMixin, CustomErrorMixin, VariableEndpointMixin, viewsets.ViewSet):
    permission_classes = Permission('system.view_system_panel')
    get_serializer = VariableSerializer

    def list(self, *args, **kwargs):
        # Note: show detail (similar to retrieve)
        return self._variable_get_endpoint(Variable.IO_NAME, *args, **kwargs)

    def create(self, *args, **kwargs):
        # Note: perform_update (similar to update method)
        return self._variable_endpoint(Variable.IO_NAME, *args, **kwargs)
