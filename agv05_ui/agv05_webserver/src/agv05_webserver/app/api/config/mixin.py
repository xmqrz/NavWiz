from __future__ import absolute_import
from __future__ import unicode_literals

from django.utils import timezone
from rest_framework import status
from rest_framework.response import Response

from agv05_webserver.system.models import Variable
from agv05_webserver.system.signals import mark_models_cache_as_dirty


class VariableEndpointMixin(object):

    def _variable_endpoint(self, name, request, *args, **kwargs):
        instance = None
        try:
            instance = Variable.objects.get(name=name)
        except Variable.DoesNotExist:
            pass
        serializer = self.get_serializer(instance or Variable(name=name), data=request.data, partial=True)
        serializer.is_valid(raise_exception=True)

        overwrite = serializer.validated_data.get('overwrite')
        if not overwrite and instance:
            modified = serializer.validated_data.get('modified')
            if str(modified) != str(instance.modified):
                self._custom_error('Your changes are not saved. The content has been changed while you are editing it.', status_code=status.HTTP_409_CONFLICT)

        instance = serializer.save()
        mark_models_cache_as_dirty()
        return Response(serializer.data)

    def _variable_get_endpoint(self, name, request, *args, **kwargs):
        try:
            v = Variable.objects.get(name=name)
        except Exception:
            v = Variable(
                name=name,
                modified=timezone.now() - timezone.timedelta(days=1)
            )

        serializer = self.get_serializer(v)
        return Response(serializer.data)
