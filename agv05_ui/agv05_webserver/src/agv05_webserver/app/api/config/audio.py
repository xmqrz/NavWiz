from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Parameter
from django.conf import settings as django_settings
from django.contrib.auth import get_user_model
from django.core.files.base import File
from django.core.files.storage import default_storage
from django.http import Http404
from rest_framework import serializers
from rest_framework.response import Response
from rest_framework.reverse import reverse
from rest_framework.views import APIView
from sendfile import sendfile
from six import python_2_unicode_compatible
import logging
import os
try:
    from yaml import full_load
except Exception:
    from yaml import load as full_load

from ..mixin import Permission
from ...serializers import audioserializer_factory
from .parameter import ParameterConfigViewSet

User = get_user_model()
logger = logging.getLogger(__name__)


class AudioConfigViewSet(ParameterConfigViewSet):

    def list(self, request, *args, **kwargs):
        # Note: show detail (similar to retrieve)
        instance = self.get_object()
        serializer = self.get_serializer(instance)
        return Response(serializer.data)

    def get_serializer(self, instance=None, *args, **kwargs):
        if not instance:
            instance = self.get_object()
        kwargs['many'] = False
        return super(AudioConfigViewSet, self).get_serializer(instance, *args, **kwargs)

    def create(self, request):
        # Note: perform_update (similar to update method)
        instance = self.get_object()
        serializer = self.get_serializer(instance, data=request.data)
        serializer.is_valid(raise_exception=True)
        self.perform_update(serializer)

        # NOTE: use data from dyncfg_client, saved data might be modified.
        return self.list(request)

    def get_object(self):
        try:
            return Parameter.objects.get(key=django_settings.AGV05_AUDIO_PLAYER)
        except Parameter.DoesNotExist:
            return Parameter(key=django_settings.AGV05_AUDIO_PLAYER)

    def get_initial(self, obj, serializer_class):
        initial = super(AudioConfigViewSet, self).get_initial(obj, serializer_class)
        self._patch_file_initial(initial, serializer_class)
        return initial

    def _patch_file_initial(self, initial, serializer_class):
        # convert path string to file obj.
        fields = serializer_class._declared_fields
        for k, v in initial.items():
            if k not in fields:
                continue
            field = fields[k]
            if isinstance(field, serializers.ListField) and isinstance(field.child, serializers.FileField):
                initial[k] = [self._trick_file(p, k) for p in v.split(';') if p]
            if isinstance(field, serializers.FileField):
                initial[k] = self._trick_file(v, k) if v else None

    def _trick_file(self, path, field_name):
        return self.TrickFile(path, reverse(
            'app:audio-download',
            kwargs={
                'field_name': field_name,
                'file_name': os.path.basename(path),
            },
            request=self.request
        ))

    @python_2_unicode_compatible
    class TrickFile(object):

        def __init__(self, file, url):
            self.file = file
            self.name = os.path.basename(self.file)
            self.url = url

        def __str__(self):
            return self.name

    def perform_update(self, serializer):
        self.save_media_fields(serializer)
        return super(AudioConfigViewSet, self).perform_update(serializer)

    def save_media_fields(self, serializer):
        # Handle file uploads
        for m in serializer.MEDIA_FIELDS:
            # files unchanged
            if m not in serializer.validated_data:
                continue

            files = serializer.validated_data[m]

            # normalize for single file upload field
            if not isinstance(files, list):
                files = [files]

            # ClearableFileField return true for reset
            files = [f for f in files if isinstance(f, File)]

            # delete old files
            if serializer.instance_data.get(m):
                orig_files = serializer.instance_data[m]
                if not isinstance(orig_files, list):
                    orig_files = [orig_files]
                for f in orig_files:
                    default_storage.delete(f)

            # delete other files in the folder in case the dyncfg data has been reset.
            folder = os.path.join('audio', m)
            try:
                for f in default_storage.listdir(folder)[1]:
                    default_storage.delete(os.path.join(folder, f))
            except Exception:
                # folder doesn't exist.
                pass

            # save new files
            new_files = []
            for f in files:
                new_files.append(default_storage.save(os.path.join(folder, f.name), f))

            serializer.validated_data[m] = ';'.join(new_files)

    def generate_serialzier_class(self, obj):
        parameter_serializer_class = super(AudioConfigViewSet, self).generate_serialzier_class(obj)
        serializer_class = audioserializer_factory(parameter_serializer_class)
        return serializer_class

    def retrieve(self, request, *args, **kwargs):
        raise Http404()

    def update(self, request, pk=None):
        raise Http404()

    def partial_update(self, request, pk=None):
        raise Http404()

    def destroy(self, request, pk=None):
        raise Http404()


class AudioFileDownloadView(APIView):
    permission_classes = Permission('system.view_system_panel', 'system.change_parameter')

    def get_context_data(self, **kwargs):
        context = {}

        try:
            context['field_name'] = kwargs['field_name']
            context['file_name'] = kwargs['file_name']
            p = Parameter.objects.get(key=django_settings.AGV05_AUDIO_PLAYER)
            config_msg = full_load(p.value)
            context['configuration'] = {c['name']: c['value'] for c in config_msg['bools'] + config_msg['ints'] + config_msg['strs'] + config_msg['doubles']}
        except Exception:
            context['configuration'] = {}

        return context

    def get(self, request, **kwargs):
        try:
            context = self.get_context_data(**kwargs)

            configuration = context['configuration']
            paths = configuration[context['field_name']].split(';')
            paths = [p for p in paths if os.path.basename(p) == context['file_name']]
            return sendfile(request, default_storage.path(paths.pop()), attachment=True)
        except Exception:
            raise Http404('File not found.')
