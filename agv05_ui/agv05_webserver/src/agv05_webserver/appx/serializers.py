from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from django.contrib.auth import get_user_model
from django.core.files.base import ContentFile
from rest_framework import serializers
from rest_framework.reverse import reverse
import base64
import ujson as json

from agv05_webserver.system.models import Map
from agv05_webserver.systemx.models import MapChangeset, MapOcg
from agv05_webserver.app.serializers import MapLayoutMixin, MapListSerializer as AppMapListSerializer

User = get_user_model()


class MapListSerializer(AppMapListSerializer):
    active = serializers.SerializerMethodField()

    class Meta:
        model = Map
        fields = ('id', 'name', 'created', 'active')
        read_only_fields = ('id', 'name', 'created')


class MapLayoutSerializer(MapLayoutMixin, serializers.ModelSerializer):
    overwrite = serializers.BooleanField(required=False)
    modified = serializers.CharField(required=False, source='created')
    params = serializers.SerializerMethodField()
    reserved_params = serializers.SerializerMethodField()
    ocg_choices = serializers.SerializerMethodField()
    dynamic = serializers.SerializerMethodField()
    ocg = serializers.PrimaryKeyRelatedField(
        queryset=MapOcg.objects.all(),
        pk_field=serializers.IntegerField()
    )

    def __init__(self, *args, **kwargs):
        super(MapLayoutSerializer, self).__init__(*args, **kwargs)
        self.filter_ocg_queryset(self.fields['ocg'], *args, **kwargs)

    def filter_ocg_queryset(self, f, instance=None, data=serializers.empty, request=None, **kwargs):
        map_id = None
        if not instance and not self.context.get('view'):
            f.queryset = f.queryset.none()
            return
        elif not instance:
            view = self.context.get('view')
            map_id = view.kwargs.get('pk')
        else:
            map_id = instance.pk

        f.queryset = f.queryset.filter(map__pk=map_id)

    class Meta:
        model = MapChangeset
        fields = ('metadata', 'structure', 'stations', 'reserved_params',
                  'params', 'modified', 'overwrite', 'ocg', 'ocg_choices',
                  'dynamic')

    def get_ocg_choices(self, obj):
        ocg_choices = MapOcg.objects.filter(map=obj.map)
        ocgSerializer = MapOcgListSerializer(ocg_choices, many=True)
        return json.dumps([
            dict(
                id=d['id'],
                metadata=d['metadata'],
                display_name=d['display_name'],
                url=d['url']
            )
            for d in ocgSerializer.data
        ])

    def get_dynamic(self, obj):
        return bool(django_settings.DYNAMIC_PATH_PLANNING)


class MapChangesetSerializer(serializers.ModelSerializer):
    author = serializers.StringRelatedField()

    class Meta:
        model = MapChangeset
        fields = ('id', 'metadata', 'structure', 'stations', 'ocg', 'author', 'created')


class MapOcgListSerializer(MapLayoutMixin, serializers.ModelSerializer):
    name = serializers.CharField(required=False, write_only=True)
    display_name = serializers.SerializerMethodField()
    url = serializers.SerializerMethodField()
    png_file = serializers.ImageField(write_only=True)

    class Meta:
        model = MapOcg
        fields = ('id', 'name', 'display_name', 'metadata',
            'png_file', 'url', 'is_named', 'is_autosaved', 'created')
        read_only_fields = ('is_named', 'is_autosaved')

    def get_display_name(self, obj):
        return str(obj)

    def get_url(self, obj):
        if not obj.png_file or not obj.png_file.path:
            return ''
        return reverse(
            'app:config-api:map-ocgs-png',
            request=self.context.get('request'),
            kwargs={
                'map_pk': obj.map.pk,
                'pk': obj.pk,
            }
        )


class Base64ImageField(serializers.CharField):
    filename = 'raw.png'
    png_scheme = 'data:image/png;base64,'

    def to_internal_value(self, data):
        data = super(Base64ImageField, self).to_internal_value(data)
        if data.startswith(self.png_scheme):
            try:
                return ContentFile(base64.b64decode(data.split(self.png_scheme)[1]), name=self.filename)
            except Exception:
                pass
        self.fail('invalid')

    def to_representation(self, value):
        if value:
            try:
                png = self.png_scheme + base64.b64encode(value.read()).decode()
                return png
            except Exception:
                pass

        return ''


class MapOcgSerializer(MapOcgListSerializer):
    display_name = serializers.SerializerMethodField()
    url = serializers.SerializerMethodField()
    png_file = Base64ImageField()
    replace = serializers.BooleanField(initial=False, required=False)
    clear_name = serializers.BooleanField(initial=False, required=False)
    dynamic = serializers.SerializerMethodField()

    class Meta:
        model = MapOcg
        fields = ('id', 'name', 'clear_name', 'display_name', 'metadata', 'replace',
            'png_file', 'url', 'is_named', 'is_autosaved', 'created', 'dynamic')
        read_only_fields = ('is_named', 'is_autosaved')

    def get_dynamic(self, obj):
        return bool(django_settings.DYNAMIC_PATH_PLANNING)
