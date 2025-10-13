from __future__ import absolute_import
from __future__ import unicode_literals

from rest_framework import serializers

from .models import MapChangeset


class MapChangesetSummarySerializer(serializers.ModelSerializer):
    author = serializers.StringRelatedField()

    class Meta:
        model = MapChangeset
        fields = ('id', 'author', 'created')


class MapChangesetSerializer(serializers.ModelSerializer):
    ocg = serializers.PrimaryKeyRelatedField(read_only=True)
    author = serializers.StringRelatedField()

    class Meta:
        model = MapChangeset
        fields = ('id', 'metadata', 'structure', 'stations', 'ocg', 'author', 'created')
