from __future__ import unicode_literals

from django.core.cache import cache
from django.db import models
from django.utils import timezone
from rospy.numpy_msg import numpy_msg
from six import python_2_unicode_compatible
import nav_msgs.msg

from ..system import models as system_models

# Create your models here.

# Map attribute enums


class Heading:
    SEGMENTS = 36
    RESOLUTION = 10
    MIN = 0
    MAX = 360
    NA = 360


class PathShape:
    STRAIGHT = 0
    BEZIER = 1
    TELEPORT = 2
    MAX_N = 3


# OccupancyGrid

class OccupancyGrid(numpy_msg(nav_msgs.msg.OccupancyGrid)):
    pass


def get_mapocg_upload_to(instance, filename):
    return 'maps/%s.png' % timezone.localtime(instance.created or timezone.now()).strftime('%Y%m%d_%H%M%S')


@python_2_unicode_compatible
class MapOcg(models.Model):
    name = models.CharField(max_length=127, unique=True, blank=True, null=True, default=None)
    metadata = models.TextField(blank=True, default='')
    png_file = models.ImageField(upload_to=get_mapocg_upload_to)
    map = models.ForeignKey(system_models.Map, on_delete=models.CASCADE)
    is_named = models.BooleanField(default=False)
    is_autosaved = models.BooleanField(default=False)
    created = models.DateTimeField(auto_now_add=True, db_index=True)

    class Meta:
        verbose_name = 'raw map'
        ordering = ('-is_named', 'name', '-created')

    def __str__(self):
        if not self.is_named:
            return '%s [%s]' % (
                '~Auto Saved~' if self.is_autosaved else '~Manual Saved~',
                timezone.localtime(self.created).strftime('%Y-%m-%d %H:%M:%S'))
        return self.name

    def save(self, *args, **kwargs):
        if not self.name:
            self.name = None
        self.is_named = not not self.name
        super(MapOcg, self).save(*args, **kwargs)


class MapChangeset(system_models.MapChangeset):
    ocg = models.ForeignKey(MapOcg, on_delete=models.SET_NULL, null=True)

    def save(self, *args, **kwargs):
        """
        Force creating a new instance instead of updating.
        """
        self.id = None
        super(MapChangeset, self).save(*args, **kwargs)


# Caching

class Cache(system_models.Cache):
    # Map
    OCG = 'ocg'

    @classmethod
    def set_ocg(cls, ocg):
        cache.set(cls.OCG, ocg, None)

    @classmethod
    def get_ocg(cls):
        return cache.get(cls.OCG)
