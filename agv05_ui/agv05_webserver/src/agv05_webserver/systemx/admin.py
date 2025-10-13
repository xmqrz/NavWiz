from __future__ import absolute_import
from __future__ import unicode_literals

from django.contrib import admin

from . import models
from ..system import admin as system_admin


# Register your models here.

@admin.register(models.MapOcg)
class MapOcg(admin.ModelAdmin):
    list_display = ('__str__', 'map', 'created')


@admin.register(models.MapChangeset)
class MapChangesetAdmin(system_admin.MapChangesetAdmin):
    pass
