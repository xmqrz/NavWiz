from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from django.contrib import admin
from django.core import exceptions
from django.db.models import Q

from . import models


# Register your models here.

@admin.register(models.AgvActivity)
class AgvActivityAdmin(admin.ModelAdmin):
    list_display = ('start', 'end', 'activity')


@admin.register(models.AgvStatistic)
class AgvStatisticAdmin(admin.ModelAdmin):
    list_display = ('__str__', )


@admin.register(models.Map)
class MapAdmin(admin.ModelAdmin):
    list_display = ('__str__', 'created')


class MapChangesetAdmin(admin.ModelAdmin):
    list_display = ('__str__', 'map', 'author')

    def save_model(self, request, obj, form, change):
        if getattr(obj, 'author', None) is None:
            obj.author = request.user
        super(MapChangesetAdmin, self).save_model(request, obj, form, change)


if not django_settings.TRACKLESS:
    admin.register(models.MapChangeset)(MapChangesetAdmin)


@admin.register(models.MapAnnotation)
class MapAnnotationAdmin(admin.ModelAdmin):
    list_display = ('__str__', 'map')


@admin.register(models.Parameter)
class ParameterAdmin(admin.ModelAdmin):
    list_display = ('__str__', )
    search_fields = ('key', )

    def get_queryset(self, request):
        return models.Parameter.objects.all()


@admin.register(models.Task)
class TaskAdmin(admin.ModelAdmin):
    list_display = ('__str__', 'task_template', 'status', 'sequence', 'owner')


@admin.register(models.TaskStatistic)
class TaskStatisticAdmin(admin.ModelAdmin):
    list_display = ('__str__', )


@admin.register(models.TaskTemplate)
class TaskTemplateAdmin(admin.ModelAdmin):
    list_display = ('__str__', 'category', 'is_active', 'is_top_level', 'modified')
    list_filter = ('category', 'is_active', 'is_top_level')


@admin.register(models.Variable)
class VariableAdmin(admin.ModelAdmin):
    list_display = ('__str__', 'value')
    search_fields = ('name', )

    def save_model(self, request, obj, form, change):
        if self._protected(obj):
            raise exceptions.PermissionDenied()
        return super(VariableAdmin, self).save_model(request, obj, form, change)

    def has_change_permission(self, request, obj=None):
        if self._protected(obj):
            return False
        return super(VariableAdmin, self).has_change_permission(request, obj)

    def has_delete_permission(self, request, obj=None):
        if self._protected(obj):
            return False
        return super(VariableAdmin, self).has_delete_permission(request, obj)

    def _protected(self, obj):
        return obj and (
            obj.name in models.Variable.PROTECTED or obj.name.startswith('license_'))


if django_settings.TRACKLESS:
    class VariableRos(models.Variable):
        class Meta:
            proxy = True
            verbose_name = 'variable (ROS)'
            verbose_name_plural = 'variables (ROS)'

    @admin.register(VariableRos)
    class VariableRosAdmin(VariableAdmin):
        def get_queryset(self, request):
            return (models.Variable.objects.using('alt')
                .filter(Q(name__startswith='/') |
                    Q(name__startswith='license_') |
                    Q(name__in=models.Variable.PROTECTED))
                .exclude(name__startswith='/agv05_executor/'))


@admin.register(models.Webhook)
class WebhookAdmin(admin.ModelAdmin):
    list_display = ('__str__', 'verify_ssl', 'events')
