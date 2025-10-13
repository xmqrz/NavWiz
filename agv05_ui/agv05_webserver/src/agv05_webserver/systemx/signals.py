from __future__ import absolute_import
from __future__ import unicode_literals

from django.apps import apps
from django.db.models.signals import post_migrate, post_save
from django.dispatch import receiver

from .models import MapChangeset, MapOcg
from ..system.models import Variable
from ..system.signals import mark_models_cache_as_dirty, signals_suppressed


@receiver(post_save, sender=MapChangeset)
@receiver(post_save, sender=MapOcg)
@receiver(post_migrate, sender=apps.get_app_config('systemx'))
def _mark_models_cache_as_dirty(sender, **kwargs):
    if signals_suppressed[0]:
        return

    if sender in [MapChangeset, MapOcg]:
        try:
            active_map = [int(v) for v in Variable.objects.get(name=Variable.ACTIVE_MAP).value.split(',')]
        except Exception:
            return

        instance = kwargs.get('instance', None)
        if not instance:
            return

        if sender in [MapChangeset, MapOcg]:
            instance = instance.map
        if instance.pk not in active_map:
            return

        if sender == MapOcg:
            mc = MapChangeset.objects.filter(map=instance).first()
            if not mc or mc.ocg != kwargs['instance']:
                return

    mark_models_cache_as_dirty()
