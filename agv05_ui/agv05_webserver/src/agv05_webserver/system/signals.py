from __future__ import absolute_import
from __future__ import unicode_literals

from django.apps import apps
from django.conf import settings as django_settings
from django.db import transaction
from django.db.models.signals import post_delete, post_migrate, post_save
from django.dispatch import receiver
import contextdecorator
import logging
import rospy
import std_srvs.srv

from .models import Cache, Map, MapAnnotation, MapChangeset, Task, TaskTemplate, Variable
from .tasks.dispatch_webhook import dispatch_webhook


logger = logging.getLogger(__name__)

signals_suppressed = [False]


@contextdecorator.contextmanager
def suppress_signals():
    signals_suppressed[0] = True
    yield
    signals_suppressed[0] = False


def mark_models_cache_as_dirty():
    if signals_suppressed[0]:
        return

    Cache.set_flag(Cache.Flag.Dirty.value)

    def on_commit():
        # Send action trigger to agv05_executor
        agv05_executor = getattr(django_settings, 'AGV05_EXECUTOR')
        try:
            srv = rospy.ServiceProxy(agv05_executor + '/validate_models', std_srvs.srv.Trigger, persistent=False)
            res = srv()
        except Exception as ex:
            logger.warn('Failed to call validate_models service: %s', ex)
    transaction.on_commit(on_commit)


@receiver(post_save, sender=MapChangeset)
@receiver(post_save, sender=TaskTemplate)
@receiver(post_save, sender=MapAnnotation)
@receiver(post_delete, sender=Map)
@receiver(post_delete, sender=TaskTemplate)
@receiver(post_migrate, sender=apps.get_app_config('system'))
def _mark_models_cache_as_dirty(sender, **kwargs):
    if signals_suppressed[0]:
        return

    if sender in [Map, MapAnnotation, MapChangeset]:
        try:
            active_map = [int(v) for v in Variable.objects.get(name=Variable.ACTIVE_MAP).value.split(',')]
        except Exception:
            return

        instance = kwargs.get('instance', None)
        if not instance:
            return

        if sender in [MapChangeset, MapAnnotation]:
            instance = instance.map
        if instance.pk not in active_map:
            return

    mark_models_cache_as_dirty()


@receiver(post_save, sender=Task)
def _invalidate_tasks(sender, instance, created, **kwargs):
    if signals_suppressed[0]:
        return

    event_type = 'task_create' if created else 'task_update'
    dispatch_webhook.apply_async((event_type, instance), expires=3600)
