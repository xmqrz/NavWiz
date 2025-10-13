from __future__ import absolute_import
from __future__ import unicode_literals

from celery import shared_task
import random


@shared_task(ignore_result=True)
def update_license():
    from ...app.api.config.license import update_license
    update_license()


@shared_task(ignore_result=True)
def trigger():
    # apply a random delay to avoid flooding the license server
    update_license.apply_async(countdown=random.randint(0, 300))
