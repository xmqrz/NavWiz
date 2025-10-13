from __future__ import absolute_import
from __future__ import unicode_literals

from celery import shared_task
import requests

from ..models import Webhook
from ...app.serializers import AgvSerializer, TaskSerializer


@shared_task(ignore_result=True)
def dispatch_webhook(event_type, instance):
    if not instance:
        return

    if event_type == 'agv_update':
        data = AgvSerializer(instance).data

    elif event_type in ['task_create', 'task_update']:
        data = TaskSerializer(instance).data

    else:
        return

    for w in Webhook.objects.filter(events__contains='"%s"' % event_type):
        headers = {
            'Content-Type': 'application/json',
            'X-Navwiz-Event': event_type,
        }
        if w.secret_token:
            headers['X-Navwiz-Token'] = w.secret_token

        try:
            requests.post(w.url, headers=headers, json=data, verify=w.verify_ssl, timeout=(3.05, 7))
        except Exception:
            pass
