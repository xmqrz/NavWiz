from __future__ import absolute_import

from celery import Celery
import os

# set the default Django settings module for the 'celery' program.
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'agv05_webserver.settings')

from django.apps import apps  # noqa

app = Celery('agv05_webserver')

# Using a string here means the worker will not have to
# pickle the object when using Windows.
app.config_from_object('agv05_webserver.celeryconfig')
app.autodiscover_tasks(lambda: [n.name for n in apps.get_app_configs()])


@app.task(bind=True)
def debug_task(self):
    print('Request: {0!r}'.format(self.request))
