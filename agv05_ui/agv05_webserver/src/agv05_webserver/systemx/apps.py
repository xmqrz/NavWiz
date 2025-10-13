from __future__ import unicode_literals

from django.apps import AppConfig


class SystemxConfig(AppConfig):
    name = 'agv05_webserver.systemx'

    def ready(self):
        from . import signals
