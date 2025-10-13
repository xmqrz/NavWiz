from __future__ import unicode_literals

from django.apps import AppConfig
import os
import sys


class SystemConfig(AppConfig):
    name = 'agv05_webserver.system'

    def ready(self):
        from . import signals

        for p in sys.path:
            if p.startswith('/opt/ros'):
                break
        else:
            raise RuntimeError('Invalid package')

        for d in ['agv05_executor', 'agv05_skills', 'agv05_webserver']:
            for root, dirs, files in os.walk(os.path.join(p, d)):
                for f in files:
                    if f.endswith('.py'):
                        raise RuntimeError('Invalid package')
                if 'migrations' in dirs:
                    dirs.remove('migrations')
