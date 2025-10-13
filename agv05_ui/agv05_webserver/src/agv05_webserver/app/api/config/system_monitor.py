from __future__ import absolute_import
from __future__ import unicode_literals

from rest_framework import viewsets
from rest_framework.response import Response

from ..mixin import Permission
from .license_void import LicenseVoidMixin

import subprocess


class SystemMonitorConfigViewSet(LicenseVoidMixin, viewsets.ViewSet):
    permission_classes = Permission('system.view_log_files')

    def list(self, request, *args, **kwargs):
        usage = {}

        try:
            proc = subprocess.Popen(['grep', 'cpu ', '/proc/stat'],
                stdout=subprocess.PIPE, universal_newlines=True)
            parts = proc.communicate()[0].strip().split()
            values = list(map(int, parts[1:]))

            # Reference: https://github.com/giampaolo/psutil/blob/master/psutil/__init__.py
            # On Linux, guest times are already accounted in "user" ot "nice" times, so we skip reading them.
            fields = ['user', 'nice', 'system', 'idle', 'iowait', 'irq', 'softirq', 'steal']
            cpu_times = dict(zip(fields, values))
            total_times = sum(cpu_times.values())
            cpu = (total_times - cpu_times['idle'] - cpu_times['iowait']) * 100.0 / total_times

            usage['cpu'] = '%.3f%%' % cpu
            usage['cpu_times'] = cpu_times
        except Exception:
            pass

        try:
            proc = subprocess.Popen('lscpu | grep "CPU MHz"', stdout=subprocess.PIPE, shell=True, universal_newlines=True)
            usage['cpu_freq'] = proc.communicate()[0].strip().split()[-1] + ' MHz'
        except Exception:
            pass

        try:
            proc = subprocess.Popen(['free', '-h'], stdout=subprocess.PIPE, universal_newlines=True)
            usage['memory'] = proc.communicate()[0]
        except Exception:
            pass

        try:
            proc = subprocess.Popen(['df', '-h'], stdout=subprocess.PIPE, universal_newlines=True)
            usage['disk'] = proc.communicate()[0]
        except Exception:
            pass

        return Response(usage)
