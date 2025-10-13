from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.utils import timezone
from rest_framework import viewsets
from rest_framework.response import Response
from rest_framework.reverse import reverse

import logging
import os

from ..mixin import Permission

logger = logging.getLogger(__name__)
DIAGNOSTIC_DIR = '/var/lib/navwiz/.ros/diagnostics'


class DiagnosticConfigViewSet(viewsets.ViewSet):
    permission_classes = Permission('system.view_log_files')

    def list(self, request, *args, **kwargs):
        diagnostics = []
        try:
            for filename in os.listdir(DIAGNOSTIC_DIR):
                filepath = os.path.join(DIAGNOSTIC_DIR, filename)
                if os.path.isdir(filepath):
                    continue

                stat = os.stat(filepath)
                modified_time = timezone.datetime.fromtimestamp(stat.st_mtime, timezone.get_current_timezone())
                filesize = '%.1f MB' % (stat.st_size / (1024.0 * 1024.0))
                url = reverse('diagnostic-download', kwargs={
                    'file_name': filename,
                }, request=request)

                diagnostics.append(OrderedDict((
                    ('filename', filename),
                    ('timestamp', modified_time),
                    ('size', filesize),
                    ('url', url),
                )))
        except Exception as ex:
            logger.error('Fail to get list of diagnostics: %s' % ex)

        diagnostics = sorted(diagnostics, key=lambda x: x['timestamp'], reverse=True)
        return Response(diagnostics)
