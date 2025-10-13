from __future__ import absolute_import
from __future__ import unicode_literals

from django.core.files.storage import default_storage
from django.http import Http404
from django.utils import timezone
from rest_framework import viewsets
from rest_framework.response import Response
from rest_framework.reverse import reverse

from agv05_webserver.app.api.config.license_void import LicenseVoidMixin
from agv05_webserver.app.api.mixin import Permission
from sendfile import sendfile
import os
import re


class MapQualityMixin(LicenseVoidMixin):
    permission_classes = Permission('system.view_map_quality')


class MapQualityConfigViewSet(MapQualityMixin, viewsets.ViewSet):
    def get_context_data(self, request):
        context = {}

        # gather quality maps
        qmaps = []
        try:
            files = iter(
                sorted(os.listdir(default_storage.path('map_quality'))))
        except OSError:
            return context

        def _find_next_file(base, suffix):
            while True:
                self.filename = next(files)
                if self.filename[:15] != base:
                    return False
                if self.filename[15:] == suffix:
                    return True

        base_pattern = re.compile(r'\d{8}_\d{6}')
        try:
            self.filename = next(files)
            while True:
                base = self.filename[:15]
                if not base_pattern.match(base) or self.filename[15:] != '_avg.png':
                    self.filename = next(files)
                    continue
                if not _find_next_file(base, '_count.png'):
                    continue
                if not _find_next_file(base, '_layout.json'):
                    continue
                if not _find_next_file(base, '_max.png'):
                    continue
                if not _find_next_file(base, '_min.png'):
                    continue
                if not _find_next_file(base, '_ocg.json'):
                    continue
                if not _find_next_file(base, '_ocg.png'):
                    continue
                if not _find_next_file(base, '_sum.png'):
                    continue

                start = timezone.datetime.strptime(base, '%Y%m%d_%H%M%S')
                end = os.path.getmtime(default_storage.path('map_quality/' + base + '_avg.png'))
                end = timezone.datetime.fromtimestamp(end)

                qmaps.append((base, start, end))
                self.filename = next(files)

        except StopIteration:
            pass

        qmaps.reverse()
        context['qmaps'] = qmaps

        filename = '20220202_020202_avg.png'
        context['media_url'] = reverse(
            'app:config-api:map-quality-media-list',
            request=request,
            kwargs={'filename': filename}
        )[:-len(filename)]

        return context

    def list(self, request, *args, **kwargs):
        return Response(self.get_context_data(request))


class MapQualityMediaConfigViewSet(MapQualityMixin, viewsets.ViewSet):
    def list(self, request, *args, **kwargs):
        try:
            return sendfile(request, default_storage.path('map_quality/' + kwargs['filename']), attachment=True)
        except Exception:
            raise Http404('File not found.')
