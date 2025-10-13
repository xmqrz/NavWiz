from __future__ import absolute_import
from __future__ import unicode_literals

from django.core.files.storage import default_storage
from django.http import StreamingHttpResponse
from django.utils import timezone
from rest_framework import permissions, viewsets
from rest_framework.decorators import action
from rest_framework.response import Response
from rest_framework.reverse import reverse
import codecs
import itertools

from ..serializers import LogSerializer


class LogViewSet(viewsets.ViewSet):
    permission_classes = (permissions.AllowAny,)
    serializer_class = LogSerializer

    def list(self, request, *args, **kwargs):
        return Response({
            'custom': reverse('app:api:log-custom', request=request),
        })

    @action(detail=False)
    def custom(self, request, *args, **kwargs):
        if not request.user.has_perm('system.view_log_files'):
            self.permission_denied(request)

        filename = 'custom_log_%s.csv' % timezone.localtime().strftime('%Y%m%d_%H%M%S')
        response_gen = itertools.chain(self._print_header(),
            self._dump_log('log/custom.log.1'), self._dump_log('log/custom.log'))
        response = StreamingHttpResponse(response_gen, content_type='text/csv')
        response['Content-Disposition'] = 'attachment; filename="%s"' % filename
        return response

    def _print_header(self):
        yield '\ufeff'  # Unicode BOM
        yield '"Date","Time","Time (ms)","Severity Level","Message"\n'

    def _dump_log(self, file_path):
        if not default_storage.exists(file_path):
            return

        with codecs.open(default_storage.path(file_path), 'r', 'utf-8') as f:
            for line in f:
                line = line.strip()
                try:
                    t, level, msg = line.split(None, 2)
                    t = timezone.make_naive(timezone.datetime.fromtimestamp(float(t), timezone.utc))
                    assert level in ['INFO', 'WARNING', 'ERROR']

                    yield '"%s","%s","%s","%s","%s"\n' % (
                        t.date(), t.time().replace(microsecond=0), int(t.microsecond / 1000),
                        level, self._csv_escape(msg))
                except Exception:
                    yield '"","","","","%s"\n' % self._csv_escape(line)

    def _csv_escape(self, s):
        return s.replace('"', '""')
