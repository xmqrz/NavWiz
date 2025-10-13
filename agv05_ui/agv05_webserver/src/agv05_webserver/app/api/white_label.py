from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Variable
from django.core.files.base import File
from django.core.files.storage import default_storage
from django.http import Http404
from django.shortcuts import redirect
from django.template import Template
from django.template.context import make_context
from django.templatetags.static import static as django_static
from rest_framework import permissions, viewsets, mixins
from rest_framework.decorators import action
from rest_framework.response import Response
from rest_framework.reverse import reverse
from rest_framework.views import APIView
from sendfile import sendfile
from six import python_2_unicode_compatible
import os

from ..serializers import WhiteLabelSerializer
from .config.software_update import get_current_version


labels = {
    Variable.ADDRESS_LABEL: """<h3>DF Automation & Robotics Sdn. Bhd.</h3>
<address>
 5, Jalan Impian Emas 18<br/>
 Taman Impian Emas<br/>
 81300 Skudai, Johor, Malaysia.<br/>
 Tel: +607-5623547
</address>
<address>
 <strong>Sales and Support:</strong> sales@dfautomation.com
</address>
""",
    Variable.COPYRIGHT_LABEL:
    """Copyright &copy; 2013-{% now "Y" %}. DF Automation &amp; Robotics Sdn. Bhd.""",
}


def _get_labels(render=False):
    context = None
    if render:
        context = make_context({})
    values = dict(Variable.objects.filter(pk__in=labels).values_list('name', 'value'))
    data = {}
    for key in labels:
        val = values.get(key) or labels[key]
        try:
            if render:
                val = Template(val).render(context)
        except Exception:
            pass
        data[key] = val
    return data


class WhiteLabelViewSet(
    mixins.ListModelMixin,
    viewsets.GenericViewSet
):
    permission_classes = (permissions.AllowAny,)
    variable_info = [Variable.ADDRESS_LABEL, Variable.COPYRIGHT_LABEL]
    variable_file = [Variable.FAVICON_LABEL, Variable.LOGO_LABEL]

    # this is actually a `retrieve` operation portrayed as `list` so that
    # it can be included in the router's urls.
    def list(self, request, *args, **kwargs):
        data = _get_labels(render=True)
        data.update({
            'favicon': reverse('app:api:white-label-favicon', request=request),
            'logo': reverse('app:api:white-label-logo', request=request),
            'navwiz_version': get_current_version()
        })
        return Response(data)

    @action(detail=False)
    def favicon(self, request, *args, **kwargs):
        try:
            favicon = Variable.objects.get(pk=Variable.FAVICON_LABEL).value
            if favicon and default_storage.exists(favicon):
                return sendfile(self.request, default_storage.path(favicon))
        except Exception:
            pass
        return redirect(django_static('system/img/favicon.ico'))

    @action(detail=False)
    def logo(self, request, *args, **kwargs):
        try:
            logo = Variable.objects.get(pk=Variable.LOGO_LABEL).value
            if logo and default_storage.exists(logo):
                return sendfile(self.request, default_storage.path(logo))
        except Exception:
            pass
        return redirect(django_static('system/img/df_logo.png'))

    @action(detail=False, methods=['POST'], url_path='update-label', serializer_class=WhiteLabelSerializer)
    def update_label(self, request):
        # Note: perform_update (similar to update method)
        serializer = self.get_serializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        for pk in labels:
            Variable.objects.update_or_create(pk=pk, defaults={
                'value': serializer.validated_data[pk] or ''
            })

        # Handle file uploads
        for f in self.variable_file:
            if f not in serializer.validated_data:
                continue

            file = serializer.validated_data[f]

            # delete old files
            try:
                v = Variable.objects.get(pk=f)
                if default_storage.exists(v.value):
                    default_storage.delete(v.value)
                v.value = ''
                v.save()
            except Exception:
                pass

            if isinstance(file, File):
                file_path = os.path.join('white_label/img', file.name)
                default_storage.save(file_path, file)
                Variable.objects.update_or_create(pk=f, defaults={
                    'value': file_path
                })

        return self.get_update_label(request)

    @update_label.mapping.get
    def get_update_label(self, request, *args, **kwargs):
        # TODO: user OrderedDict
        initial = {}
        initial.update(_get_labels())
        favicon = None
        try:
            favicon = Variable.objects.get(pk=Variable.FAVICON_LABEL).value
        except Exception:
            pass
        initial['favicon_label'] = self.TrickFile(favicon, reverse(
            'app:white-label-download',
            kwargs={
                'file_name': os.path.basename(favicon),
            },
            request=self.request
        )) if favicon else self.TrickFile('favicon.ico', reverse(
            'app:api:white-label-favicon',
            request=self.request
        ))
        logo = None
        try:
            logo = Variable.objects.get(pk=Variable.LOGO_LABEL).value
        except Exception:
            pass
        initial['logo_label'] = self.TrickFile(logo, reverse(
            'app:white-label-download',
            kwargs={
                'file_name': os.path.basename(logo),
            },
            request=self.request
        )) if logo else self.TrickFile('logo.png', reverse(
            'app:api:white-label-logo',
            request=self.request
        ))
        return Response(self.get_serializer(initial).data)

    @python_2_unicode_compatible
    class TrickFile(object):

        def __init__(self, file, url):
            self.file = file
            self.name = os.path.basename(self.file)
            self.url = url

        def __str__(self):
            return self.name


class WhiteLabelFileDownload(APIView):
    # TODO: apply permission.
    # permission_required = ['system.view_system_panel', 'system.change_parameter']
    permission_classes = []

    def get(self, request, **kwargs):
        try:
            file_name = kwargs['file_name']
            file_path = os.path.join('white_label/img', file_name)
            return sendfile(request, default_storage.path(file_path), attachment=True)
        except Exception:
            raise Http404('File not found.')
