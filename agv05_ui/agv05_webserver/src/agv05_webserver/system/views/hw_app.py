from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from django.http import Http404
from django.views import View
from sendfile import sendfile
import os
import subprocess
import uuid

from ..models import Cache


class HwAppView(View):

    def get(self, request, *args, **kwargs):
        app_id = kwargs.get('app_id')
        asset = kwargs.get('asset')

        app_desc = None
        app_descriptions = Cache.get_models_app_descriptions()
        if app_descriptions:
            for a in app_descriptions:
                if a['id'] == app_id:
                    app_desc = a
                    break

        if not app_desc:
            raise Http404('Invalid app id.')

        base_directory = app_desc['assets']
        file_path = os.path.join(base_directory, asset)

        if os.path.exists(file_path):
            return unsafe_sendfile(request, file_path)
        else:
            raise Http404('Asset file unavailable')


def unsafe_sendfile(request, file_path):
    unsafe_dir = os.path.join(django_settings.MEDIA_ROOT, 'sendfile')
    if not os.path.exists(unsafe_dir):
        try:
            os.makedirs(unsafe_dir)
        except Exception:
            pass

    filename = os.path.basename(file_path)
    symlink_path = os.path.join(unsafe_dir, '%s%s' % (uuid.uuid4(), filename))
    os.symlink(file_path, symlink_path)

    # Let nginx start serve file before we remove it.
    subprocess.Popen('sleep 3 && sudo rm %s' % symlink_path, shell=True)

    return sendfile(request, symlink_path, attachment_filename=filename)
