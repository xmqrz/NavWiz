from __future__ import absolute_import
from __future__ import unicode_literals

from django.contrib.auth import login
from django.http import Http404
from rest_framework import views, permissions
from rest_framework.response import Response

from .auth import get_user_info


class AgvPanelLogin(views.APIView):
    permission_classes = (permissions.AllowAny,)

    def post(self, request, format=None):
        user = request.user

        if user is not None and user.username == 'agv_panel':
            if user.is_active:
                login(request, user, 'django.contrib.auth.backends.ModelBackend')
                return Response(get_user_info(user))

        raise Http404()
