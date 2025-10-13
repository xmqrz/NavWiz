from __future__ import absolute_import
from __future__ import unicode_literals

from django.contrib.auth import get_user_model
from rest_framework.authentication import SessionAuthentication, get_authorization_header
from rest_framework import exceptions
from django.utils.translation import ugettext_lazy

from .models import check_agv_panel_pin


class AgvPanelSessionAuthentication(SessionAuthentication):
    keyword = 'Pin'

    def authenticate(self, request):
        ret = super(AgvPanelSessionAuthentication, self).authenticate(request)
        if not ret:
            return ret
        user, _ = ret
        if user.username != 'agv_panel':
            return ret

        auth = get_authorization_header(request).split()
        if not auth or len(auth) != 2 or auth[0].lower() != self.keyword.lower().encode():
            return ret

        try:
            pin = auth[1].decode()
        except UnicodeError:
            return ret

        if not check_agv_panel_pin(pin):
            raise exceptions.AuthenticationFailed(ugettext_lazy('Invalid pin number.'))

        try:
            User = get_user_model()
            user_agv_panel_pp = User.objects.get(username='agv_panel_pin_protected')
            return (user_agv_panel_pp, None)
        except Exception:
            return ret
