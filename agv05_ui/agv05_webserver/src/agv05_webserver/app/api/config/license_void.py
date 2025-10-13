from __future__ import absolute_import
from __future__ import unicode_literals

from rest_framework.exceptions import PermissionDenied as Http403
import time

from agv05_webserver.system.models import val_lic


class LicenseVoidMixin(object):
    permitted = 0

    def initial(self, request, *args, **kwargs):
        super(LicenseVoidMixin, self).initial(request, *args, **kwargs)
        now = time.time()
        if now - self.permitted < 60:
            return
        r = val_lic()
        if r.valid:
            LicenseVoidMixin.permitted = now
        else:
            if not r.features:
                msg = 'The license key is invalid.'
            elif r.days < 0:
                msg = 'This %s license has expired.' % ('subscription' if r.features & 1 else 'trial')
            else:
                msg = 'Some features are not licensed.'
            raise Http403(msg)
