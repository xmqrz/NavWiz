from __future__ import absolute_import
from __future__ import unicode_literals

from rest_framework.exceptions import NotAcceptable as Http406
import ujson as json

from agv05_webserver.system.models import ExecutorMode, Variable


class FmsVoidMixin(object):

    def initial(self, request, *args, **kwargs):
        super(FmsVoidMixin, self).initial(request, *args, **kwargs)

        executor_mode = None
        try:
            executor_mode = int(Variable.objects.get(name=Variable.EXECUTOR_MODE).value)
        except Exception:
            pass

        if executor_mode == ExecutorMode.DFleet.value:
            fms_dashboard = None
            try:
                fms_dashboard = json.loads(Variable.objects.get(name=Variable.FMS_METADATA).value)['dashboard']
            except Exception:
                pass

            raise Http406({
                'detail': 'This setting is not available when activating DFleet mode.',
                'fms_dashboard': fms_dashboard
            })
