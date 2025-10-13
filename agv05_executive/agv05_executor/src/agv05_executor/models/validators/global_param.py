from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, ExecutorMode, Variable
from django.utils.encoding import force_text
import ujson as json

from .validator import ValidationError, Validator

TTGP_PARAM = {
    'gp': {
        'name': 'Task template global parameter',
        'id': 'task-template-global-param',
        'item': '',
    }
}


class GlobalParamValidationMixin(object):

    def _validate_one(self, gp_name, gp_type, raw):

        try:
            if gp_type == 'bool':
                cooked = bool(raw)
            elif gp_type == 'int':
                cooked = int(raw)
            elif gp_type == 'double':
                cooked = float(raw)
            elif gp_type == 'str':
                cooked = force_text(raw)
            elif gp_type == 'Station':
                if raw not in self.models.tmp['station_names']:
                    raise ValidationError('{gp} "%s" refers to invalid station "%s".' % (gp_name, raw), params=TTGP_PARAM)
                cooked = raw
            elif gp_type == 'Register':
                if not self.robot.registry.get(raw):
                    raise ValidationError('{gp} "%s" refers to invalid register "%s".' % (gp_name, raw), params=TTGP_PARAM)
                cooked = raw
            else:
                raise ValidationError('{gp} "%s" has invalid type "%s".' % (gp_name, gp_type), params=TTGP_PARAM)

            return cooked
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('{gp} "%s" has invalid value "%s".' % (gp_name, raw), params=TTGP_PARAM)


class GlobalParamValidator(GlobalParamValidationMixin, Validator):
    cache = Cache
    provides = ['global_params']

    def is_enabled(self, pre_validate=False):
        executor_mode = self.models.tmp['executor_mode'] if pre_validate else self.models.executor_mode
        return executor_mode == ExecutorMode.Standalone.value

    def validate(self):
        reserved_global_params = [{
            'name': 'agv_home',
            'type': 'Station',
            'default': None,
        }]

        self.global_param_names = set(gp['name'] for gp in reserved_global_params)

        try:
            gps = Variable.objects.get(pk=Variable.GLOBAL_PARAM).value
            if not gps:
                raise RuntimeError()
        except Exception:
            self.global_params = list(reserved_global_params)
            return

        try:
            self.global_params = json.loads(gps)
            assert isinstance(self.global_params, list)
        except Exception:
            raise ValidationError('{gp} data might have corrupted.', params=TTGP_PARAM)

        try:
            self._validate_global_param()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('{gp} data might have corrupted.', params=TTGP_PARAM)

        self.global_params = reserved_global_params + self.global_params

    def _validate_global_param(self):
        for gp in self.global_params:
            gp_name = gp['name']
            if not gp_name:
                raise ValidationError('{gp} has invalid name.', params=TTGP_PARAM)

            gp['default'] = self._validate_one(gp_name, gp['type'], gp['default'])

            if gp_name in self.global_param_names:
                raise ValidationError('{gp} "%s" has duplicate name.' % gp_name, params=TTGP_PARAM)
            self.global_param_names.add(gp_name)
