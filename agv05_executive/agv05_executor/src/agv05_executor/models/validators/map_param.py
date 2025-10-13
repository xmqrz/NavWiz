from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, ExecutorMode, Variable
import ujson as json

from .validator import ValidationError, Validator

MP_PARAM = {
    'mp': {
        'name': 'Map parameter',
        'id': 'map-param',
        'item': '',
    }
}


class MapParamValidationMixin(object):

    def _validate_one(self, mp_name, mp_type, raw):

        try:
            if mp_type == 'double':
                cooked = float(raw)
            else:
                raise ValidationError('{mp} "%s" has invalid type "%s".' % (mp_name, mp_type), params=MP_PARAM)
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('{mp} "%s" has invalid value "%s".' % (mp_name, raw), params=MP_PARAM)


class MapParamValidator(MapParamValidationMixin, Validator):
    cache = Cache
    provides = ['map_params']
    extra_provides = ['map_params_assoc']

    def is_enabled(self, pre_validate=False):
        executor_mode = self.models.tmp['executor_mode'] if pre_validate else self.models.executor_mode
        return executor_mode == ExecutorMode.Standalone.value

    def validate(self):
        reserved_map_params = [{
            'name': 'Unlimited',
            'type': 'double',
            'default': -1.0,
        }]

        self.map_param_names = set(mp['name'] for mp in reserved_map_params)

        try:
            mps = Variable.objects.get(pk=Variable.MAP_PARAM).value
            if not mps:
                raise RuntimeError()
        except Exception:
            self.map_params = list(reserved_map_params)
            self.assemble()
            return

        try:
            self.map_params = json.loads(mps)
            assert isinstance(self.map_params, list)
        except Exception:
            raise ValidationError('{mp} data might have corrupted.', params=MP_PARAM)

        try:
            self._validate_map_param()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('{mp} data might have corrupted.', params=MP_PARAM)

        self.map_params = reserved_map_params + self.map_params
        self.assemble()

    def _validate_map_param(self):
        for mp in self.map_params:
            mp_name = mp['name']
            if not mp_name:
                raise ValidationError('{mp} has invalid name.', params=MP_PARAM)

            self._validate_one(mp_name, mp['type'], mp['default'])

            if mp_name in self.map_param_names:
                raise ValidationError('{mp} has duplicate parameter name: "%s".' % mp_name, params=MP_PARAM)
            self.map_param_names.add(mp_name)

    def assemble(self):
        self.map_params_assoc = {'${%s}' % p['name']: p for p in self.map_params}
