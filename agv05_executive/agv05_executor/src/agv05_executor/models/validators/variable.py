from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, ExecutorMode, Variable
from django.utils.encoding import force_text
import ujson as json

from .validator import ValidationError, Validator

VARIABLE_PARAM = {
    'v': {
        'name': 'Task Template Variable',
        'id': 'variable',
        'item': '',
    }
}


class VariableValidator(Validator):
    cache = Cache
    provides = ['variables']
    extra_provides = ['variables_assoc']

    type_map = {
        'bool': 'vbool',
        'int': 'vint',
        'str': 'vstr',
        'double': 'vdouble',
        'Station': 'vStation',
    }

    def is_enabled(self, pre_validate=False):
        executor_mode = self.models.tmp['executor_mode'] if pre_validate else self.models.executor_mode
        return executor_mode == ExecutorMode.Standalone.value

    def validate(self):
        self.variables = []
        try:
            variables = Variable.objects.get(pk=Variable.VARIABLE).value
            if not variables:
                raise RuntimeError()
        except Exception:
            self._construct_variables_assoc()
            return

        try:
            self.variables = json.loads(variables)
            assert isinstance(self.variables, list)
        except Exception:
            raise ValidationError('{v} data might have corrupted.', params=VARIABLE_PARAM)

        try:
            self._validate_variable()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('{v} data might have corrupted.', params=VARIABLE_PARAM)

        self._construct_variables_assoc()

    def _validate_variable(self):
        variable_names = []
        for v in self.variables:
            v_name = v['name']
            if not v_name:
                raise ValidationError('{v} has invalid name.', params=VARIABLE_PARAM)

            self._validate_one(v_name, v['type'], v['default'])
            v['vtype'] = self.type_map[v['type']]

            if v_name in variable_names:
                raise ValidationError('{v} "%s" has duplicate name.' % v_name, params=VARIABLE_PARAM)
            variable_names.append(v_name)

    def _validate_one(self, v_name, v_type, raw):

        def error_msg(error):
            return '{v} "%s" %s' % (v_name, error)

        try:
            if v_type == 'bool':
                cooked = bool(raw)
            elif v_type == 'int':
                cooked = int(raw)
            elif v_type == 'double':
                cooked = float(raw)
            elif v_type == 'str':
                cooked = force_text(raw)
            elif v_type == 'Station':
                if raw not in self.models.tmp['station_names']:
                    raise ValidationError(error_msg('refers to invalid station "%s".' % raw), params=VARIABLE_PARAM)
            else:
                raise ValidationError(error_msg('has invalid type "%s".' % v_type), params=VARIABLE_PARAM)
        except ValidationError:
            raise
        except Exception:
            raise ValidationError(error_msg('has invalid value "%s".' % raw), params=VARIABLE_PARAM)

    def assemble(self):
        self._construct_variables_assoc()

    def _construct_variables_assoc(self):
        self.variables_assoc = {
            v['name']: v for v in self.variables
        }
