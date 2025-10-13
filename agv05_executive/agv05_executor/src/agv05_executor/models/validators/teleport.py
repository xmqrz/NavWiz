from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, Direction, ExecutorMode, \
    PathShape, Variable
from django.utils.encoding import force_text
import six
import ujson as json

from .validator import ValidationError, Validator

TELEPORT_PARAM = {
    'teleport': {
        'name': 'Teleport',
        'id': 'map-teleport',
        'item': '',
    }
}


class TeleportValidator(Validator):
    cache = Cache
    provides = ['teleports']
    extra_provides = ['graph']

    HEADING_NA = Direction.NA
    HEADING_WARNING = 'directionless'

    teleport_params = [{
        'name': 'start',
        'type': 'Station',
    }, {
        'name': 'end',
        'type': 'Station',
    }, {
        'name': 'auto_reset_agv_position',
        'type': 'int',
    }, {
        'name': 'next_motion',
        'type': 'int',
    }, {
        'name': 'pre_steps',
        'type': 'int',
    }, {
        'name': 'distance_cost',
        'type': 'double',
    }]

    def is_enabled(self, pre_validate=False):
        executor_mode = self.models.tmp['executor_mode'] if pre_validate else self.models.executor_mode
        return executor_mode == ExecutorMode.Standalone.value

    def validate(self):
        self.teleports = []
        self.graph = self.models.tmp['graph']
        self.stations_assoc = self.models.tmp['stations_assoc']

        try:
            teleport = Variable.objects.get(pk=Variable.TELEPORT).value
        except Exception:
            return

        if not teleport:
            return

        try:
            self.teleports = json.loads(teleport)
            assert isinstance(self.teleports, list)
        except Exception:
            raise ValidationError('{teleport} data might have corrupted.', params=TELEPORT_PARAM)

        try:
            self._validate_teleport()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('{teleport} data might have corrupted.', params=TELEPORT_PARAM)

        self._amend_graph()

    def _validate_teleport(self):
        self._params_dict = {
            '${%s}' % p['name']: p for p in self.teleport_params
        }
        for p in self.models.tmp['global_params']:
            self._params_dict.setdefault('${%s}g' % p['name'], p)
        for v in self.models.tmp['variables']:
            self._params_dict.setdefault('${%s}v' % v['name'], v)

        edges_map = {}
        for idx, t in enumerate(self.teleports):
            msg_prefix = '{teleport} #%d' % (idx + 1)
            if t['start'] not in self.models.tmp['station_names']:
                raise ValidationError('%s: start point refers to invalid station "%s".' %
                    (msg_prefix, t['start']), params=TELEPORT_PARAM)

            if t['end'] not in self.models.tmp['station_names']:
                raise ValidationError('%s: end point refers to invalid station "%s".' %
                    (msg_prefix, t['end']), params=TELEPORT_PARAM)

            if t['start'] == t['end']:
                raise ValidationError('%s: start and end points must not be the same.' % msg_prefix, params=TELEPORT_PARAM)

            start_location = self.stations_assoc[t['start']]
            end_location = self.stations_assoc[t['end']]

            if start_location[0] == end_location[0]:
                raise ValidationError('%s: start and end points must not be on the same junction.' % msg_prefix, params=TELEPORT_PARAM)

            if end_location[1] == self.HEADING_NA:
                raise ValidationError('%s: end point must not be %s.' % (msg_prefix, self.HEADING_WARNING), params=TELEPORT_PARAM)

            e = (start_location[0], end_location[0])
            if e in edges_map:
                raise ValidationError('%s: start and end points overlap with teleport #%d.' % (msg_prefix, edges_map[e] + 1), params=TELEPORT_PARAM)
            edges_map[e] = idx

            try:
                if t['preAction']['skillId']:
                    self._validate_action(t['preAction'], ['base', 'plan', 'traffic'])
                else:
                    t['preAction'] = None
            except ValidationError as ex:
                params = TELEPORT_PARAM.copy()
                params.update(ex.params)
                raise ValidationError('%s pre-action: %s' % (msg_prefix, ex), params=params)

            try:
                if t['action']['skillId']:
                    self._validate_action(t['action'], ['plan', 'traffic'])
                else:
                    t['action'] = None
            except ValidationError as ex:
                params = TELEPORT_PARAM.copy()
                params.update(ex.params)
                raise ValidationError('%s action: %s' % (msg_prefix, ex), params=params)

            try:
                t['alignStationType'] = int(t['alignStationType'])
                if self.robot.trackless:
                    if t['alignStationType'] not in [-1, 0, 1]:
                        raise RuntimeError()
                else:
                    if t['alignStationType'] not in [0, 1]:
                        raise RuntimeError()
            except Exception:
                raise ValidationError('%s: `align station type` has invalid value.' % msg_prefix, params=TELEPORT_PARAM)

            try:
                t['nonStopTransition'] = bool(t['nonStopTransition'])
            except Exception:
                raise ValidationError('%s: `non-stop transition` has invalid value.' % msg_prefix, params=TELEPORT_PARAM)

            try:
                t['autoResetAgvPosition'] = int(t['autoResetAgvPosition'])
                if self.robot.trackless:
                    if t['autoResetAgvPosition'] not in [0, 1, 2]:
                        raise RuntimeError()
                else:
                    if t['autoResetAgvPosition'] not in [0, 1]:
                        raise RuntimeError()
            except Exception:
                raise ValidationError('%s: `auto-reset AGV position` has invalid value.' % msg_prefix, params=TELEPORT_PARAM)

            try:
                t['validateRfid'] = bool(t.get('validateRfid', True)) if not self.robot.trackless else False
            except Exception:
                raise ValidationError('%s: `validate RFID` has invalid value.' % msg_prefix, params=TELEPORT_PARAM)

            try:
                t['distance'] = float(t['distance'])
                if t['distance'] <= 0 or t['distance'] > 10000:
                    raise RuntimeError()
            except Exception:
                raise ValidationError('%s: `distance cost` must be a number between 0 and 10000.' % msg_prefix, params=TELEPORT_PARAM)

    def _validate_action(self, action, restricted_mutexes):
        desc_params = None
        desc_outcomes = None

        if action['skillId'].startswith('_ttpk_'):
            try:
                template_id = int(action['skillId'][6:])
                tt_inner = self.models.tmp['task_templates_assoc'][template_id]
                if not tt_inner:
                    raise RuntimeError()
            except Exception:
                raise ValidationError('task template is either inactive or has been removed.')

            desc_params = tt_inner['params']
            desc_outcomes = tt_inner['outcomes']

            if set(tt_inner['mutexes']).intersection(restricted_mutexes):
                raise ValidationError('action must not consume the following resources: "%s".' % '", "'.join(restricted_mutexes))
        else:
            try:
                skill_class = self.robot.skill_manager.get_skill_class(action['skillId'])
                if not skill_class:
                    raise RuntimeError()
            except Exception:
                raise ValidationError('action has invalid skillId: "%s".' % action.get('skillId', ''))

            desc_params = skill_class.Meta.params
            desc_outcomes = skill_class.Meta.outcomes

            if set(skill_class.Meta.mutexes).intersection(restricted_mutexes):
                raise ValidationError('action must not consume the following resources: "%s".' % '", "'.join(restricted_mutexes))

        for param in desc_params:
            if 'name' not in param or 'type' not in param:
                raise ValidationError('Invalid skill descriptions.')

            param_name = param['name']
            param_version = param.get('version')
            param_key = '%s:%s' % (param_name, param_version) if param_version else param_name
            if 'params' not in action or param_key not in action['params']:
                raise ValidationError('action has missing param "%s".' % param_name)

            raw = action['params'][param_key]
            if isinstance(raw, six.string_types) and raw.startswith('${'):
                if raw not in self._params_dict:
                    raise ValidationError('param "%s" refers to inherited param "%s" that is unavailable.' % (param_name, raw))

                if param['type'] != self._params_dict[raw]['type']:
                    raise ValidationError('param "%s" refers to inherited param "%s" of incompatible type.' % (param_name, raw))
            elif param['type'] == 'Station':
                if raw not in self.models.tmp['station_names']:
                    raise ValidationError('param "%s" refers to invalid station "%s".' % (param_name, raw))
            elif param['type'] == 'Register':
                if not self.robot.registry.get(raw):
                    raise ValidationError('param "%s" refers to invalid register "%s".' % (param_name, raw))
            elif param['type'].startswith('v'):
                if raw not in self.models.tmp['variables_assoc']:
                    raise ValidationError('param "%s" refers to variable "%s" that is unavailable.' % (param_name, raw))
                v = self.models.tmp['variables_assoc'][raw]
                if param['type'] != v['vtype']:
                    raise ValidationError('param "%s" refers to variable "%s" of incompatible type.' % (param_name, raw))
            else:
                try:
                    if param['type'] == 'bool':
                        cooked = bool(raw)
                    elif param['type'] in ['int', 'double']:
                        cooked = int(raw) if param['type'] == 'int' else float(raw)
                        a = param.get('min')
                        b = param.get('max')
                        if a is not None and cooked < a or b is not None and cooked > b:
                            raise ValidationError('param "%s" value is out-of-range.' % param_name)
                    elif param['type'] == 'str':
                        cooked = force_text(raw)
                except ValidationError:
                    raise
                except Exception:
                    raise ValidationError('param "%s" has invalid value "%s".' % (param_name, raw))

    def assemble(self):
        self.graph = self.models.graph
        self.stations_assoc = self.models.stations_assoc

        if self.teleports:
            self._amend_graph()

    def _amend_graph(self):
        for idx, t in enumerate(self.teleports):
            j1 = self.stations_assoc[t['start']][0]
            j2 = self.stations_assoc[t['end']][0]

            kwargs = {
                'direction': Direction.NA,
                'shape': PathShape.TELEPORT,
                'teleport': idx,
                'distance': t['distance'],
            }
            self.graph.add_edge(j1, j2, **kwargs)
