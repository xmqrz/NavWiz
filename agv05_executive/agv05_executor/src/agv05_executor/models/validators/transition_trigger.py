from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, Direction, ExecutorMode, Variable
from django.utils.encoding import force_text
import six
import ujson as json

from .validator import ValidationError, Validator

TT_PARAM = {
    'tran_trigger': {
        'name': 'Transition Trigger',
        'id': 'map-transition-trigger',
        'item': '',
    }
}


class TransitionTriggerValidator(Validator):
    cache = Cache
    provides = ['transition_triggers']

    HEADING_NA = Direction.NA
    HEADING_WARNING = 'directionless'

    transition_trigger_params = [{
        'name': 'start',
        'type': 'Station',
    }, {
        'name': 'end',
        'type': 'Station',
    }, {
        'name': 'next_motion',
        'type': 'int',
    }]

    def is_enabled(self, pre_validate=False):
        executor_mode = self.models.tmp['executor_mode'] if pre_validate else self.models.executor_mode
        return executor_mode == ExecutorMode.Standalone.value

    def validate(self):
        self.transition_triggers_list = []
        self.transition_triggers = dict()
        self.stations_assoc = self.models.tmp['stations_assoc']

        try:
            transition_trigger = Variable.objects.get(pk=Variable.TRANSITION_TRIGGER).value
        except Exception:
            return

        if not transition_trigger:
            return

        try:
            self.transition_triggers_list = json.loads(transition_trigger)
            assert isinstance(self.transition_triggers_list, list)
        except Exception:
            raise ValidationError('{tran_trigger} data might have corrupted.', params=TT_PARAM)

        try:
            self._validate_transition_trigger()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('{tran_trigger} data might have corrupted.', params=TT_PARAM)

    def _validate_transition_trigger(self):
        self._params_dict = {
            '${%s}' % p['name']: p for p in self.transition_trigger_params
        }
        for p in self.models.tmp['global_params']:
            self._params_dict.setdefault('${%s}g' % p['name'], p)
        for v in self.models.tmp['variables']:
            self._params_dict.setdefault('${%s}v' % v['name'], v)

        transitiontriggers_map = {}
        for idx, t in enumerate(self.transition_triggers_list):
            msg_prefix = '{tran_trigger} #%d' % (idx + 1)
            if t['start'] not in self.models.tmp['station_names']:
                raise ValidationError('%s: start point refers to invalid station "%s".' %
                    (msg_prefix, t['start']), params=TT_PARAM)

            if t['end'] not in self.models.tmp['station_names']:
                raise ValidationError('%s: end point refers to invalid station "%s".' %
                    (msg_prefix, t['end']), params=TT_PARAM)

            start_location = self.stations_assoc[t['start']]
            end_location = self.stations_assoc[t['end']]

            if start_location[0] != end_location[0]:
                raise ValidationError('%s: start and end points must be on the same junction.' % msg_prefix, params=TT_PARAM)

            if start_location[1] == self.HEADING_NA:
                raise ValidationError('%s: start point must not be %s.' % (msg_prefix, self.HEADING_WARNING), params=TT_PARAM)

            if end_location[1] == self.HEADING_NA:
                raise ValidationError('%s: end point must not be %s.' % (msg_prefix, self.HEADING_WARNING), params=TT_PARAM)

            try:
                if t['startAction']['skillId']:
                    self._validate_action(t['startAction'], ['plan', 'traffic'])
                else:
                    t['startAction'] = None
            except ValidationError as ex:
                params = TT_PARAM.copy()
                params.update(ex.params)
                raise ValidationError('%s start-action: %s' % (msg_prefix, ex), params=params)

            try:
                if t['endAction']['skillId']:
                    self._validate_action(t['endAction'], ['plan', 'traffic'])
                else:
                    t['endAction'] = None
            except ValidationError as ex:
                params = TT_PARAM.copy()
                params.update(ex.params)
                raise ValidationError('%s end-action: %s' % (msg_prefix, ex), params=params)

            try:
                t['applicableMotion'] = [int(x) for x in t['applicableMotion']]
                for selection in t['applicableMotion']:
                    if selection not in [0, 1, 2, 3, 4]:
                        raise RuntimeError()
            except Exception:
                raise ValidationError('%s: `applicable motion` has invalid value.' % msg_prefix, params=TT_PARAM)

            tt = (t['start'], t['end'])
            testset = set(t['applicableMotion'])
            if tt in transitiontriggers_map:
                for k, v in transitiontriggers_map[tt].items():
                    if testset.intersection(v):
                        raise ValidationError('%s: start point, end point and applicable motion overlap with Transition Trigger #%d.' % (msg_prefix, k + 1), params=TT_PARAM)
            else:
                transitiontriggers_map[tt] = {}
            transitiontriggers_map[tt][idx] = testset

            try:
                t['cancelNonStopTransition'] = bool(t['cancelNonStopTransition'])
            except Exception:
                raise ValidationError('%s: `cancel-non-stop transition` has invalid value.' % msg_prefix, params=TT_PARAM)

            # Add to dictionary
            junction = start_location[0]
            if junction not in self.transition_triggers:
                self.transition_triggers[junction] = []
            transition_trigger = self.transition_triggers[junction]
            transition_trigger.append({
                'start': t['start'],
                'end': t['end'],
                'startAction': t['startAction'],
                'endAction': t['endAction'],
                'applicableMotion': t['applicableMotion'],
                'cancelNonStopTransition': t['cancelNonStopTransition'],
                'previous_heading': start_location[1],
                'next_heading': end_location[1],
                'transition_trigger_idx': idx
            })
            self.transition_triggers[junction] = transition_trigger

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
