from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, ExecutorMode, TaskTemplate
from django.utils.encoding import force_text
import networkx as nx
import re
import six
import ujson as json

from .validator import ValidationError, Validator


class TaskTemplateValidator(Validator):
    cache = Cache
    provides = ['task_templates']
    extra_provides = ['task_templates_assoc']

    _param_types = ['bool', 'int', 'double', 'str', 'Station', 'Register',
                    'vbool', 'vint', 'vdouble', 'vstr', 'vStation']

    def is_enabled(self, pre_validate=False):
        executor_mode = self.models.tmp['executor_mode'] if pre_validate else self.models.executor_mode
        return executor_mode == ExecutorMode.Standalone.value

    def validate(self):
        self.tts_from_db = list(TaskTemplate.objects.filter(is_active=True).prefetch_related('allowed_groups', 'allowed_users'))
        self.task_templates = []

        self._validate_metadata()
        self._construct_task_templates_assoc()
        self._validate_structure()
        self._validate_mutexes()

    def _validate_metadata(self):
        # First pass - validate metadata
        for tt in self.tts_from_db:
            tt_param = {
                'tt': {
                    'name': str(tt),
                    'id': '_ttpk_%s' % tt.pk,
                    'item': '',
                }
            }

            try:
                metadata = json.loads(tt.metadata)
                params = metadata['params']
                outcomes = metadata['outcomes']
                create_suspended = bool(metadata.get('create_suspended'))
                params_dict = {}

                # Validate task templates - params
                for p in params:
                    p_name = p['name']
                    p_type = p['type']

                    if not p_name:
                        raise ValidationError('Task template "{tt}" has invalid param name.', params=tt_param)

                    if p_type not in self._param_types:
                        raise ValidationError('Task template "{tt}" has invalid param type.', params=tt_param)

                    if p_type in ['int', 'double']:
                        a = p.get('min')
                        b = p.get('max')

                        if a is not None:
                            try:
                                a = int(a) if p_type == 'int' else float(a)
                            except Exception:
                                raise ValidationError('Task template "{tt}": param "%s" has invalid `min` value.' % p_name, params=tt_param)
                            else:
                                p['min'] = a

                        if b is not None:
                            try:
                                b = int(b) if p_type == 'int' else float(b)
                            except Exception:
                                raise ValidationError('Task template "{tt}": param "%s" has invalid `max` value.' % p_name, params=tt_param)
                            else:
                                p['max'] = b

                        if a is not None and b is not None:
                            if a > b:
                                raise ValidationError('Task template "{tt}": param "%s": `min` is larger than `max`.' % p_name, params=tt_param)

                    p_key = '${%s}' % p_name
                    if p_key in params_dict:
                        raise ValidationError('Task template "{tt}" has duplicate param name: "%s".' % p_name, params=tt_param)
                    params_dict[p_key] = p

                # Validate task templates - outcomes
                if tt.is_top_level:
                    if len(outcomes) != 1 or outcomes[0] != 'End':
                        raise ValidationError('Task template "{tt}" is a top-level template, so it should have only one outcome "End".', params=tt_param)
                else:
                    if not outcomes:
                        raise ValidationError('Task template "{tt}" has no outcome.', params=tt_param)
                    for o in outcomes:
                        if not o or not isinstance(o, six.string_types):
                            raise ValidationError('Task template "{tt}" has invalid outcomes.', params=tt_param)
                        if o == 'Preempted':
                            raise ValidationError('Task template "{tt}" must not have "Preempted" as outcome, because it is a reserved outcome.', params=tt_param)
                        if o == 'Aborted':
                            raise ValidationError('Task template "{tt}" must not have "Aborted" as outcome, because it is a reserved outcome.', params=tt_param)
                        if re.match(r'^-?\d+$', o):
                            raise ValidationError('Task template "{tt}" must not have an integer as outcome.', params=tt_param)

                self.task_templates.append({
                    'id': tt.id,
                    'name': tt.name,
                    'params': params,
                    'outcomes': outcomes,
                    'create_suspended': create_suspended,
                    'is_top_level': tt.is_top_level,
                    'allowed_groups': [g.name for g in tt.allowed_groups.all()],
                    'allowed_users': [u.username for u in tt.allowed_users.all()],
                })

                tt.params_dict = params_dict  # Store params_dict for use in second pass

            except ValidationError:
                raise
            except Exception:
                raise ValidationError('Task template "{tt}" data might have corrupted.', params=tt_param)

    def _validate_structure(self):
        # Second pass - validate structure
        self.tts_graph = nx.DiGraph()
        for tt in self.tts_from_db:
            tt_param = {
                'tt': {
                    'name': str(tt),
                    'id': '_ttpk_%s' % tt.pk,
                    'item': '',
                }
            }

            try:
                self.tts_graph.add_node(tt.id)
                outcomes = self.task_templates_assoc[tt.id]['outcomes']
                params_dict = tt.params_dict
                for p in self.models.tmp['global_params']:
                    params_dict.setdefault('${%s}g' % p['name'], p)
                for v in self.models.tmp['variables']:
                    params_dict.setdefault('${%s}v' % v['name'], v)

                structure = json.loads(tt.structure)
                actions = structure['actions']
                action_count = structure['action_count']

                # Validate task templates - actions
                if not actions or not isinstance(actions[0], int) or len(actions) <= 1:
                    raise ValidationError('Task template "{tt}" has no actions defined.', params=tt_param)

                if action_count != len(actions):
                    raise ValidationError('Task template "{tt}": action count is inaccurate.', params=tt_param)

                if actions[0] < -len(outcomes) or actions[0] >= action_count:
                    raise ValidationError('Task template "{tt}": action index out of bounds.', params=tt_param)

                # pre construct action graph.
                action_graph = nx.DiGraph()
                action_graph.add_edge(0, actions[0])
                for action_node, action in enumerate(actions[1:], start=1):
                    action_graph.add_node(action_node)
                    for value in six.itervalues(action['outcomes']):
                        if value is None:
                            # will be triggered later as missing outcome
                            continue
                        next_action_node = int(value)
                        action_graph.add_edge(action_node, next_action_node)

                for outcome_idx, outcome in enumerate(outcomes, start=1):
                    outcome_node = -(outcome_idx)
                    if not action_graph.has_node(outcome_node) or not nx.has_path(action_graph, 0, outcome_node):
                        raise ValidationError('Task template "{tt}": outcome "%s" is unreachable.' % outcome, params=tt_param)
                    # connect all outcomes to dummy terminal node [-1].
                    action_graph.add_edge(outcome_node, -1)

                for action_node, action in enumerate(actions[1:], start=1):
                    desc_name = None
                    desc_params = None
                    desc_outcomes = None

                    if action['skillId'].startswith('_ttpk_'):
                        try:
                            template_id = int(action['skillId'][6:])
                            tt_inner = self.task_templates_assoc[template_id]
                            if not tt_inner:
                                raise RuntimeError()
                        except Exception:
                            raise ValidationError('Task template "{tt}": nested task template is either inactive or has been removed.', params=tt_param)

                        self.tts_graph.add_edge(tt.id, tt_inner['id'])
                        desc_name = tt_inner['name']
                        desc_params = tt_inner['params']
                        desc_outcomes = tt_inner['outcomes']
                    else:
                        try:
                            skill_class = self.robot.skill_manager.get_skill_class(action['skillId'])
                            if not skill_class:
                                raise RuntimeError()
                        except Exception:
                            raise ValidationError('Task template "{tt}": action has invalid skillId: "%s".' % action.get('skillId', ''),
                                                  params=tt_param)

                        desc_name = skill_class.Meta.name
                        desc_params = skill_class.Meta.params
                        desc_outcomes = skill_class.Meta.outcomes

                    for param in desc_params:
                        if 'name' not in param or 'type' not in param:
                            raise ValidationError('Invalid skill descriptions.')

                        param_name = param['name']
                        param_version = param.get('version')
                        param_key = '%s:%s' % (param_name, param_version) if param_version else param_name
                        if 'params' not in action or param_key not in action['params']:
                            raise ValidationError('Task template "{tt}": action "%s" has missing param "%s".' % (desc_name, param_name), params=tt_param)

                        raw = action['params'][param_key]
                        if isinstance(raw, six.string_types) and raw.startswith('${'):
                            if raw not in params_dict:
                                raise ValidationError(
                                    'Task template "{tt}": action "%s": param "%s" refers to inherited param "%s" that is unavailable.' % (desc_name, param_name, raw,),
                                    params=tt_param
                                )

                            p = params_dict[raw]
                            types = [p['type']]
                            if p['type'].startswith('v'):
                                types.append(p['type'][1:])
                            if param['type'] not in types:
                                raise ValidationError(
                                    'Task template "{tt}": action "%s": param "%s" refers to inherited param "%s" of incompatible type.' % (desc_name, param_name, raw),
                                    params=tt_param
                                )

                        elif param['type'] == 'Station':
                            if raw not in self.models.tmp['station_names']:
                                raise ValidationError('Task template "{tt}": action "%s": param "%s" refers to invalid station "%s".' % (desc_name, param_name, raw), params=tt_param)
                        elif param['type'] == 'Register':
                            if not self.robot.registry.get(raw):
                                raise ValidationError('Task template "{tt}": action "%s": param "%s" refers to invalid register "%s".' % (desc_name, param_name, raw), params=tt_param)
                        elif param['type'].startswith('v'):
                            if raw not in self.models.tmp['variables_assoc']:
                                raise ValidationError('Task template "{tt}": action "%s": param "%s" refers to invalid variable "%s".' % (desc_name, param_name, raw), params=tt_param)
                            v = self.models.tmp['variables_assoc'][raw]
                            if param['type'] != v['vtype']:
                                raise ValidationError(
                                    'Task template "{tt}": action "%s": param "%s" refers to variable "%s" of incompatible type.' % (desc_name, param_name, raw),
                                    params=tt_param
                                )
                        else:
                            try:
                                if param['type'] == 'bool':
                                    cooked = bool(raw)
                                elif param['type'] in ['int', 'double']:
                                    cooked = int(raw) if param['type'] == 'int' else float(raw)
                                    a = param.get('min')
                                    b = param.get('max')
                                    if a is not None and cooked < a or b is not None and cooked > b:
                                        raise ValidationError('Task template "{tt}": action "%s": param "%s" value is out-of-range.' % (desc_name, param_name), params=tt_param)
                                elif param['type'] == 'str':
                                    cooked = force_text(raw)
                            except ValidationError:
                                raise
                            except Exception:
                                raise ValidationError('Task template "{tt}": action "%s": param "%s" has invalid value "%s".' % (desc_name, param_name, raw), params=tt_param)

                    for outcome in desc_outcomes:
                        if 'outcomes' not in action or outcome not in action['outcomes'] or \
                                not action['outcomes'][outcome]:
                            raise ValidationError('Task template "{tt}": action "%s" has missing outcomes.' % desc_name, params=tt_param)

                        next_action = int(action['outcomes'][outcome])
                        min_action = -2 if tt.is_top_level else -len(outcomes)
                        if next_action < min_action or next_action >= action_count:
                            raise ValidationError('Task template "{tt}": action index out of bounds.', params=tt_param)

                    if not action_graph.has_node(action_node) or not nx.has_path(action_graph, 0, action_node):
                        raise ValidationError('Task template "{tt}": action "%s" is unreachable.' % desc_name, params=tt_param)
                    if not nx.has_path(action_graph, action_node, -1):
                        raise ValidationError('Task template "{tt}": infinite loop detected for action "%s".' % desc_name, params=tt_param)

                self.task_templates_assoc[tt.id]['actions'] = actions

            except ValidationError:
                raise
            except Exception:
                raise ValidationError('Task template "{tt}" data might have corrupted.', params=tt_param)

            # Validate that task template structure is non-recursive
            if nx.number_strongly_connected_components(self.tts_graph) != len(self.tts_graph):
                raise ValidationError('Nested task templates may not contain direct or indirect recursive loop onto the task template itself.')

    def _validate_mutexes(self):
        # Todo: Third pass - validate mutexes in concurrency containers
        for v in nx.dfs_postorder_nodes(self.tts_graph):
            tt = self.task_templates_assoc[v]
            tt['mutexes'] = set()
            for action in tt['actions'][1:]:
                if action['skillId'].startswith('_ttpk_'):
                    template_id = int(action['skillId'][6:])
                    tt_inner = self.task_templates_assoc[template_id]
                    tt['mutexes'].update(tt_inner['mutexes'])
                else:
                    skill_class = self.robot.skill_manager.get_skill_class(action['skillId'])
                    tt['mutexes'].update(skill_class.Meta.mutexes)
            tt['mutexes'] = sorted(list(tt['mutexes']))

    def provide_reloadable(self, provide, cur, tmp):
        if provide == 'task_templates':
            # allow task_templates reloadable
            return True

        return super(TaskTemplateValidator, self).provide_reloadable(provide, cur, tmp)

    def hot_reload(self, downloadables=None):
        if downloadables is None:
            self.apply()
        else:
            self.from_downloadables(downloadables)

    def assemble(self):
        self._construct_task_templates_assoc()

        # from_downloadables
        for tt in self.task_templates:
            if 'allowed_agvs' in tt:
                tt['allowed_groups'] = []

                if self.models.agv_uuid in tt['allowed_agvs']:
                    if tt['allowed_agvs'][self.models.agv_uuid][0]:
                        tt['allowed_users'] = '*'
                    else:
                        tt['allowed_users'] = []
                else:
                    tt['is_top_level'] = False

                del tt['allowed_agvs']

    def _construct_task_templates_assoc(self):
        self.task_templates_assoc = {
            tt['id']: tt for tt in self.task_templates
        }
