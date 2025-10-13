from __future__ import absolute_import
from __future__ import unicode_literals

from django.utils.encoding import force_text
import copy
import rospy
import six
import smach
import smach_ros  # noqa : F401 - import to redirect smach loggers to rospy loggers

from .fms_skill import fms_skill_factory, FmsSkill
from .register import Register


def __is_paused(trigger=False):
    return False


def set_pause_check(fn):
    if fn and callable(fn):
        smach.is_paused = fn
    else:
        smach.is_paused = __is_paused


set_pause_check(None)


class StateConstructError(RuntimeError):
    pass


class StateMachine(smach.StateMachine):

    def __init__(self, robot, models, skill_id, params, actions, outcomes, *args, **kwargs):
        self.robot = robot
        self.models = copy.copy(models) if models == robot.models else models
        self.skill_id = skill_id
        self.params = params
        self.actions = actions
        self.sm_outcomes = outcomes
        self._action_cb = None
        super(StateMachine, self).__init__(outcomes, *args, **kwargs)

    def execute(self, *args, **kwargs):
        if not self._states:
            state_machine_factory_bottom_half(self)
        return super(StateMachine, self).execute(*args, **kwargs)

    def register_action_cb(self, callback, label='1', call_stack=[]):
        self._cs = call_stack + [(label, self.skill_id)]
        self._action_cb = callback

    def _update_once(self):
        if self.preempt_requested():
            return 'Preempted'
        if smach.is_paused():
            try:
                self._state_transitioning_lock.release()
                rospy.sleep(0.2)
            except Exception:
                pass
            finally:
                self._state_transitioning_lock.acquire()
            return

        # update variable state
        state = self._current_state
        if isinstance(state, StateMachine):
            for k, v in state.params['__variables'].items():
                v.update_derived_value()
                # TODO: some param got min max value. should check and trigger error here.
        elif isinstance(state, FmsSkill):
            for k, v in state.params['__variables'].items():
                state.params[k] = v.value
        else:
            for k, v in getattr(state, '__variables').items():
                # TODO: some param got min max value. should check and trigger error here.
                setattr(state, k, v.value)

        # call action callback
        label = self._current_label
        if self._action_cb:
            if isinstance(state, StateMachine):
                state.register_action_cb(self._action_cb, label, self._cs)
            else:
                self._action_cb(state, self._cs + [(label, state.skill_id)])

        return super(StateMachine, self)._update_once()


def state_factory(robot, skill_id, skill_params, inherited_params=None, **kwargs):
    inherited_params = inherited_params or {}
    models = kwargs.get('__cached_models') or robot.models
    template_id = None
    skill_class = None
    desc_params = None

    if skill_id.startswith('_ttpk_'):
        try:
            template_id = int(skill_id[6:])
            template = models.task_templates_assoc[template_id]
            assert template
        except Exception:
            raise StateConstructError('Nested task template is unavailable.')

        desc_params = template['params']

    elif skill_id.startswith('fms:'):
        try:
            skill_class = fms_skill_factory(models.fms_skill_descriptions_assoc[skill_id])
            assert skill_class
        except Exception:
            raise StateConstructError('DFleet skill "%s" is unavailable.' % skill_id)

        desc_params = skill_class.Meta.params

    else:
        try:
            skill_class = robot.skill_manager.get_skill_class(skill_id)
            assert skill_class
        except Exception:
            raise StateConstructError('Invalid skill id: %s' % skill_id)

        desc_params = skill_class.Meta.params

    init_kwargs = {}
    variables_state = {}
    init_kwargs['__variables'] = variables_state
    inherited_params.setdefault('${agv_home}g', models.agv_home)
    for p in models.global_params or []:
        if 'default' not in p:
            continue
        inherited_params.setdefault('${%s}g' % p['name'], p['default'])

    for vid, v in robot.variable.get_variables():
        inherited_params.setdefault('${%s}v' % vid, v)

    for param in desc_params:
        if 'name' not in param or 'type' not in param:
            raise StateConstructError('Invalid skill param desc.')

        name = param['name']
        version = param.get('version')
        key = '%s:%s' % (name, version) if version else name
        if key not in skill_params:
            raise StateConstructError('Missing param: %s' % name)
        raw = skill_params[key]

        if isinstance(raw, six.string_types) and raw.startswith('${'):
            if raw not in inherited_params:
                raise StateConstructError('Reference to missing param: %s' % raw)
            raw = inherited_params[raw]

        if robot.variable.is_variable(raw) and not param['type'].startswith('v'):
            if not raw.is_type(param['type']):
                raise StateConstructError('Mismatch variable type reference for param "%s"' % name)
            if skill_class:
                init_kwargs[name] = raw.value
                variables_state[name] = raw
            elif raw.is_derived:
                init_kwargs[name] = raw
            else:
                raw = raw.derive()
                init_kwargs[name] = raw
                variables_state[name] = raw
        elif param['type'] == 'bool':
            init_kwargs[name] = bool(raw)
        elif param['type'] == 'int':
            init_kwargs[name] = int(raw)
        elif param['type'] == 'double':
            init_kwargs[name] = float(raw)
        elif param['type'] == 'str':
            init_kwargs[name] = force_text(raw)
        elif param['type'] == 'Station':
            init_kwargs[name] = force_text(raw)
        elif param['type'] == 'Register':
            init_kwargs[name] = raw if isinstance(raw, Register) else robot.registry.get(raw)
        elif param['type'] == 'FmsRegister':
            init_kwargs[name] = force_text(raw)
        elif param['type'].startswith('v'):
            v = raw
            if not robot.variable.is_variable(v):
                v = robot.variable.get(raw)
            if not v or not v.is_vtype(param['type']):
                raise StateConstructError('Missing variable for param "%s"' % name)
            init_kwargs[name] = v

    try:
        if skill_class:
            return skill_class(robot, **init_kwargs)
        else:
            return state_machine_factory(robot, template_id, init_kwargs, __cached_models=models)
    except StateConstructError:
        raise
    except Exception as ex:
        raise StateConstructError('Exception: %s' % ex)


def state_machine_factory(robot, template_id, params, top_level=False, **kwargs):
    models = kwargs.get('__cached_models') or robot.models
    try:
        template = models.task_templates_assoc[template_id]
        assert template
    except Exception:
        raise StateConstructError('Task template (id=%d) is unavailable.' % template_id)

    variables = params.pop('__variables', {})

    # transforming params so names are enclosed in '${}'.
    if not top_level:
        params = {'${%s}' % k: v for k, v in params.items()}
    else:
        validated_params = {}
        for p in template['params']:
            name = p['name']
            if name not in params:
                raise StateConstructError('Missing param "%s"' % p['name'])
            raw = params[p['name']]

            # preprocess variable here since can inherit by non variable.
            if p['type'].startswith('v'):
                if not robot.variable.is_variable(raw):
                    raw = robot.variable.get(raw)
                if not raw or not raw.is_vtype(p['type']):
                    raise StateConstructError('Missing variable for param "%s"' % name)

            validated_params['${%s}' % name] = raw

        params = validated_params

    params['__variables'] = variables

    actions = template['actions']
    if not actions or not isinstance(actions[0], int) or len(actions) <= 1:
        raise StateConstructError('No actions defined in task template.')

    try:
        if top_level:
            sm_outcomes = ['Completed', 'Aborted']
        else:
            sm_outcomes = template['outcomes'] + ['Preempted', 'Aborted']
        return StateMachine(robot, models, '_ttpk_%s' % template_id, params, actions, sm_outcomes)
    except Exception as ex:
        raise StateConstructError('Exception: %s' % ex)


def state_machine_factory_bottom_half(sm):
    robot = sm.robot
    models = sm.models
    params = sm.params
    actions = sm.actions
    sm_outcomes = sm.sm_outcomes

    initial_state = force_text(actions[0])

    with sm.opened():
        for idx in range(1, len(actions)):
            action = actions[idx]
            try:
                action_skillId = action['skillId']
                action_params = action['params']
                action_outcomes = action['outcomes']
            except Exception:
                raise StateConstructError('Invalid task template data.')

            state = state_factory(robot, action_skillId, action_params, params, __cached_models=models)

            transitions = {}
            for outcome in state._outcomes:
                if outcome not in action_outcomes:
                    # implicit transition to Aborted state for Preempted, Aborted, and other unknown outcomes.
                    raw = -len(sm_outcomes)
                else:
                    raw = action_outcomes[outcome]

                if int(raw) < 0 and int(raw) >= -len(sm_outcomes):
                    transitions[outcome] = sm_outcomes[~int(raw)]
                else:
                    transitions[outcome] = force_text(raw)

            try:
                StateMachine.add(str(idx), state, transitions=transitions)
            except Exception as ex:
                raise StateConstructError('Exception: %s' % ex)

    sm.set_initial_state([initial_state])
