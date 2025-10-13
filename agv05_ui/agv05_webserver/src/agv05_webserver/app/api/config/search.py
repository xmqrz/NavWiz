from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.conf import settings as django_settings
from django.utils.functional import cached_property
from rest_framework import viewsets, status
from rest_framework.response import Response
import ujson as json

from ...serializers import SearchSerializer
from ..mixin import Permission
from .fms_void import FmsVoidMixin
from .license_void import LicenseVoidMixin
from agv05_webserver.system.models import Variable, Cache, TaskTemplate, Map, MapChangeset


# TODO: map param search. display where it defined and where it used in map..
class SearchViewSet(FmsVoidMixin, LicenseVoidMixin, viewsets.ViewSet):
    permission_classes = Permission('system.view_system_panel')
    serializer_class = SearchSerializer

    def list(self, request, *args, **kwargs):
        if not self.request.user.has_perm('system.view_system_panel'):
            self.permission_denied(self.request)

        choices = []

        # built-in skills
        try:
            skillset = json.loads(Cache.get_models_skillset())
            builtins = skillset['skill_descriptions']
            if builtins:
                choices.append(('Built-in Action',
                                [('_a_%s' % s['id'], s['name']) for s in builtins]))
        except Exception:
            pass

        # task template
        tasktemplate = TaskTemplate.objects.values('id', 'name').order_by('is_top_level', '-is_active', 'name')
        if tasktemplate:
            choices.append(('Task Template',
                            [('_ttpk_%s' % tt['id'], tt['name']) for tt in tasktemplate]))

        # global params
        gp_choices = []
        # reserved global params
        gp_choices.append(('_g_agv_home', 'agv_home'))

        try:
            global_params = json.loads(Variable.objects.get(name=Variable.GLOBAL_PARAM).value)
            if global_params:
                gp_choices.extend(('_g_%s' % g['name'], g['name']) for g in global_params)
        except Exception:
            pass

        choices.append(('Global Parameter', gp_choices))

        # variables
        try:
            variables = json.loads(Variable.objects.get(name=Variable.VARIABLE).value)
            if variables:
                choices.append(('Variable',
                                [('_v_%s' % v['name'], v['name']) for v in variables]))
        except Exception:
            pass

        # stations
        try:
            stations = set()
            maps = [m['id'] for m in Map.objects.all().values('id')]
            for m in maps:
                mc = MapChangeset.objects.filter(map_id=m).first()
                if mc:
                    stations.update(json.loads(mc.stations))
            stations = sorted(stations)

            if stations:
                choices.append(('Station',
                                [('_Station_%s' % s, s) for s in stations]))
        except Exception:
            pass

        # register list
        try:
            skillset = json.loads(Cache.get_models_skillset())
            register_list = skillset['register_list']
            if register_list:
                choices.append(('Register',
                                [('_Register_%s' % r, r) for r in register_list]))
        except Exception:
            pass

        return Response(OrderedDict([
            ('choices', choices),
        ]))

    def create(self, request, *args, **kwargs):
        serializer = SearchSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        search = serializer.data['search']

        search_results = []
        try:
            search_results = self.get_search_results(search)
        except Exception as ex:
            if django_settings.DEBUG:
                raise
            err_msg = 'Search result error'
            return Response({'result': False, 'message': err_msg}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        return Response({
            'result': True,
            'data': search_results,
            'search': search,
        }, status=status.HTTP_200_OK)

    @cached_property
    def skill_descriptions(self):
        data = {}
        skill_descriptions = {}
        try:
            skillset = json.loads(Cache.get_models_skillset())
            skill_descriptions = {s['id']: s for s in skillset['skill_descriptions']}
        except Exception:
            pass

        return skill_descriptions

    @cached_property
    def tasktemplate_metas(self):
        # NOTE: call clear once finish search to ensure using latest data.

        tasktemplate_metas = TaskTemplate.objects.values('id', 'name', 'metadata', 'is_active', 'is_top_level').order_by('is_top_level', '-is_active', 'name')
        tasktemplate_metas = {'_ttpk_%s' % t['id']: t for t in tasktemplate_metas}

        return tasktemplate_metas

    def clear_tasktemplate_metas(self):
        try:
            del self.tasktemplate_metas
        except Exception:
            pass

    def _get_skill_desc(self, skill_id):
        try:
            if skill_id.startswith('_ttpk_'):
                skill_desc = self.tasktemplate_metas[skill_id]
                metadata = json.loads(skill_desc['metadata'])
                return {
                    'params': metadata['params']
                }
            else:
                skill_desc = self.skill_descriptions[skill_id]
                return {
                    'params': [json.loads(p) for p in skill_desc['params']]
                }
        except Exception:
            pass

    def _iter_action_param_val(self, action):
        if not action or not action.get('skillId') or not action.get('params'):
            return
        skill_desc = self._get_skill_desc(action['skillId'])
        if not skill_desc:
            return
        for p in skill_desc['params']:
            v = action['params'].get(p['name'])
            yield p, v

    def search_station_in_task_template(self, action, station):
        for p, v in self._iter_action_param_val(action):
            if p['type'] == 'Station' and v == station:
                return True

    def search_register_in_task_template(self, action, register):
        for p, v in self._iter_action_param_val(action):
            if p['type'] == 'Register' and v == register:
                return True

    def search_variable_in_task_template(self, action, variable):
        for p, v in self._iter_action_param_val(action):
            if v == '${%s}v' % variable:
                return True
            if v == variable and p['type'].startswith('v'):
                return True

    def search_global_param_in_task_template(self, action, gp):
        for p, v in self._iter_action_param_val(action):
            if v == '${%s}g' % gp:
                return True

    def get_search_results(self, search):
        try:
            if search.startswith('_Station_'):
                return self.search_station(search)
            elif search.startswith('_Register_'):
                return self.search_register(search)
            elif search.startswith('_g_'):
                return self.search_global_param(search)
            elif search.startswith('_v_'):
                return self.search_variable(search)
            elif search.startswith('_ttpk_'):
                return self.search_task_template(search)
            elif search.startswith('_a_'):
                return self.search_skill(search)
        finally:
            self.clear_tasktemplate_metas()

    def search_station(self, id):
        results = []

        station = id[9:]

        # Task Template
        for t in TaskTemplate.objects.all():
            structure = json.loads(t.structure)
            for a in structure['actions'][1:]:
                if self.search_station_in_task_template(a, station):
                    results.append(self._task_template_entry('Station used in task template: ', t, id))
                    break

        # Maps
        try:
            active_maps = [int(v) for v in Variable.objects.get(pk=Variable.ACTIVE_MAP).value.split(',')]
            for map in Map.objects.all():
                mc = MapChangeset.objects.filter(map_id=map.pk).first()
                if mc:
                    stations = json.loads(mc.stations)
                    if station in stations:
                        if int(map.id) in active_maps:
                            results.append(self._map_entry('Station found in ACTIVE map: ', map, id))
                        else:
                            results.append(self._map_entry('Station found in inactive map: ', map, id))
        except Exception:
            pass

        # Agvs
        try:
            agv_home = Variable.objects.get(name=Variable.AGV_HOME).value
            if agv_home == station:
                results.append(self._agv_entry('Station used as agv home: ', 'AGV_HOME'))
        except Exception:
            pass

        try:
            def search_station_in_task_trigger(data, t_type):
                if station in data['ignore_stations']:
                    results.append(self._agv_entry('Station in task trigger configuration: ', '%s_TASK_TRIGGER' % t_type))
                    return

                if data.get('action'):
                    action = data['action']
                    skill_desc = self._get_skill_desc(action['skillId'])
                    if not skill_desc:
                        return
                    for p in skill_desc['params']:
                        if p['type'] != 'Station':
                            continue
                        v = action['params'].get(p['name'])
                        if v != station:
                            continue
                        results.append(self._agv_entry('Station in task trigger configuration: ', '%s_TASK_TRIGGER' % t_type))
                        return

            cfg = json.loads(Variable.objects.get(name=Variable.EXECUTOR_CFG).value)
            task_triggers = cfg.get('task_triggers')
            if task_triggers:
                agv_idle = task_triggers.get('agv_idle')
                if agv_idle:
                    search_station_in_task_trigger(agv_idle, 'AGV_IDLE')
                battery_low = task_triggers.get('battery_low')
                if battery_low:
                    search_station_in_task_trigger(battery_low, 'BATTERY_LOW')

        except Exception:
            pass

        # Teleport
        try:
            v = Variable.objects.get(name=Variable.TELEPORT)
            teleport = json.loads(v.value)
            for t in teleport:
                if t.get('start') == station:
                    results.append(self._teleport_entry('Station used as start station of map teleport: ', id))
                    break
                if t.get('end') == station:
                    results.append(self._teleport_entry('Station used as end station of map teleport: ', id))
                    break

                if t.get('preAction') and self.search_station_in_task_template(t['preAction'], station):
                    results.append(self._teleport_entry('Station used in map teleport preAction: ', id))
                    break

                if t.get('action') and self.search_station_in_task_template(t['action'], station):
                    results.append(self._teleport_entry('Station used in map teleport action: ', id))
                    break

        except Exception:
            pass

        # Transition trigger
        try:
            v = Variable.objects.get(name=Variable.TRANSITION_TRIGGER)
            transition_trigger = json.loads(v.value)
            for t in transition_trigger:
                if t.get('start') == station:
                    results.append(self._transition_trigger_entry('Station used as start station of map transition trigger: ', id))
                    break
                if t.get('end') == station:
                    results.append(self._transition_trigger_entry('Station used as end station of map transition trigger: ', id))
                    break

                if t.get('startAction') and self.search_station_in_task_template(t['startAction'], station):
                    results.append(self._transition_trigger_entry('Station used in map transition trigger start action: ', id))
                    break

                if t.get('endAction') and self.search_station_in_task_template(t['endAction'], station):
                    results.append(self._transition_trigger_entry('Station used in map transition trigger end action: ', id))
                    break

        except Exception:
            pass

        # Global Params
        try:
            v = Variable.objects.get(name=Variable.GLOBAL_PARAM)
            global_params = json.loads(v.value)

            for gp in global_params:
                if gp.get('type') != 'Station':
                    continue
                if gp.get('default') == station:
                    results.append(self._gp_entry('Station used for Global Parameter: ', id))
                    break
        except Exception:
            pass

        # Variable
        try:
            v = Variable.objects.get(name=Variable.VARIABLE)
            variables = json.loads(v.value)

            for variable in variables:
                if variable.get('type') == 'Station' and \
                        variable.get('default') == station:
                    results.append(self._variable_entry('Station used for Variable: ', id))
        except Exception:
            pass

        return results

    def search_register(self, id):
        results = []

        register = id[10:]

        # Task Template
        for t in TaskTemplate.objects.all():
            structure = json.loads(t.structure)
            for a in structure['actions'][1:]:
                if self.search_register_in_task_template(a, register):
                    results.append(self._task_template_entry('Register used in task template: ', t, id))
                    break

        # Agvs
        try:
            def search_register_in_task_trigger(data, t_type):
                if not data.get('action'):
                    return

                action = data['action']
                skill_desc = self._get_skill_desc(action['skillId'])
                if not skill_desc:
                    return
                for p in skill_desc['params']:
                    if p['type'] != 'Register':
                        continue
                    v = action['params'].get(p['name'])
                    if v != register:
                        continue
                    results.append(self._agv_entry('Register used in task trigger configuration: ', '%s_TASK_TRIGGER' % t_type))
                    return

            cfg = json.loads(Variable.objects.get(name=Variable.EXECUTOR_CFG).value)
            task_triggers = cfg.get('task_triggers')
            if task_triggers:
                agv_idle = task_triggers.get('agv_idle')
                if agv_idle:
                    search_register_in_task_trigger(agv_idle, 'AGV_IDLE')
                battery_low = task_triggers.get('battery_low')
                if battery_low:
                    search_register_in_task_trigger(battery_low, 'BATTERY_LOW')
        except Exception:
            pass

        # Teleport
        try:
            v = Variable.objects.get(name=Variable.TELEPORT)
            teleport = json.loads(v.value)
            for t in teleport:
                if t.get('preAction') and self.search_register_in_task_template(t['preAction'], register):
                    results.append(self._teleport_entry('Register used in map teleport preAction: ', id))
                    break

                if t.get('action') and self.search_register_in_task_template(t['action'], register):
                    results.append(self._teleport_entry('Register used in map teleport action: ', id))
                    break
        except Exception:
            pass

        # Transition trigger
        try:
            v = Variable.objects.get(name=Variable.TRANSITION_TRIGGER)
            transition_trigger = json.loads(v.value)
            for t in transition_trigger:
                if t.get('startAction') and self.search_register_in_task_template(t['startAction'], register):
                    results.append(self._transition_trigger_entry('Register used in map transition trigger start action: ', id))
                    break

                if t.get('endAction') and self.search_register_in_task_template(t['endAction'], register):
                    results.append(self._transition_trigger_entry('Register used in map transition trigger end action: ', id))
                    break
        except Exception:
            pass

        # Global Params
        try:
            v = Variable.objects.get(name=Variable.GLOBAL_PARAM)
            global_params = json.loads(v.value)

            for gp in global_params:
                if gp.get('type') != 'Register':
                    continue
                if gp.get('default') == register:
                    results.append(self._gp_entry('Register used for Global Parameter: ', id))
                    break
        except Exception:
            pass

        return results

    def search_global_param(self, id):
        gp = id[3:]

        results = []

        # Global Params
        if gp == 'agv_home':
            results.append(('Task template global param: ', gp, 'task-template-global-param', ''))
        else:
            try:
                v = Variable.objects.get(name=Variable.GLOBAL_PARAM)
                global_params = json.loads(v.value)

                for g in global_params:
                    if g.get('name') == gp:
                        results.append(('Task template global param: ', gp, 'task-template-global-param', ''))
                        break
            except Exception:
                pass

        # Task Template
        for t in TaskTemplate.objects.all():
            structure = json.loads(t.structure)
            for a in structure['actions'][1:]:
                if self.search_global_param_in_task_template(a, gp):
                    results.append(self._task_template_entry('Global Parameter used in task template: ', t, id))
                    break

        # Teleport
        try:
            v = Variable.objects.get(name=Variable.TELEPORT)
            teleport = json.loads(v.value)
            for t in teleport:
                if t.get('preAction') and self.search_global_param_in_task_template(t['preAction'], gp):
                    results.append(self._teleport_entry('Global Parameter used in map teleport preAction: ', id))
                    break

                if t.get('action') and self.search_global_param_in_task_template(t['action'], gp):
                    results.append(self._teleport_entry('Global Parameter used in map teleport action: ', id))
                    break
        except Exception:
            pass

        # Transition trigger
        try:
            v = Variable.objects.get(name=Variable.TRANSITION_TRIGGER)
            transition_trigger = json.loads(v.value)
            for t in transition_trigger:
                if t.get('startAction') and self.search_global_param_in_task_template(t['startAction'], gp):
                    results.append(self._transition_trigger_entry('Global Parameter used in map transition trigger start action: ', id))
                    break

                if t.get('endAction') and self.search_global_param_in_task_template(t['endAction'], gp):
                    results.append(self._transition_trigger_entry('Global Parameter used in map transition trigger end action: ', id))
                    break
        except Exception:
            pass

        return results

    def search_variable(self, id):
        variable = id[3:]

        results = []

        # Variable
        try:
            v = Variable.objects.get(name=Variable.VARIABLE)
            variables = json.loads(v.value)

            for v in variables:
                if v.get('name') == variable:
                    results.append(('Variable: ', variable, 'variable', ''))
                    break
        except Exception:
            pass

        # Task Template
        for t in TaskTemplate.objects.all():
            structure = json.loads(t.structure)
            for a in structure['actions'][1:]:
                if self.search_variable_in_task_template(a, variable):
                    results.append(self._task_template_entry('Variable used in task template: ', t, id))
                    break

        # Agvs
        try:
            def search_variable_in_task_trigger(data, t_type):
                if not data.get('action'):
                    return

                action = data['action']
                skill_desc = self._get_skill_desc(action['skillId'])
                if not skill_desc:
                    return
                for p in skill_desc['params']:
                    v = action['params'].get(p['name'])
                    if v == variable and p['type'].startswith('v'):
                        results.append(self._agv_entry('Variable used for task trigger configuration: ', '%s_TASK_TRIGGER' % t_type))
                        return

            cfg = json.loads(Variable.objects.get(name=Variable.EXECUTOR_CFG).value)
            task_triggers = cfg.get('task_triggers')
            if task_triggers:
                agv_idle = task_triggers.get('agv_idle')
                if agv_idle:
                    search_variable_in_task_trigger(agv_idle, 'AGV_IDLE')
                battery_low = task_triggers.get('battery_low')
                if battery_low:
                    search_variable_in_task_trigger(battery_low, 'BATTERY_LOW')

        except Exception:
            pass

        # Teleport
        try:
            v = Variable.objects.get(name=Variable.TELEPORT)
            teleport = json.loads(v.value)
            for t in teleport:
                if t.get('preAction') and self.search_variable_in_task_template(t['preAction'], variable):
                    results.append(self._teleport_entry('Variable used in map teleport preAction: ', id))
                    break

                if t.get('action') and self.search_variable_in_task_template(t['action'], variable):
                    results.append(self._teleport_entry('Variable used in map teleport action: ', id))
                    break
        except Exception:
            pass

        # Transition trigger
        try:
            v = Variable.objects.get(name=Variable.TRANSITION_TRIGGER)
            transition_trigger = json.loads(v.value)
            for t in transition_trigger:
                if t.get('startAction') and self.search_variable_in_task_template(t['startAction'], variable):
                    results.append(self._transition_trigger_entry('Variable used in map transition trigger start action: ', id))
                    break

                if t.get('endAction') and self.search_variable_in_task_template(t['endAction'], variable):
                    results.append(self._transition_trigger_entry('Variable used in map transition trigger end action: ', id))
                    break
        except Exception:
            pass

        return results

    def search_task_template(self, tt_id):
        results = []
        tt_pk = None
        try:
            tt_pk = int(tt_id[6:])
            tt = TaskTemplate.objects.get(pk=tt_pk)
            results.append(self._task_template_entry('Task Template: ', tt))
        except TaskTemplate.DoesNotExist:
            pass

        if not tt_pk:
            return

        # Task Template
        for t in TaskTemplate.objects.all():
            structure = json.loads(t.structure)
            for d in structure['actions'][1:]:
                if d['skillId'] == tt_id:
                    results.append(self._task_template_entry('Inherited in task template: ', t, tt_id))
                    break

        # Agvs
        try:
            def search_task_template_in_task_trigger(data, t_type):
                if not data.get('action'):
                    return
                action = data['action']

                if action['skillId'] == tt_id:
                    results.append(self._agv_entry('Task template use for task trigger configuration: ', '%s_TASK_TRIGGER' % t_type))
                    return

            cfg = json.loads(Variable.objects.get(name=Variable.EXECUTOR_CFG).value)
            task_triggers = cfg.get('task_triggers')
            if task_triggers:
                agv_idle = task_triggers.get('agv_idle')
                if agv_idle:
                    search_task_template_in_task_trigger(agv_idle, 'AGV_IDLE')
                battery_low = task_triggers.get('battery_low')
                if battery_low:
                    search_task_template_in_task_trigger(battery_low, 'BATTERY_LOW')

                if tt_pk in cfg['custom_init']:
                    results.append(self._agv_entry('Task template used for custom init: ', 'CUSTOM_INIT'))

                if tt_pk in cfg['pre_init']:
                    results.append(self._agv_entry('Task template used for pre init', 'PRE_INIT'))

                if tt_pk == cfg['default_init']['task_template']:
                    results.append(self._agv_entry('Task template used for default init', 'DEFAULT_INIT'))

        except Exception:
            pass

        # Teleport
        try:
            v = Variable.objects.get(name=Variable.TELEPORT)
            teleport = json.loads(v.value)
            for t in teleport:
                if t.get('preAction') and t['preAction'].get('skillId') == tt_id:
                    results.append(self._teleport_entry('Task template used in map teleport preAction: ', tt_id))
                    break

                if t.get('action') and t['action'].get('skillId') == tt_id:
                    results.append(self._teleport_entry('Task template used in map teleport action: ', tt_id))
                    break
        except Exception:
            pass

        # Transition trigger
        try:
            v = Variable.objects.get(name=Variable.TRANSITION_TRIGGER)
            transition_trigger = json.loads(v.value)
            for t in transition_trigger:
                if t.get('startAction') and t['startAction'].get('skillId') == tt_id:
                    results.append(self._transition_trigger_entry('Task template used in map transition trigger start action: ', tt_id))
                    break

                if t.get('endAction') and t['endAction'].get('skillId') == tt_id:
                    results.append(self._transition_trigger_entry('Task template used in map transition trigger end action: ', tt_id))
                    break
        except Exception:
            pass

        return results

    def search_skill(self, id):
        results = []
        skill_id = id[3:]

        # Task Template
        for t in TaskTemplate.objects.all():
            structure = json.loads(t.structure)
            for d in structure['actions'][1:]:
                if d['skillId'] == skill_id:
                    results.append(self._task_template_entry('Action used in task template: ', t, id))
                    break

        # Teleport
        try:
            v = Variable.objects.get(name=Variable.TELEPORT)
            teleport = json.loads(v.value)
            for t in teleport:
                if t.get('preAction') and t['preAction'].get('skillId') == skill_id:
                    results.append(self._teleport_entry('Action used in map teleport preAction: ', id))
                    break

                if t.get('action') and t['action'].get('skillId') == skill_id:
                    results.append(self._teleport_entry('Action used in map teleport action: ', id))
                    break
        except Exception:
            pass

        # Transition trigger
        try:
            v = Variable.objects.get(name=Variable.TRANSITION_TRIGGER)
            transition_trigger = json.loads(v.value)
            for t in transition_trigger:
                if t.get('startAction') and t['startAction'].get('skillId') == skill_id:
                    results.append(self._transition_trigger_entry('Action used in map transition trigger start action: ', id))
                    break

                if t.get('endAction') and t['endAction'].get('skillId') == skill_id:
                    results.append(self._transition_trigger_entry('Action used in map transition trigger end action: ', id))
                    break
        except Exception:
            pass

        return results

    def _task_template_entry(self, desc, t, item=''):
        return (desc, '%s' % t, '_ttpk_%s' % t.id, item)

    def _teleport_entry(self, desc, item=''):
        return (desc, 'Teleport', 'map-teleport', item)

    def _gp_entry(self, desc, item):
        return (desc, 'Task template global param', 'task-template-global-param', item)

    def _variable_entry(self, desc, item=''):
        return (desc, 'Variable', 'variable', item)

    def _map_entry(self, desc, map, item=''):
        return (desc, '%s' % map, '_map_%s' % map.id, item)

    def _agv_entry(self, desc, item=''):
        return (desc, 'Agv', 'agv', item)

    def _transition_trigger_entry(self, desc, item=''):
        return (desc, 'Transition Trigger', 'map-transition-trigger', item)
