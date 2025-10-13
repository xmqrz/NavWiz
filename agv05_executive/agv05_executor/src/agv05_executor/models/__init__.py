from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, ExecutorMode, \
    Variable, db_auto_reconnect
from django.db import transaction
from django.utils.functional import cached_property
import hashlib
import importlib
import networkx as nx
import os
import rospkg
import rospy
import std_msgs.msg
import std_srvs.srv
import threading
import traceback
import ujson as json

from .map_tracker import T_DISTANCE
from .read_write_lock import ReadWriteLock
from .validators.validator import ValidationError, Validator

Flag = Cache.Flag


class Models(object):
    try:
        VERSION = rospkg.RosPack().get_manifest('agv05_executor').version
    except Exception:
        VERSION = '1.11.0'

    tracked_validators = [
        'agv05_executor.models.validators.agv.ExecutorModeValidator',
        'agv05_executor.models.validators.fms_skillset.FmsSkillsetValidator',
        'agv05_executor.models.validators.map_param.MapParamValidator',
        'agv05_executor.models.validators.map.MapValidator',
        'agv05_executor.models.validators.global_param.GlobalParamValidator',
        'agv05_executor.models.validators.variable.VariableValidator',
        'agv05_executor.models.validators.task_template.TaskTemplateValidator',
        'agv05_executor.models.validators.teleport.TeleportValidator',
        'agv05_executor.models.validators.transition_trigger.TransitionTriggerValidator',
        'agv05_executor.models.validators.agv.AgvValidator',
        'agv05_executor.models.validators.agv.FmsValidator',
        'agv05_executor.models.validators.agv.StandaloneValidator',
        'agv05_executor.models.validators.map.ConstrainedMapValidator',
        'agv05_executor.models.validators.transaction.TransactionValidator',
    ]
    trackless_validators = [
        'agv05_executor.models.validators.agv.ExecutorModeValidator',
        'agv05_executor.models.validators.fms_skillset.FmsSkillsetValidator',
        'agv05_executor.models.validators.map_param.MapParamValidator',
        'agv05_executor.models.validators.map_x.MapXValidator',
        'agv05_executor.models.validators.global_param.GlobalParamValidator',
        'agv05_executor.models.validators.variable.VariableValidator',
        'agv05_executor.models.validators.task_template.TaskTemplateValidator',
        'agv05_executor.models.validators.teleport_x.TeleportXValidator',
        'agv05_executor.models.validators.transition_trigger_x.TransitionTriggerXValidator',
        'agv05_executor.models.validators.agv.AgvValidator',
        'agv05_executor.models.validators.agv.FmsValidator',
        'agv05_executor.models.validators.agv_x.StandaloneXValidator',
        'agv05_executor.models.validators.map_x.ConstrainedMapXValidator',
        'agv05_executor.models.validators.transaction.TransactionValidator',
    ]

    def __init__(self, robot):
        self.robot = robot
        self.trackless = robot.trackless
        self.validators = []

        if self.trackless:
            validators = type(self).trackless_validators
        else:
            validators = type(self).tracked_validators
        for v in validators:
            self._add_validator(v)

        self._lock = threading.Lock()

        self._rw_lock = ReadWriteLock()
        self.read_lock = self._rw_lock.read

        self.short = False
        self.valid = False
        self.warning = ''
        cache_valid = (self.skillset_md5 == Cache.get_validated_md5())
        flag = Cache.get_flag()

        if cache_valid and (flag == Flag.Clean.value or flag == Flag.Invalid.value):
            if flag == Flag.Invalid.value:
                self.warning = 'Warning: Using data from cache because the current data is invalid.'
            if self.from_cache():
                self.valid = True
        elif self.validate():
            self.apply()
            self.valid = True
        elif cache_valid:
            self.warning = 'Warning: Using data from cache because the current data is invalid.'
            if self.from_cache():
                self.valid = True

        self.task_counter  # cache task counter
        if self.valid and self.is_standalone_mode():
            rospy.loginfo('Graph nodes: %s', self.graph.nodes(data=True))
            rospy.loginfo('Graph edges: %s', self.graph.edges(data=True))
            rospy.loginfo('Geoms: %s', self.geoms)
            rospy.loginfo('Allowed motions: %s', self.allowed_motions)

        self.__hot_reloaded_pub = rospy.Publisher('~hot_reloaded', std_msgs.msg.Empty, queue_size=1)
        self.__validate_models_service = rospy.Service('~validate_models', std_srvs.srv.Trigger, self._execute)
        self.__hot_reload_service = rospy.Service('~hot_reload', std_srvs.srv.Trigger, self._hot_reload)

        # Publish executor info
        with transaction.atomic():
            Cache.set_models_version(self.VERSION)
            Cache.set_models_skillset(self.skillset)
            Cache.set_models_skillset_md5(self.skillset_md5)
            Cache.set_models_app_descriptions(self.app_descriptions)
            Cache.set_models_rootfs(self.rootfs)

        if Cache.get_flag() == Flag.Dirty.value:
            self._execute({})

    def _add_validator(self, validator_path):
        try:
            module_name, cls_name = validator_path.rsplit('.', 1)
            module = importlib.import_module(module_name)
            cls = getattr(module, cls_name)

            if not issubclass(cls, Validator):
                rospy.logerr('Not a valid validator [%s].', cls)
                return

            self.validators.append(cls(self))
        except Exception as ex:
            rospy.logerr('Cannot load validator [%s]: %s', validator_path, ex)

    @db_auto_reconnect()
    def validate(self, short=False):
        with self._lock:
            rospy.loginfo('Validating models...')
            Cache.set_flag(Flag.Validating.value)
            self.short = short  # short the validators
            self.tmp = {}

            try:
                for v in self.validators:
                    v.validate_and_share_tmp()

                reloadable = self.check_reloadable(self.tmp)
            except ValidationError as ex:
                self._set_error(ex)
                if self.valid:
                    rospy.logwarn('Validation error: %s', ex.error_msg)
                else:
                    rospy.logerr('Validation error: %s', ex.error_msg)
                return False
            except Exception as ex:
                self._set_error(ValidationError('%s' % ex))
                if self.valid:
                    rospy.logwarn('Validator error: %s', ex)
                    rospy.logwarn(traceback.format_exc())
                else:
                    rospy.logerr('Validator error: %s', ex)
                    rospy.logerr(traceback.format_exc())
                return False
            finally:
                self.short = False
                self.tmp = None

            Cache.set_flag_if(
                Flag.Reloadable.value if reloadable else Flag.Valid.value,
                Flag.Validating.value
            )
            rospy.loginfo('Validation OK.')
            rospy.loginfo('Reloadable is %s' % reloadable)
            return True

    @db_auto_reconnect()
    def hot_reload(self, short=False):
        if self.robot.models.is_fms_mode():
            return

        with self._lock:
            if not self.robot.is_alive():
                rospy.logwarn('Hot reload received while not running.')
                return

            rospy.loginfo('Validating models for hot_reload...')
            Cache.set_flag(Flag.Validating.value)
            self.short = short  # short the validators
            self.tmp = {}

            try:
                for v in self.validators:
                    v.validate_and_share_tmp()

                reloadable = self.check_reloadable(self.tmp)
            except ValidationError as ex:
                self._set_error(ex)
                if self.valid:
                    rospy.logwarn('Validation error: %s', ex.error_msg)
                else:
                    rospy.logerr('Validation error: %s', ex.error_msg)
                return False
            except Exception as ex:
                self._set_error(ValidationError('%s' % ex))
                if self.valid:
                    rospy.logwarn('Validator error: %s', ex)
                    rospy.logwarn(traceback.format_exc())
                else:
                    rospy.logerr('Validator error: %s', ex)
                    rospy.logerr(traceback.format_exc())
                return False
            finally:
                self.short = False
                self.tmp = None

            Cache.set_flag_if(
                Flag.Reloadable.value if reloadable else Flag.Valid.value,
                Flag.Validating.value
            )
            rospy.loginfo('Validation OK.')
            rospy.loginfo('Reloadable is %s' % reloadable)

            if reloadable:
                rospy.loginfo('Hot reload triggered')
                with self._rw_lock.write:
                    for v in self.validators:
                        v.hot_reload()

                    try:
                        del self.downloadables_md5
                    except Exception:
                        pass

                    Cache.set_flag_if(Flag.Clean.value, (Flag.Valid.value, Flag.Reloadable.value))
                self.__hot_reloaded_pub.publish(std_msgs.msg.Empty())

            return reloadable

    def _set_error(self, err):
        Cache.set_validation_data({
            'error_msg': str(err),
            'params': err.params,
        })
        Cache.set_flag_if(Flag.Invalid.value, Flag.Validating.value)

    def apply(self):
        for v in self.validators:
            v.apply()

        Variable.objects.filter(name='/agv05_executor/previous_location').delete()

        Cache.set_validated_md5(self.skillset_md5)

        # set flag only if it has not been changed.
        Cache.set_flag_if(Flag.Clean.value, (Flag.Valid.value, Flag.Reloadable.value))
        return True

    def from_cache(self):
        try:
            for v in self.validators:
                v.from_cache()
        except Exception as ex:
            self._set_error(ValidationError('Using data from cache but cache is corrupted: %s' % ex))
            rospy.logerr('Cache reconstruction error: %s', ex)
            rospy.logerr(traceback.format_exc())
            return False

        return True

    def from_downloadables(self):
        assert (self.is_fms_mode())
        with self._lock, self._rw_lock.write:
            try:
                # self-validate against skillset_md5 of downloadables
                if self.downloadables_md5['skillset'] != self.skillset_md5:
                    raise RuntimeError('AGV skillset is incompatible.')
                for v in self.validators:
                    v.from_downloadables(self.downloadables)
            except Exception as ex:
                self.downloadables['error'] = '%s' % ex
                rospy.logerr('Downloadables reconstruction error: %s', ex)
                rospy.logerr(traceback.format_exc())
            else:
                return True

    def hot_reload_downloadables(self):
        assert (self.is_fms_mode())
        with self._lock, self._rw_lock.write:
            try:
                # self-validate against skillset_md5 of downloadables
                if self.downloadables_md5['skillset'] != self.skillset_md5:
                    raise RuntimeError('AGV skillset is incompatible.')
                rospy.loginfo('Hot reload downloadables triggered')
                for v in self.validators:
                    v.hot_reload(self.downloadables)
            except Exception as ex:
                self.downloadables['error'] = '%s' % ex
                rospy.logerr('Downloadables reconstruction error while hot reload: %s', ex)
                rospy.logerr(traceback.format_exc())
            else:
                return True

    def check_reloadable(self, data, enabled=True, pre_validate=True):
        reloadable = False
        try:
            for v in self.validators:
                if v.is_enabled(pre_validate=pre_validate) != enabled:
                    continue

                if not v.check_reloadable(data):
                    reloadable = False
                    break
            else:
                reloadable = True
        except Exception as e:
            rospy.logerr('Check reloadable error: %s', e)

        return reloadable

    @cached_property
    def skillset(self):
        return json.dumps({
            'version': self.VERSION,
            'trackless': bool(self.trackless),
            'dynamic_path_planning': bool(self.robot.dynamic_path_planning),
            'skill_descriptions': self.skill_descriptions,
            'register_list': self.register_list,
            'summary': (
                'Version: %s ' % self.VERSION +
                '(%s)\n' % ('trackless' if self.trackless else 'tracked') +
                'Features: -\n' +
                'Maps (with Map Parameters, Transitions and Teleports), ' +
                'Task Templates (with Global Parameters, Variables and DFleet skills)\n' +
                'Actions: -\n' +
                '%s\n' % ', '.join(self.robot.skill_manager.get_summary()) +
                'Registers: -\n' +
                '%s' % ',  '.join(self.robot.registry.get_summary())),
        }, sort_keys=True)

    @cached_property
    def skillset_md5(self):
        return hashlib.md5(self.skillset.encode('utf-8')).hexdigest()

    @cached_property
    def skill_descriptions(self):
        return self.robot.skill_manager.get_skill_descriptions()

    @cached_property
    def app_descriptions(self):
        return self.robot.app_manager.get_app_descriptions()

    @cached_property
    def rootfs(self):
        return os.environ.get('ROOTFS')

    @cached_property
    def register_list(self):
        return self.robot.registry.get_register_list()

    @cached_property
    def downloadables(self):
        d = Cache.get_downloadables()
        return d if d and isinstance(d, dict) else {}

    @cached_property
    def downloadables_md5(self):
        d = Cache.get_downloadables_md5()
        return d if d and isinstance(d, dict) else {}

    def is_valid(self):
        return self.valid

    def get_validation_msg(self):
        return Cache.get_validation_msg()

    def get_warning_msg(self):
        return self.warning

    def _execute(self, req):
        threading.Thread(target=self.validate, args=(True,)).start()
        return True, ''

    def _hot_reload(self, req):
        threading.Thread(target=self.hot_reload).start()
        return True, ''

    def find_shortest_path(self, j1, j2):
        try:
            plan = nx.shortest_path(self.graph, j1, j2, 'distance')
            return plan
        except nx.NetworkXNoPath:
            return []

    def find_shortest_constrained_path(self, start, end, reverse=False):
        NO_PATH = [], []

        # raises AssertionError if invalid start heading
        self.check_transition(start[1], 0, zero=True)

        # cannot rotate at home
        home = self.get_agv_home_location()
        if start[0] == home[0]:
            if not self.check_transition(start[1], home[1], zero=True):
                return NO_PATH
        if end[0] == home[0]:
            if not self.check_transition(home[1], end[1], zero=True):
                return NO_PATH

        # same spot (check rotation)
        if start[0] == end[0]:
            t = self.check_transition(start[1], end[1], j=start[0])
            if t:
                return [start[0]], [t]

        # must be able to perform straight movement
        restricted = [True, 'forward' not in self.allowed_motions, 'reverse' not in self.allowed_motions]
        if all(restricted):
            return NO_PATH
        restricted[0] = restricted[2 if reverse else 1]

        # find a manoeuvre
        q = self.geoms[self.get_dimension_profile()]
        lgraph = self.rlgraphs[q] if reverse else self.flgraphs[q]
        lgraph.add_node('start')
        lgraph.add_node((end[0], 'end'))
        added_nodes = ['start', (end[0], 'end')]

        for e in self.graph.out_edges_iter(start[0], data=True):
            if not self.is_teleport(e[2]) and restricted[e[2]['facing']]:
                continue
            t = self.check_transition(start[1], e, reverse=reverse)
            if not t:
                continue
            d = T_DISTANCE[t]
            lgraph.add_edge('start', (e[0], e[1]), {'distance': d, 'transition': t})

        for e in self.graph.in_edges_iter(end[0], data=True):
            if not self.is_teleport(e[2]) and restricted[e[2]['facing']]:
                continue
            t = self.check_transition(e, end[1], reverse=reverse)
            if not t:
                continue
            d = e[2]['distance'] + T_DISTANCE[t]
            lgraph.add_edge((e[0], e[1]), (end[0], 'end'), {'distance': d, 'transition': t})

        try:
            lplan = nx.shortest_path(lgraph, 'start', (end[0], 'end'), 'distance')
        except nx.NetworkXNoPath:
            return NO_PATH
        else:
            plan = []
            transitions = []
            iter_lplan = iter(lplan)
            e = next(iter_lplan)
            for f in iter_lplan:
                plan.append(f[0])
                transitions.append(lgraph[e][f]['transition'])
                e = f
            return plan, transitions
        finally:
            lgraph.remove_nodes_from(added_nodes)

    def validate_constrained_path(self, start, end, plan, reverse=False):
        # raises AssertionError if invalid start heading
        self.check_transition(start[1], 0, zero=True)

        # match start and end points
        if not plan or start[0] != plan[0] or end[0] != plan[-1]:
            return False

        # cannot rotate at home
        home = self.get_agv_home_location()
        if start[0] == home[0]:
            if not self.check_transition(start[1], home[1], zero=True):
                return False
        if end[0] == home[0]:
            if not self.check_transition(home[1], end[1], zero=True):
                return False

        # same spot (check rotation)
        if len(plan) == 1:
            return bool(self.check_transition(start[1], end[1], j=start[0]))

        # must be able to perform straight movement
        restricted = [True, 'forward' not in self.allowed_motions, 'reverse' not in self.allowed_motions]
        if all(restricted):
            return False
        restricted[0] = restricted[2 if reverse else 1]

        # validate the manoeuvre
        q = self.geoms[self.get_dimension_profile()]
        lgraph = self.rlgraphs[q] if reverse else self.flgraphs[q]
        return self._validate_constrained_path2(start, end, plan, reverse, restricted, lgraph)

    def _validate_constrained_path2(self, start, end, plan, reverse, restricted, lgraph):
        self._validated_transitions = []
        iter_plan = iter(plan)
        prev_j = next(iter_plan)
        cur_j = next(iter_plan)

        try:
            e = (prev_j, cur_j, self.get_path(prev_j, cur_j))
        except Exception:
            return False
        t = self.check_transition(start[1], e, reverse=reverse)
        if not t:
            return False
        self._validated_transitions.append(t)

        if len(plan) == 2:
            if not self.is_teleport(e[2]) and restricted[e[2]['facing']]:
                return False
            t = self.check_transition(e, end[1], reverse=reverse)
            self._validated_transitions.append(t)
            return bool(t)

        e = e[:2]
        for next_j in iter_plan:
            f = (cur_j, next_j)
            try:
                t = lgraph[e][f]['transition']
            except Exception:
                return False
            self._validated_transitions.append(t)
            prev_j, cur_j = cur_j, next_j
            e = f

        try:
            e = (prev_j, cur_j, self.get_path(prev_j, cur_j))
        except Exception:
            return False
        t = self.check_transition(e, end[1], reverse=reverse)
        self._validated_transitions.append(t)
        return bool(t)

    def get_agv_home_location(self):
        return self.get_location_from_station(self.agv_home)

    def get_location_from_station(self, station):
        if station in self.stations_assoc:
            return self.stations_assoc[station]

    def get_location_from_rfid(self, rfid):
        assert not self.trackless
        heading = None
        s = rfid.rsplit(':', 1)
        if len(s) > 1:
            if s[1] in self.rfid_suffix_assoc:
                heading = self.rfid_suffix_assoc.get(s[1])
                rfid = s[0]

        if rfid in self.rfid_assoc:
            location = self.rfid_assoc[rfid]
            if heading is not None:
                location = (location[0], heading)
            return location

    def get_junction(self, j):
        n = self.graph.node[j]
        return {
            'x': n['x'],
            'y': n['y']
        }

    def get_path(self, j1, j2):
        return self.graph[j1][j2]

    def incr_task_counter(self):
        with transaction.atomic():
            v = Variable.objects.select_for_update().filter(pk=Variable.TASK_COUNTER).first()
            if v:
                v.value = int(v.value) + 1
                v.save()
                self.task_counter = int(v.value)
                return self.task_counter

        Variable.objects.get_or_create(pk=Variable.TASK_COUNTER, defaults={'value': 0})
        return self.incr_task_counter()

    @cached_property
    def task_counter(self):
        v = Variable.objects.get_or_create(pk=Variable.TASK_COUNTER, defaults={'value': 0})[0]
        return int(v.value)

    def is_standalone_mode(self):
        return self.executor_mode == ExecutorMode.Standalone.value

    def is_fms_mode(self):
        return self.executor_mode == ExecutorMode.DFleet.value
