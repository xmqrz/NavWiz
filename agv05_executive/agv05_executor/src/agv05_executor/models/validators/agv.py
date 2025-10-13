from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, Direction, \
    ExecutorMode, PathFacing, Variable
from django.utils.encoding import force_text
from six.moves import zip
import math
import shapely.geometry
import six
import ujson as json

from .validator import ValidationError, Validator

AGV_PARAM = {
    'agv': {
        'name': 'AGV',
        'id': 'agv',
        'item': '',
    }
}


class AgvValidator(Validator):
    cache = Cache
    provides = ['agv_uuid', 'agv_name']

    def validate(self):
        self.agv_uuid = Variable.get_agv_uuid()
        if not self.agv_uuid:
            raise ValidationError('{agv} does not have a UUID. Have you activated the NavWiz license?', params=AGV_PARAM)

        try:
            self.agv_name = Variable.objects.get(pk=Variable.AGV_NAME).value
            if not self.agv_name:
                raise RuntimeError()
        except Exception:
            raise ValidationError('{agv} name is empty.', params=AGV_PARAM)


class ExecutorModeValidator(Validator):
    cache = Cache
    provides = ['executor_mode']

    def validate(self):
        try:
            self.executor_mode = int(Variable.objects.get(pk=Variable.EXECUTOR_MODE).value)
        except Exception:
            raise ValidationError('{agv}\'s executor mode is not configured.', params=AGV_PARAM)

        if self.executor_mode not in next(zip(*ExecutorMode.choices())):
            raise ValidationError('{agv} executor mode is invalid.', params=AGV_PARAM)


class FmsValidator(Validator):
    cache = Cache
    provides = ['fms_metadata']
    extra_provides = ['allowed_motions']

    def is_enabled(self, pre_validate=False):
        executor_mode = self.models.tmp['executor_mode'] if pre_validate else self.models.executor_mode
        return executor_mode == ExecutorMode.DFleet.value

    def validate(self):
        import kombu
        try:
            self.fms_metadata = json.loads(Variable.objects.get(pk=Variable.FMS_METADATA).value)
            if not (self.fms_metadata['broker'] and
                    kombu.Connection(self.fms_metadata['broker']) and
                    self.fms_metadata['endpoint'] and
                    self.fms_metadata['live_endpoint'] and
                    self.fms_metadata['task_endpoint'] and
                    self.fms_metadata['transaction_endpoint'] and
                    self.fms_metadata['dashboard'] and
                    self.fms_metadata['agv_listing'] and
                    self.fms_metadata['token'] and
                    (self.fms_metadata['is_active'] or True)):
                raise RuntimeError('Some data are missing.')
        except Exception as ex:
            raise ValidationError('{agv}\'s DFleet server is not configured or paired properly: %s' % ex, params=AGV_PARAM)

        self.assemble()

    def assemble(self):
        # pre-initialize `allowed_motions` variable while not yet synced from DFleet
        self.allowed_motions = {'forward', 'reverse', 'rotate_left', 'rotate_right', 'uturn_left', 'uturn_right'}


class StandaloneValidator(Validator):
    cache = Cache
    provides = ['agv_home', 'executor_cfg']
    extra_provides = ['graph', 'geoms', 'dimension', 'rr', 'allowed_motions', 'task_triggers',
        'custom_init', 'station_init', 'pre_init', 'default_init',
        'default_paused', 'default_app', 'min_battery_level']

    def is_enabled(self, pre_validate=False):
        executor_mode = self.models.tmp['executor_mode'] if pre_validate else self.models.executor_mode
        return executor_mode == ExecutorMode.Standalone.value

    def validate(self):
        self.graph = self.models.tmp['graph']
        self.stations_assoc = self.models.tmp['stations_assoc']
        self._validate_home()
        self._validate_others()
        self._assemble()

    def _validate_home(self):
        # Validate agv_home
        try:
            self.agv_home = Variable.objects.get(pk=Variable.AGV_HOME).value
        except Exception:
            raise ValidationError('{agv} home is not specified.', params=AGV_PARAM)

        if self.agv_home not in self.models.tmp['station_names']:
            raise ValidationError('{agv} home is invalid.', params=AGV_PARAM)

        s = self.stations_assoc[self.agv_home]
        if s[1] == Direction.NA:
            raise ValidationError('{agv} home station cannot be directionless.', params=AGV_PARAM)

        for name, location in self.stations_assoc.items():
            if name == self.agv_home:
                continue
            if location[0] == s[0]:
                raise ValidationError('{agv} home station cannot overlap with another station.', params=AGV_PARAM)

        self._amend_graph()

    def _amend_graph(self):
        s = self.stations_assoc[self.agv_home]

        for e in self.graph.out_edges_iter(s[0], data=True):
            d = e[2]['direction']
            if d == Direction.NA:
                continue  # path of teleport type

            if s[1] == d:
                if e[2]['facing'] == PathFacing.REVERSE_UNI:
                    raise ValidationError('Path\'s reverse motion from {agv} home station does not align with the direction of the station itself.', params=AGV_PARAM)
                e[2]['facing'] = PathFacing.FORWARD_UNI

            elif s[1] == (d + 2) % 4:
                if e[2]['facing'] == PathFacing.FORWARD_UNI:
                    raise ValidationError('Path\'s forward motion from {agv} home station does not align with the direction of the station itself.', params=AGV_PARAM)
                e[2]['facing'] = PathFacing.REVERSE_UNI

            else:
                raise ValidationError('Path from {agv} home station must align with the direction of the station itself.', params=AGV_PARAM)

        for e in self.graph.in_edges_iter(s[0], data=True):
            d = e[2]['direction']
            if d == Direction.NA:
                continue  # path of teleport type
            d = (d + e[2]['transform']) % 4

            if s[1] == d:
                if e[2]['facing'] == PathFacing.REVERSE_UNI:
                    raise ValidationError('Path\'s reverse motion to {agv} home station does not align with the direction of the station itself.', params=AGV_PARAM)
                e[2]['facing'] = PathFacing.FORWARD_UNI

            elif s[1] == (d + 2) % 4:
                if e[2]['facing'] == PathFacing.FORWARD_UNI:
                    raise ValidationError('Path\'s forward motion to {agv} home station does not align with the direction of the station itself.', params=AGV_PARAM)
                e[2]['facing'] = PathFacing.REVERSE_UNI

            else:
                raise ValidationError('Path to {agv} home station must align with the direction of the station itself.', params=AGV_PARAM)

    def _validate_others(self):
        # Validate executor_cfg
        self._params_dict = {}
        try:
            cfg = json.loads(Variable.objects.get(pk=Variable.EXECUTOR_CFG).value)
            cfg = {
                'dimension': {
                    'body': cfg['dimension']['body'],
                    'payloads': cfg['dimension']['payloads'],
                },
                'allowed_motions': set(cfg['allowed_motions']),
                'task_triggers': {
                    'agv_idle': cfg['task_triggers']['agv_idle'],
                    'battery_low': cfg['task_triggers']['battery_low'],
                },
                'custom_init': [int(v) for v in cfg['custom_init']],
                'station_init': {
                    'allowed': int(cfg['station_init']['allowed']),
                    'stations': cfg['station_init']['stations'],
                },
                'pre_init': [int(v) for v in cfg['pre_init']],
                'default_init': {
                    'task_template': int(cfg['default_init']['task_template']),
                    'timeout': int(cfg['default_init']['timeout']),
                },
                'default_paused': bool(cfg['default_paused']),
                'default_app': {
                    'app': cfg['default_app']['app'],
                    'timeout': int(cfg['default_app']['timeout']),
                },
                'min_battery_level': int(cfg['min_battery_level']),
            }
            assert (0 <= cfg['station_init']['allowed'] <= 2)
            assert (isinstance(cfg['station_init']['stations'], list))
        except Exception:
            raise ValidationError('{agv} settings are not configured or outdated.', params=AGV_PARAM)

        # Dimension
        try:
            body = cfg['dimension']['body']
            length = float(body['length'])
            del body['length']
            width = float(body['width'])
            del body['width']
            vcenter = float(body['vcenter'])
            del body['vcenter']
            hcenter = float(body['hcenter'])
            del body['hcenter']
            margin = [float(sm) for sm in body['safetyMargin']]
            del body['safetyMargin']
            assert (0 < length <= 3)
            assert (0 < width <= 3)
            assert (0 < vcenter < length)
            assert (0 < hcenter < width)
            assert (body['svgPath'])
            assert (len(margin) == 4)
            for m in margin:
                assert (0 <= m <= 3)

            min_xn = -vcenter
            max_xn = length - vcenter
            min_yn = hcenter - width
            max_yn = hcenter
            bq = (min_xn, max_xn, min_yn, max_yn)
            min_x = min_xn - margin[1]  # bottom
            max_x = max_xn + margin[0]  # top
            min_y = min_yn - margin[3]  # right
            max_y = max_yn + margin[2]  # left
            if not self.robot.trackless:
                min_yn, max_yn = -max_yn, -min_yn
                min_y, max_y = -max_y, -min_y
            body['qn'] = (min_xn, min_yn, max_xn, max_yn)
            body['q'] = (min_x, min_y, max_x, max_y)

            payloads = cfg['dimension']['payloads']
            assert (isinstance(payloads, list) and len(payloads) == 5)
            for payload in payloads:
                length = float(payload['length'])
                del payload['length']
                width = float(payload['width'])
                del payload['width']
                vcenter = float(payload['vcenter'])
                del payload['vcenter']
                hcenter = float(payload['hcenter'])
                del payload['hcenter']
                margin = [float(sm) for sm in payload['safetyMargin']]
                del payload['safetyMargin']
                assert (0 < length <= 3)
                assert (0 < width <= 3)
                assert (-2 <= vcenter <= 2)
                assert (-2 <= hcenter <= 2)
                assert (payload['svgPath'])
                assert (len(margin) == 4)
                for m in margin:
                    assert (0 <= m <= 3)

                length /= 2.0
                width /= 2.0
                min_xn = -vcenter - length
                max_xn = -vcenter + length
                min_yn = -hcenter - width
                max_yn = -hcenter + width
                min_xq = bq[0] - margin[1]  # bottom
                max_xq = bq[1] + margin[0]  # top
                min_yq = bq[2] - margin[3]  # right
                max_yq = bq[3] + margin[2]  # left
                min_x = min_xn - margin[1]  # bottom
                max_x = max_xn + margin[0]  # top
                min_y = min_yn - margin[3]  # right
                max_y = max_yn + margin[2]  # left
                if not self.robot.trackless:
                    min_yn, max_yn = -max_yn, -min_yn
                    min_yq, max_yq = -max_yq, -min_yq
                    min_y, max_y = -max_y, -min_y
                payload['qn'] = (min_xn, min_yn, max_xn, max_yn)
                payload['bq'] = (min_xq, min_yq, max_xq, max_yq)
                payload['q'] = (min_x, min_y, max_x, max_y)
        except Exception:
            raise ValidationError('{agv} dimension profile has invalid data.', params=AGV_PARAM)

        # Task Triggers
        try:
            agv_idle = cfg['task_triggers']['agv_idle']
            cooked = int(agv_idle['timeout'])
            agv_idle['timeout'] = cooked
            if cooked < 3 or cooked > 9999:
                raise ValidationError('{agv} task trigger agv_idle: timeout value is out-of-range.', params=AGV_PARAM)
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('{agv} task trigger agv_idle has invalid data.', params=AGV_PARAM)

        try:
            battery_low = cfg['task_triggers']['battery_low']
            cooked = int(battery_low['threshold'])
            battery_low['threshold'] = cooked
            if cooked < 0 or cooked > 100:
                raise ValidationError('{agv} task trigger battery_low: threshold value is out-of-range.', params=AGV_PARAM)
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('{agv} task trigger battery_low has invalid data.', params=AGV_PARAM)

        for key, value in cfg['task_triggers'].items():
            try:
                cooked = bool(value['abortable'])
                value['abortable'] = cooked
                cooked = bool(value['ignore_charging'])
                value['ignore_charging'] = cooked
                cooked = bool(value['ignore_home'])
                value['ignore_home'] = cooked
                cooked = bool(value['ignore_low_battery'])
                value['ignore_low_battery'] = cooked
                cooked = bool(value['ignore_trigger_active'])
                value['ignore_trigger_active'] = cooked

                for raw in value['ignore_stations']:
                    if raw not in self.models.tmp['station_names']:
                        raise ValidationError('stations refers to invalid station "%s".' % raw)

                if value['action']['skillId']:
                    self._validate_action(value['action'])
                    value['action']['template_id'] = int(value['action']['skillId'][6:])
                else:
                    cfg['task_triggers'][key] = None

            except ValidationError as ex:
                params = AGV_PARAM.copy()
                params.update(ex.params)
                raise ValidationError('{agv} task trigger %s: %s' % (key, ex), params=params)
            except Exception as ex:
                raise ValidationError('{agv} task trigger %s has invalid data.' % key, params=AGV_PARAM)

        # Custom Init
        if not cfg['custom_init']:
            raise ValidationError('{agv} initialization task template must not be empty.', params=AGV_PARAM)
        for ci in cfg['custom_init']:
            if ci in [-1, -2, -3]:
                continue
            try:
                tt = self.models.tmp['task_templates_assoc'][ci]
            except Exception:
                raise ValidationError('{agv} initialization task template is invalid.', params=AGV_PARAM)

            if 'plan' in tt['mutexes']:
                params = {
                    'tt': {
                        'name': tt['name'],
                        'id': '_ttpk_%s' % tt['id'],
                        'item': '',
                    }
                }
                params.update(AGV_PARAM)
                raise ValidationError('{agv} initialization task template "{tt}" must not contain actions that require path planning.', params=params)

        # Station Init
        allowed = cfg['station_init']['allowed']
        if allowed in [0, 1]:
            cfg['station_init'] = allowed
        else:
            allowed_junction_init = [-1]
            junction_init = [s for s in cfg['station_init']['stations'] if s in allowed_junction_init]
            cfg['station_init'] = [s for s in cfg['station_init']['stations'] if s not in allowed_junction_init]
            for raw in cfg['station_init']:
                if raw not in self.models.tmp['station_names']:
                    raise ValidationError('{agv} initialization stations refer to invalid station "%s".' % raw, params=AGV_PARAM)
            cfg['station_init'] += junction_init

        # Pre Init
        for pi in cfg['pre_init']:
            try:
                tt = self.models.tmp['task_templates_assoc'][pi]
            except Exception:
                raise ValidationError('{agv} pre initialization task template is invalid.', params=AGV_PARAM)

            if 'plan' in tt['mutexes']:
                params = {
                    'tt': {
                        'name': tt['name'],
                        'id': '_ttpk_%s' % tt['id'],
                        'item': '',
                    }
                }
                params.update(AGV_PARAM)
                raise ValidationError('{agv} pre initialization task template "{tt}" must not contain actions that require path planning.', params=params)

        # Default Init
        tt = cfg['default_init']['task_template']
        if not tt:
            cfg['default_init'] = None
        elif tt not in cfg['custom_init']:
            raise ValidationError('{agv} default initialization task template is not one of the initialization task templates.', params=AGV_PARAM)
        elif tt in [-1, -2, -3]:
            pass
        elif self.models.tmp['task_templates_assoc'][tt]['params']:
            raise ValidationError('{agv} default initialization task template must not require parameters.', params=AGV_PARAM)

        # Default App
        if cfg['default_app']['app'] not in ['task-runner']:
            cfg['default_app'] = None

        # Min Battery Level
        if cfg['min_battery_level'] < 0:
            cfg['min_battery_level'] = 0
        elif cfg['min_battery_level'] > 100:
            cfg['min_battery_level'] = 100

        self.executor_cfg = cfg

    def _validate_action(self, action):
        desc_params = None
        desc_outcomes = None

        if not action['skillId'].startswith('_ttpk_'):
            raise ValidationError('invalid task template action.')

        try:
            template_id = int(action['skillId'][6:])
            tt_inner = self.models.tmp['task_templates_assoc'][template_id]
            if not tt_inner:
                raise RuntimeError()
        except Exception:
            raise ValidationError('task template is either inactive or has been removed.')

        if not tt_inner['is_top_level']:
            tt_param = {
                'tt': {
                    'name': tt_inner['name'],
                    'id': '_ttpk_%s' % tt_inner['id'],
                    'item': '',
                }
            }
            raise ValidationError('task template "{tt}" is not top level.', params=tt_param)

        desc_params = tt_inner['params']
        desc_outcomes = tt_inner['outcomes']

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
        self._amend_graph()
        self._assemble()

    def _assemble(self):
        for k in self.extra_provides[2:]:
            setattr(self, k, self.executor_cfg.get(k))

        self._construct_geoms()

        # from_downloadables
        self.allowed_motions = set(self.allowed_motions)

        # remove `dynamic` if the feature is disabled
        if not self.robot.dynamic_path_planning:
            self.allowed_motions.discard('dynamic')

    def _construct_geoms(self):
        e = 1e-9
        self.geoms = geoms = {}
        self.rr = []

        # from_downloadables: convert list to tuple
        q = tuple(self.dimension['body']['q'])
        qe = [q[0] - e, q[1] - e, q[2] + e, q[3] + e]
        geoms[0] = q
        geoms[q] = g = shapely.geometry.box(*qe)
        g.r = math.hypot(max(abs(qe[0]), abs(qe[2])), max(abs(qe[1]), abs(qe[3])))
        g.a = True

        qn = self.dimension['body']['qn']
        qne = [qn[0] - e, qn[1] - e, qn[2] + e, qn[3] + e]
        rn = math.hypot(max(abs(qne[0]), abs(qne[2])), max(abs(qne[1]), abs(qne[3])))
        self.rr.append((rn, g.r))

        for i, payload in enumerate(self.dimension['payloads'], start=1):
            # from_downloadables: convert list to tuple
            q = tuple(payload['bq'])
            p = tuple(payload['q'])
            qp = (q, p)
            geoms[i] = qp

            if q not in geoms:
                qe = [q[0] - e, q[1] - e, q[2] + e, q[3] + e]
                geoms[q] = g = shapely.geometry.box(*qe)
                g.r = math.hypot(max(abs(qe[0]), abs(qe[2])), max(abs(qe[1]), abs(qe[3])))

            if p not in geoms:
                pe = [p[0] - e, p[1] - e, p[2] + e, p[3] + e]
                geoms[p] = g = shapely.geometry.box(*pe)
                g.r = math.hypot(max(abs(pe[0]), abs(pe[2])), max(abs(pe[1]), abs(pe[3])))

            if qp not in geoms:
                geoms[qp] = geoms[q].union(geoms[p])

            pn = payload['qn']
            pne = [pn[0] - e, pn[1] - e, pn[2] + e, pn[3] + e]
            rpn = math.hypot(max(abs(pne[0]), abs(pne[2])), max(abs(pne[1]), abs(pne[3])))
            self.rr.append((max(rn, rpn), max(geoms[q].r, geoms[p].r)))

            if self.models.short:  # short the validator
                return

    def provide_reloadable(self, provide, cur, tmp):
        if provide == 'executor_cfg':
            cur = cur.copy()
            cur['allowed_motions'] = sorted(cur['allowed_motions'])
            tmp = tmp.copy()
            tmp['allowed_motions'] = sorted(tmp['allowed_motions'])
            return super(StandaloneValidator, self).provide_reloadable(provide, cur, tmp)

        return super(StandaloneValidator, self).provide_reloadable(provide, cur, tmp)
