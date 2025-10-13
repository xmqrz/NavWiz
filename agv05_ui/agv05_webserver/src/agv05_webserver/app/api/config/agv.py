from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from rest_framework import viewsets, status
from rest_framework.decorators import action
from rest_framework.response import Response
from rest_framework.reverse import reverse
import logging
import requests
import subprocess
import ujson as json
import uuid

from agv05_webserver.system.models import Cache, ExecutorMode, Variable, MapChangeset, TaskTemplate
from agv05_webserver.system.signals import mark_models_cache_as_dirty
from .license_void import LicenseVoidMixin
from ..mixin import Permission

logger = logging.getLogger(__name__)


# TODO: add serializer class.
class AgvConfigViewSet(LicenseVoidMixin, viewsets.ViewSet):
    permission_classes = Permission(
        'system.view_system_panel', 'system.change_agv')

    vars_common = [Variable.AGV_NAME, Variable.EXECUTOR_MODE]
    vars_standalone = [Variable.AGV_HOME, Variable.EXECUTOR_CFG]
    vars_fms = [Variable.FMS_METADATA]
    vars_all = vars_common + vars_standalone + vars_fms

    def list(self, request):
        data = {}
        response = OrderedDict()
        response['agv_uuid'] = Variable.get_agv_uuid()
        data = dict(Variable.objects.filter(
            pk__in=self.vars_all).values_list('name', 'value'))
        for n in [Variable.AGV_NAME, Variable.EXECUTOR_MODE, Variable.AGV_HOME]:
            response[n] = data.get(n)
        response['skillset'] = Cache.get_models_skillset()
        skillset_md5 = Cache.get_models_skillset_md5()
        response['skillset_img'] = '%s?size=128x128&set=set1&bgset=bg1' % reverse(
            'hashicon:index',
            [skillset_md5],
            request=request) if skillset_md5 else ''

        try:
            cfg = json.loads(data[Variable.EXECUTOR_CFG])
            response['custom_init'] = cfg['custom_init']
            response['pre_init'] = cfg['pre_init']
            response['default_init'] = cfg['default_init']['task_template']
            response['default_init_timeout'] = cfg['default_init']['timeout']
            response['default_paused'] = bool(cfg.get('default_paused'))
            response['default_app'] = cfg['default_app']['app']
            response['default_app_timeout'] = cfg['default_app']['timeout']
            response['min_battery_level'] = cfg.get('min_battery_level', 30)
            response['dimension'] = json.dumps(cfg.get('dimension', ''))
            response['task_triggers'] = json.dumps(cfg.get('task_triggers', ''))
            # new field, will raise exception if not exist
            response['allowed_motions'] = cfg['allowed_motions']
        except Exception:
            pass

        try:
            # new field, will raise exception if not exist
            si = cfg['station_init']
            response['station_init'] = si['allowed']
            response['station_init_stations'] = si['stations']
        except Exception:
            pass

        try:
            data['fms_metadata'] = json.loads(data['fms_metadata'])
            response['fms_endpoint'] = data['fms_metadata']['endpoint']
        except Exception:
            pass

        # task trigger form data
        try:
            skillset = json.loads(Cache.get_models_skillset())
            response['register_list'] = json.dumps(skillset['register_list'])
        except Exception:
            pass

        tasktemplate_metas = TaskTemplate.objects.filter(is_top_level=True, is_active=True).values(
            'id', 'name', 'metadata', 'is_active', 'is_top_level').order_by('name')
        response['tasktemplate_metas'] = json.dumps(list(tasktemplate_metas))

        response['station_list'] = '[]'
        try:
            stations = []
            active_map = [int(v) for v in Variable.objects.get(
                pk=Variable.ACTIVE_MAP).value.split(',')]
            for m in active_map:
                mc = MapChangeset.objects.filter(map_id=m).first()
                if mc:
                    stations += json.loads(mc.stations)
            stations.sort()
            response['station_list'] = json.dumps(stations)
        except Exception:
            pass
        else:
            self.station_choices = [(s, s) for s in stations]

        tasktemplate_list = TaskTemplate.objects.filter(is_active=True).values(
            'id', 'name', 'is_active', 'is_top_level').order_by('name')
        tasktemplate_list = list(tasktemplate_list)

        init_choices = []
        pre_init_choices = []

        built_in_init_options = {
            'name': 'Built-in',
            'options': [
                (-1, 'AGV is parked at its home'),
                (-2, 'Forward to charger (tracked)'),
                (-3, 'Reverse to charger (tracked)'),
            ]
        }

        task_template_options = {
            'name': 'Task Template',
            'options': []
        }

        task_template_top_level_options = {
            'name': 'Task Template (Top-level)',
            'options': []
        }

        for d in tasktemplate_list:
            if not d['is_top_level']:
                task_template_options['options'].append(
                    (d['id'], d['name'], not d['is_active']))
            else:
                task_template_top_level_options['options'].append(
                    (d['id'], d['name'], not d['is_active']))

        init_choices.append(built_in_init_options)
        init_choices.append(task_template_options)
        init_choices.append(task_template_top_level_options)

        pre_init_choices.append(task_template_options)
        pre_init_choices.append(task_template_top_level_options)

        response['init_choices'] = json.dumps(init_choices)
        response['pre_init_choices'] = json.dumps(pre_init_choices)

        STATION_INIT_CHOICES = (
            (0, 'Not allowed'),
            (1, 'Allowed at all stations'),
            (2, 'Allowed at specific stations:'),
        )
        response['station_init_choices'] = json.dumps(STATION_INIT_CHOICES)

        response['station_choices'] = json.dumps([{
            'name': 'Junctions',
            'options': [(-1, 'Previous Task Runner Stopped')]
        }, {
            'name': 'Stations',
            'options': getattr(self, 'station_choices', [])
        }])

        return Response(response)

    def create(self, request):
        form_data = request.data

        try:
            is_fms = int(form_data['executor_mode']) == ExecutorMode.DFleet.value
        except Exception as e:
            return Response({'error': 'Invalid executor mode value'}, status=status.HTTP_400_BAD_REQUEST)

        if is_fms:
            response = self._establish_pairing(form_data)
            if response:
                return response
        else:

            error = False
            error_msgs = []
            cfg = {}
            try:
                cfg['allowed_motions'] = form_data['allowed_motions']
            except Exception:
                error_msgs.append('Missing allowed motions data')
                error = True

            try:
                dimension = json.loads(form_data['dimension'])
                cfg['dimension'] = {
                    'body': dimension['body'],
                    'payloads': dimension['payloads'],
                }
            except Exception:
                error_msgs.append('Dimension data corrupted')
                error = True

            try:
                task_triggers = json.loads(form_data['task_triggers'])
                cfg['task_triggers'] = {
                    'agv_idle': task_triggers['agv_idle'],
                    'battery_low': task_triggers['battery_low'],
                }
            except Exception:
                error_msgs.append('Task trigger data corrupted')
                error = True

            ci = form_data.get('custom_init')
            if not ci:
                error_msgs.append('Custom Init field is required')
                error = True
            cfg['custom_init'] = ci

            allowed = form_data.get('station_init')
            stations = form_data.get('station_init_stations')
            if not isinstance(allowed, int):
                error_msgs.append('Station Init field is required')
                error = True
            elif allowed == 2 and not stations:
                error_msgs.append('Station Init Stations field is required')
                error = True
            cfg['station_init'] = {
                'allowed': allowed,
                'stations': stations,
            }

            cfg['pre_init'] = form_data.get('pre_init', [])

            tt = form_data.get('default_init')
            timeout = form_data.get('default_init_timeout')
            if not timeout:
                error_msgs.append('Default initialization timeout must not be empty')
                error = True
            cfg['default_init'] = {
                'task_template': tt if tt else 0,
                'timeout': timeout,
            }

            cfg['default_paused'] = form_data.get('default_paused', False)

            app = form_data.get('default_app')
            timeout = form_data.get('default_app_timeout')
            if not timeout:
                error_msgs.append('Default app timeout must not be empty')
                error = True
            cfg['default_app'] = {
                'app': app,
                'timeout': timeout,
            }

            cfg['min_battery_level'] = form_data.get('min_battery_level')
            if cfg['min_battery_level'] is None:
                error_msgs.append('Minimum battery level must not be empty')
                error = True

            if error:
                return Response(
                    {'error': '%s.' % ', '.join(error_msgs)},
                    status=status.HTTP_400_BAD_REQUEST
                )
            form_data[Variable.EXECUTOR_CFG] = json.dumps(cfg)

        for pk in self.vars_common + (self.vars_fms if is_fms else self.vars_standalone):
            Variable.objects.update_or_create(pk=pk, defaults={
                'value': form_data[pk]
            })

        mark_models_cache_as_dirty()

        if is_fms:
            # TODO: refactor this
            fms_metadata = json.loads(form_data['fms_metadata'])
            return Response({
                'result': True,
                'is_active': fms_metadata['is_active'],
                'url': fms_metadata['agv_listing'],
            })
        return Response({'result': True})

    def _establish_pairing(self, form_data):
        # return response if error.
        # Establish pairing to DFleet server.
        data = dict(Variable.objects.filter(pk__in=self.vars_fms).values_list('name', 'value'))
        data['agv_uuid'] = Variable.get_agv_uuid()

        skillset = Cache.get_models_skillset()
        skillset_md5 = Cache.get_models_skillset_md5()
        if not (skillset and skillset_md5):
            error_msg = ('Error pairing with DFleet: Failed to obtain AGV skillset. ' +
                         'Please ensure that the robot controller has been started.')
            logger.error(error_msg)
            return Response({'error': error_msg}, status=status.HTTP_400_BAD_REQUEST)

        try:
            fms_metadata = json.loads(data['fms_metadata'])
            token = fms_metadata['token']
        except Exception:
            token = ''

        headers = {
            'Authorization': 'AgvToken %s:%s' % (data['agv_uuid'], token)
        }
        agv_data = {
            'name': form_data['agv_name'],
            # TODO: how to resolve for ui url
            'config_panel': '/config',
            'skillset': skillset,
            'skillset_md5': skillset_md5,
        }

        fms_metadata = {}
        try:
            r = requests.post(form_data['fms_endpoint'], headers=headers, json=agv_data,
                              timeout=(3.05, 21), allow_redirects=False, verify=False)
            if r.status_code == 200:
                try:
                    data = json.loads(r.content)['data']
                    fms_metadata['broker'] = data['broker']
                    fms_metadata['endpoint'] = form_data['fms_endpoint']
                    fms_metadata['live_endpoint'] = data['live_endpoint']
                    fms_metadata['task_endpoint'] = data['task_endpoint']
                    fms_metadata['transaction_endpoint'] = data['transaction_endpoint']
                    fms_metadata['dashboard'] = data['dashboard']
                    fms_metadata['agv_listing'] = data['agv_listing']
                    fms_metadata['token'] = data['token']
                    fms_metadata['is_active'] = data['is_active']
                    ntp_server = data['chrony']
                except Exception as ex:
                    raise RuntimeError('Data format error: %s' % ex)
            else:
                try:
                    detail = json.loads(r.content)['detail']
                except Exception:
                    detail = r.reason
                raise RuntimeError('Got (%s) %s' % (r.status_code, detail))

        except requests.exceptions.ConnectionError:
            error_msg = 'Error pairing with DFleet: Cannot connect to server.'
            logger.error(error_msg)
            return Response({'error': error_msg}, status=status.HTTP_400_BAD_REQUEST)

        except Exception as ex:
            error_msg = 'Error pairing with DFleet: %s' % ex
            logger.error(error_msg)
            return Response({'error': error_msg}, status=status.HTTP_400_BAD_REQUEST)

        # Update chrony.conf
        subprocess.call((
            'sudo sed -i \'/server .* iburst prefer trust/d\' /etc/chrony/chrony.conf && '
            'sudo sed -i \'$a server %s iburst prefer trust\' /etc/chrony/chrony.conf && '
            'sudo sed -ri \'s/^initstepslew .*$/initstepslew 1.0 %s/\' /etc/chrony/chrony.conf && '
            '(sudo systemctl enable chrony &)'
        ) % (ntp_server, ntp_server), shell=True)

        form_data[Variable.FMS_METADATA] = json.dumps(fms_metadata)
