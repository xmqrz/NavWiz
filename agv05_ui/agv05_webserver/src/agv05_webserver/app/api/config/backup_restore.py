from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.conf import settings as django_settings
from django.contrib.auth import get_user_model
from django.contrib.auth.models import Group
from django.core.files.base import ContentFile
from django.core.files.storage import default_storage
from django.db import IntegrityError
from django.http import HttpResponse
from django.utils import timezone
from io import BytesIO
from rest_framework import serializers, viewsets
from rest_framework.decorators import action
from rest_framework.response import Response
from rest_framework.reverse import reverse
import base64
import gzip
import logging
import os.path
import re
import ujson as json
import uuid
import yaml
try:
    from yaml import full_load
except Exception:
    from yaml import load as full_load

from ..mixin import Permission
from .license_void import LicenseVoidMixin
from .parameter import ParameterMixin
from .software_update import get_current_version
from agv05_webserver.system.models import Cache, Map, MapAnnotation, MapChangeset, Parameter, Variable, TaskTemplate
from agv05_webserver.system.signals import mark_models_cache_as_dirty, suppress_signals

logger = logging.getLogger(__name__)

User = get_user_model()
backup_file_magic = 'DF_AGV05'
variable_backup_list = [
    Variable.AGV_NAME, Variable.AGV_HOME, Variable.EXECUTOR_MODE,
    Variable.EXECUTOR_CFG, Variable.FMS_METADATA,
    Variable.IO_NAME,
]

tracked_backup_file_magic = 'DF_AGV05'
trackless_backup_file_magic = 'DF_AGV05X'
tracked_fms_backup_file_magic = 'DF_AGVCCS'
trackless_fms_backup_file_magic = 'DF_AGVCCSX'

NAME_MAX_LENGTH = 127


class BackupRestoreMixin(LicenseVoidMixin):
    permission_classes = Permission('system.view_system_panel', 'system.backup_restore')

    magic = backup_file_magic
    variable_backup_list = variable_backup_list

    def validate_selected_tree(self, options, selected):
        try:
            if not selected:
                raise RuntimeError('No data selected.')
            available_gid = []
            selected_gid = []
            required_gid = []
            self._validate_and_get_dependency(options, selected, available_gid, selected_gid, required_gid)
            for gid in required_gid:
                if gid not in selected_gid and gid in available_gid:
                    raise RuntimeError('Dependency error.')
        except RuntimeError as ex:
            return 'Backup Error: %s' % ex
        return ''

    def _validate_and_get_dependency(self, options, selected, available_gid, selected_gid, required_gid):
        if isinstance(options, list):
            if not isinstance(selected, list):
                raise RuntimeError('Selected data is corrupted.')
            if len(selected) <= 0:
                raise RuntimeError('Selected component cannot be empty.')
            selected_id = []
            for data in selected:
                if 'id' not in data:
                    raise RuntimeError('Selected data is corrupted.')
                if data['id'] in selected_id:
                    raise RuntimeError('Duplicate selected data detected.')

                selected_id.append(data['id'])

            for data in options:
                if 'gid' in data and data['gid'] not in available_gid:
                    available_gid.append(data['gid'])

                if data['id'] not in selected_id:
                    continue
                if 'gid' in data and data['gid'] not in selected_gid:
                    selected_gid.append(data['gid'])
                if 'dep' in data and isinstance(data['dep'], list):
                    required_gid.extend([gid for gid in data['dep'] if gid not in required_gid])

                selected_id.remove(data['id'])

            if len(selected_id) > 0:
                raise RuntimeError('Some selected data does not exist.')

        elif isinstance(options, dict):
            if not isinstance(selected, dict):
                raise RuntimeError('Selected data is corrupted.')
            for key, value in selected.items():
                if key not in options:
                    raise RuntimeError('Selected data is corrupted.')
                self._validate_and_get_dependency(options[key], value, available_gid, selected_gid, required_gid)
        return (available_gid, selected_gid, required_gid)

    def _parameter_update(self, original_parameter, update_parameter):
        def _update_p(orig_config, update_cfg):
            orig_names = {c['name']: i for i, c in enumerate(orig_config)}
            new_config = []
            for c in update_cfg:
                if c['name'] not in orig_names:
                    new_config.append(c)
                    continue
                orig_config[orig_names[c['name']]] = c
            orig_config.extend(new_config)

        for t in ['bools', 'ints', 'strs', 'doubles']:
            if t not in original_parameter:
                original_parameter[t] = []

            _update_p(original_parameter[t], update_parameter[t])


class BackupDownloadMixin(LicenseVoidMixin):
    UNIX_EPOCH = timezone.datetime.fromtimestamp(0, timezone.utc)

    def _dump_map0(self, map_id):
        map = MapChangeset.objects.filter(map_id=map_id).first()
        if map and map.map:
            d = {
                'metadata': map.metadata,
                'structure': map.structure,
                'stations': map.stations,
                'map': {
                    'name': map.map.name,
                    'created': self._timestamp(map.map.created),
                },
            }

            if hasattr(map.map, 'mapannotation'):
                d['annotations'] = map.map.mapannotation.annotations
            return d

    def _timestamp(self, t):
        return (t - self.UNIX_EPOCH).total_seconds()

    def dump_info(self, data):
        info = {
            'version': get_current_version(),
            'name': Cache.get_agv_name(),
            'skillset_md5': Cache.get_models_skillset_md5(),
        }
        data['info'] = info


class BackupRestoreViewMixin(BackupRestoreMixin, ParameterMixin, object):

    @action(detail=False, methods=['POST'], serializer_class=serializers.Serializer)
    def restore(self, request):
        with suppress_signals():
            response = self._restore(request)
        mark_models_cache_as_dirty()
        return response

    def _restore(self, request):
        restore_file = request.FILES.get('restore_file')
        restore_choices = json.loads(request.data['restore_choices'])

        if not self.request.user.has_perm('system.change_protected_parameter') \
                and 'protected_parameter' in restore_choices:
            restore_choices.remove('protected_parameter')

        restore_file.seek(0)
        zbuf = BytesIO(restore_file.read())
        try:
            try:
                with gzip.GzipFile(mode='rb', fileobj=zbuf) as zfile:
                    data = json.loads(zfile.read())
            except Exception:
                # try to read as uncompressed file, if extracting it fails
                data = json.loads(zbuf.getvalue())

            self.restore_choices = data['contents']
            if 'parameter' in self.restore_choices:
                idx = self.restore_choices.index('parameter')
                self.restore_choices.insert(idx, 'protected_parameter')

            for choice in restore_choices:
                if choice not in self.restore_choices:
                    raise RuntimeError(' %s is not in the backup file.' % choice)

            if 'map' in restore_choices:
                self.load_map(data)

            if 'task_template' in restore_choices:
                self.load_task_template(data)

            normal_parameter = 'parameter' in restore_choices
            protected_parameter = 'protected_parameter' in restore_choices
            if normal_parameter or protected_parameter:
                self.load_parameter(data, normal_parameter, protected_parameter)

            if 'variable' in restore_choices:
                self.load_variable(data)

            if 'audio' in restore_choices:
                self.load_audio(data)

            # messages.info(self.request, self.success_message)
            return Response({'result': True})
        except Exception as ex:
            logger.error('Some errors happen while restoring settings from file: %s', ex)
            self.success_message = None
            return Response({'result': False, 'error': 'Some errors happen while restoring settings from file'})

    @restore.mapping.put
    def get_restore_choices(self, request):
        restore_file = request.FILES.get('restore_file')
        restore_file.seek(0)
        zbuf = BytesIO(restore_file.read())
        initial = {}

        try:
            try:
                with gzip.GzipFile(mode='rb', fileobj=zbuf) as zfile:
                    data = json.loads(zfile.read())
            except Exception:
                # try to read as uncompressed file, if extracting it fails
                data = json.loads(zbuf.getvalue())

            if data['magic'] != self.magic:
                if data['magic'] == tracked_backup_file_magic:
                    raise RuntimeError('Invalid magic bytes, backup file uploaded is for tracked mode.')
                elif data['magic'] == trackless_backup_file_magic:
                    raise RuntimeError('Invalid magic bytes, backup file uploaded is for trackless mode.')
                elif data['magic'] in [tracked_fms_backup_file_magic, trackless_fms_backup_file_magic]:
                    raise RuntimeError('Invalid magic bytes, backup file uploaded is for DFleet.')
                else:
                    raise RuntimeError('Invalid magic bytes.')

            self.restore_choices = data['contents']
            initial['restore_choices'] = list(self.restore_choices)

            if 'parameter' in self.restore_choices:
                idx = self.restore_choices.index('parameter')
                self.restore_choices.insert(idx, 'protected_parameter')
                initial['restore_choices'].insert(idx, 'protected_parameter')

            self.restore_info = data.get('info', {})
            self.restore_info['skillset_img'] = '%s?size=128x128&set=set1&bgset=bg1' % reverse('hashicon:index', [self.restore_info['skillset_md5']], request=request) \
                if self.restore_info['skillset_md5'] else ''

            initial['restore_info'] = json.dumps(self.restore_info)

            disabled_choices = '[]'
            if 'protected_parameter' in self.restore_choices and \
                    not self.request.user.has_perm('system.change_protected_parameter'):
                disabled_choices = json.dumps(['protected_parameter'])

            initial['disabled_choices'] = disabled_choices

        except Exception as ex:
            logger.error('Some errors happen while restoring settings from file: %s', ex)
            self.success_message = None
            self.invalid_file = True

            return Response({'error': 'Failed to restore settings. %s' % ex})
        return Response(initial)

    def load_map(self, data):
        # for backward compatibility
        if not isinstance(data['map'], list):
            data['map'] = [data['map']]

        for m in data['map']:
            self._load_map0(m)

        # update active map
        active_map = ','.join(['%s' % m['map'].pk for m in data['map']])
        Variable.objects.update_or_create(name=Variable.ACTIVE_MAP,
            defaults={'value': active_map})

        # load teleport data
        Variable.objects.update_or_create(name=Variable.TELEPORT,
            defaults={'value': data.get('map_teleport', '')})

        # load transition_trigger data
        Variable.objects.update_or_create(name=Variable.TRANSITION_TRIGGER,
            defaults={'value': data.get('map_transition_trigger', '')})

        # load map_param data
        if data.get('map_param') is not None:  # for backward compatiblity
            Variable.objects.update_or_create(name=Variable.MAP_PARAM,
                defaults={'value': data.get('map_param', '')})

    def _load_map0(self, map):
        if map:
            if map.get('map'):
                map['map'] = self._restore_map_session(map['map'])
            else:
                # for backward compatiblity
                map['map'] = self._create_placeholder_map()

            # update or create map annotation
            if map.get('annotations'):
                MapAnnotation.objects.update_or_create(
                    map=map['map'],
                    defaults={
                        'annotations': map['annotations'],
                    }
                )
                del map['annotations']

            # create map changeset
            map.update({
                'author': self.request.user,
            })
            MapChangeset.objects.create(**map)

    def _create_placeholder_map(self):
        name = 'Restored map placeholder'
        data = {
            'name': name,
            'is_named': True,
        }
        while True:
            try:
                map_session = Map.objects.create(**data)
            except IntegrityError:
                data['name'] = self._unique_suffix(name)
            else:
                break
        return map_session

    def _restore_map_session(self, data):
        # decode data
        data['created'] = self._from_timestamp(data['created'])

        # generate unique name in case of collision
        name = data['name']
        while True:
            try:
                map_session, created = Map.objects.update_or_create(
                    created=data['created'], defaults=data)
            except IntegrityError:
                data['name'] = self._unique_suffix(name)
            else:
                break

        # overwrite automatic timestamp.
        if created:
            Map.objects.filter(pk=map_session.pk).update(created=data['created'])
        return map_session

    def load_task_template(self, data):
        TaskTemplate.objects.all().delete()
        for d in data['task_template']:
            allowed_groups = d.pop('allowed_groups', [])
            allowed_users = d.pop('allowed_users', [])
            tt = TaskTemplate.objects.create(**d)
            tt.allowed_users.set(User.objects.filter(username__in=allowed_users))
            tt.allowed_groups.set(Group.objects.filter(name__in=allowed_groups))

        # load global_param data
        Variable.objects.update_or_create(name=Variable.GLOBAL_PARAM,
            defaults={'value': data.get('task_template_global_param', '[]')})

        # load variable data
        Variable.objects.update_or_create(name=Variable.VARIABLE,
            defaults={'value': data.get('task_template_variable', '[]')})

    def load_parameter(self, data, normal_parameter, protected_parameter):
        for p in data['parameter']:
            self._load_parameter0(p, normal_parameter, protected_parameter)

    def _load_parameter0(self, parameter_data, normal_parameter=True, protected_parameter=True):
        def _filter_parameter(c):
            if normal_parameter and protected_parameter:
                return True
            elif normal_parameter:
                return c['name'].endswith('_')
            else:
                return not c['name'].endswith('_')

        client = self.get_dyncfg_client(parameter_data['key'], reload=True)
        if client:
            # update to dynamic reconfigure server
            try:
                config_msg = full_load(parameter_data['value'])
                config = {c['name']: c['value'] for c in config_msg['bools'] + config_msg['ints'] + config_msg['strs'] + config_msg['doubles'] if _filter_parameter(c)}
                client.update_configuration(config)
            except Exception:
                pass
            else:
                return

        try:
            parameter = Parameter.objects.filter(key=parameter_data['key']).first()
            original_msg = full_load(parameter.value) if parameter else {}
            config_msg = full_load(parameter_data['value'])

            if not normal_parameter or not protected_parameter:
                config_msg['bools'] = filter(_filter_parameter, config_msg['bools'])
                config_msg['ints'] = filter(_filter_parameter, config_msg['ints'])
                config_msg['strs'] = filter(_filter_parameter, config_msg['strs'])
                config_msg['doubles'] = filter(_filter_parameter, config_msg['doubles'])

            self._parameter_update(original_msg, config_msg)
            parameter_data['value'] = yaml.dump(original_msg)
        except Exception as ex:
            logger.error('Restore offline parameter error: %s', ex)
            return

        # restore to database since dynamic reconfigure server is away
        Parameter.objects.update_or_create(key=parameter_data['key'], defaults=parameter_data)

    def load_variable(self, data):
        for v in data['variable']:
            if v['name'] in self.variable_backup_list:
                Variable.objects.update_or_create(
                    name=v['name'], defaults={'value': v['value']})

    def load_audio(self, data):
        try:
            for folder in default_storage.listdir('audio')[0]:
                for filename in default_storage.listdir(os.path.join('audio', folder))[1]:
                    default_storage.delete(os.path.join('audio', folder, filename))
        except Exception:
            pass

        for a in data['audio']:
            if 'param' in a:
                try:
                    audio_cfg = full_load(a['param']['value'])
                    for c in audio_cfg['strs']:
                        if c['name'] == 'media_root_':
                            c['value'] = default_storage.location
                            a['param']['value'] = yaml.dump(audio_cfg)
                            break
                except Exception:
                    pass
                self._load_parameter0(a['param'])
            else:
                path = os.path.join('audio', a['path'])
                content = base64.b64decode(a['bin'])
                default_storage.save(path, ContentFile(content))

    def _from_timestamp(self, timestamp):
        return timezone.datetime.fromtimestamp(timestamp, timezone.utc)

    def _unique_suffix(self, name):
        _name = name if len(name) < NAME_MAX_LENGTH - 11 else name[:NAME_MAX_LENGTH - 14] + '...'
        return '%s - %.8s' % (_name, uuid.uuid4())


class BackupAdvancedRestoreViewMixin(BackupRestoreViewMixin):

    @action(detail=False, methods=['POST'], serializer_class=serializers.Serializer, url_path='advanced-restore')
    def advanced_restore(self, request):
        with suppress_signals():
            response = self._advanced_restore(request)
        mark_models_cache_as_dirty()
        return response

    def _advanced_restore(self, request):
        restore_file = request.FILES.get('restore_file')
        restore_file.seek(0)
        zbuf = BytesIO(restore_file.read())
        try:
            selected_tree = {}
            self.mode = request.data['mode']
            try:
                selected_tree = json.loads(request.data['selected_tree'])
            except Exception as ex:
                logger.error('Some errors happen while parsing selected option: %s', ex)

            try:
                with gzip.GzipFile(mode='rb', fileobj=zbuf) as zfile:
                    data = json.loads(zfile.read())
            except Exception:
                # try to read as uncompressed file, if extracting it fails
                data = json.loads(zbuf.getvalue())

            # TODO: find a better way to reuse options_tree instead of compute again
            self.restore_choices = data['contents']
            self.options_tree = {}
            if 'map' in self.restore_choices:
                d = self.build_restore_map_tree(data)
                if d:
                    self.options_tree['map'] = d
            if 'task_template' in self.restore_choices:
                d = self.build_restore_task_template_tree(data)
                if d:
                    self.options_tree['task_template'] = d
            if 'parameter' in self.restore_choices:
                d = self.build_restore_parameter_tree(data)
                if d:
                    self.options_tree['parameter'] = d
            if 'variable' in self.restore_choices:
                d = self.build_restore_variable_tree(data)
                if d:
                    self.options_tree['variable'] = d
            if 'audio' in self.restore_choices:
                d = self.build_restore_audio_tree(data)
                if d:
                    self.options_tree['audio'] = d

            error = self.validate_selected_tree(self.options_tree, selected_tree)
            if error:
                raise RuntimeError(error)

            # Expose mapping for other component.
            self.tt_id_map = {}
            if 'task_template' in selected_tree:
                self.advanced_load_task_template(data, selected_tree['task_template'])

            if 'map' in selected_tree:
                self.advanced_load_map(data, selected_tree['map'])

            if self.mode == "restore":
                if 'parameter' in selected_tree:
                    self.advanced_load_parameter(data, selected_tree['parameter'])

                if 'variable' in selected_tree:
                    self.advanced_load_variable(data, selected_tree['variable'])

                if 'audio' in selected_tree:
                    self.advanced_load_audio(data, selected_tree['audio'])

            # messages.info(self.request, self.success_message)
            return Response({'result': True})

        except Exception as ex:
            logger.error('Some errors happen while restoring settings from file: %s', ex)
            self.success_message = None
            return Response({'result': False, 'error': 'Some errors happen while restoring settings from file'})

    @advanced_restore.mapping.put
    def get_advanced_restore_options(self, request):
        restore_file = request.FILES.get('restore_file')
        restore_file.seek(0)
        zbuf = BytesIO(restore_file.read())
        initial = {}

        try:
            try:
                with gzip.GzipFile(mode='rb', fileobj=zbuf) as zfile:
                    data = json.loads(zfile.read())
            except Exception:
                # try to read as uncompressed file, if extracting it fails
                data = json.loads(zbuf.getvalue())

            if data['magic'] != self.magic:
                if data['magic'] == tracked_backup_file_magic:
                    raise RuntimeError('Invalid magic bytes, backup file uploaded is for tracked mode.')
                elif data['magic'] == trackless_backup_file_magic:
                    raise RuntimeError('Invalid magic bytes, backup file uploaded is for trackless mode.')
                elif data['magic'] in [tracked_fms_backup_file_magic, trackless_fms_backup_file_magic]:
                    raise RuntimeError('Invalid magic bytes, backup file uploaded is for DFleet.')
                else:
                    raise RuntimeError('Invalid magic bytes.')

            self.restore_choices = data['contents']
            self.options_tree = {}
            if 'map' in self.restore_choices:
                d = self.build_restore_map_tree(data)
                if d:
                    self.options_tree['map'] = d
            if 'task_template' in self.restore_choices:
                d = self.build_restore_task_template_tree(data)
                if d:
                    self.options_tree['task_template'] = d
            if 'parameter' in self.restore_choices:
                d = self.build_restore_parameter_tree(data)
                if d:
                    self.options_tree['parameter'] = d
            if 'variable' in self.restore_choices:
                d = self.build_restore_variable_tree(data)
                if d:
                    self.options_tree['variable'] = d
            if 'audio' in self.restore_choices:
                d = self.build_restore_audio_tree(data)
                if d:
                    self.options_tree['audio'] = d

            initial['options_tree'] = json.dumps(self.options_tree)

            self.restore_info = data.get('info', {})
            self.restore_info['skillset_img'] = '%s?size=128x128&set=set1&bgset=bg1' % reverse('hashicon:index', [self.restore_info['skillset_md5']], request=request) \
                if self.restore_info['skillset_md5'] else ''

            initial['restore_info'] = json.dumps(self.restore_info)
        except Exception as ex:
            logger.error('Some errors happen while restoring settings from file: %s', ex)
            self.success_message = None
            self.invalid_file = True
            return Response({'result': False, 'error': 'Failed to restore settings. The file might have corrupted.'})
        return Response(initial)

    def advanced_load_map(self, data, selected):
        # for backward compatibility
        if not isinstance(data.get('map', []), list):
            data['map'] = [data['map']]

        if 'active_map' in selected:
            active_map = selected['active_map']
            active_map_id = [d['id'] for d in active_map]

            for i, m in enumerate(data.get('map', [])):
                if i not in active_map_id:
                    continue
                if self.mode == 'restore':
                    self._load_map0(m)
                else:
                    result = self._advanced_load_map0(m)
                    if not result:
                        active_map_id.remove(i)

            # update active map
            active_map_pk = [m['map'].pk for i, m in enumerate(data.get('map', [])) if i in active_map_id]
            if self.mode != 'restore':
                local_active_map_pk = []
                try:
                    local_active_map_pk = [int(v) for v in Variable.objects.get(name=Variable.ACTIVE_MAP).value.split(',')]
                except Exception:
                    pass
                for a in active_map_pk:
                    if a not in local_active_map_pk:
                        local_active_map_pk.append(a)
                active_map_pk = local_active_map_pk

            Variable.objects.update_or_create(name=Variable.ACTIVE_MAP,
                defaults={'value': ','.join(['%s' % i for i in active_map_pk])})

        if 'inactive_map' in selected:
            inactive_map = selected['inactive_map']
            inactive_map_id = [d['id'] for d in inactive_map]

            for i, m in enumerate(data.get('map_inactive', [])):
                if i not in inactive_map_id:
                    continue
                if self.mode == 'restore':
                    self._load_map0(m)
                else:
                    self._advanced_load_map0(m)

        # load teleport data
        if 'teleport' in selected:
            teleports = []
            try:
                teleports_raw = data.get('map_teleport', '[]')
                for key, val in self.tt_id_map.items():
                    teleports_raw = teleports_raw.replace('"_ttpk_%d"' % key, '"_ttpk_%d"' % val)

                teleports_data = json.loads(teleports_raw)
                selected_teleport = selected['teleport']
                selected_teleport_id = [s['id'] for s in selected_teleport]
                teleports = [t for i, t in enumerate(teleports_data) if i in selected_teleport_id]
            except Exception:
                pass

            if self.mode == 'restore':
                Variable.objects.update_or_create(name=Variable.TELEPORT,
                    defaults={'value': json.dumps(teleports)})
            else:
                filtered_list = []
                local_teleports = []
                local_teleports_dict = {}
                try:
                    local_teleports = json.loads(Variable.objects.get(name=Variable.TELEPORT).value)
                    for idx, t in enumerate(local_teleports):
                        e = (t['start'], t['end'])
                        local_teleports_dict[e] = idx
                except Exception:
                    pass

                if self.mode == 'rename':
                    filtered_list = local_teleports + teleports
                elif self.mode == 'overwrite':
                    for t in teleports:
                        e = (t['start'], t['end'])
                        if e in local_teleports_dict:
                            local_teleports[local_teleports_dict[e]] = t
                            continue
                        filtered_list.append(t)
                    filtered_list += local_teleports
                elif self.mode == 'preserve':
                    filtered_list = local_teleports
                    for t in teleports:
                        e = (t['start'], t['end'])
                        if e in local_teleports_dict:
                            continue
                        filtered_list.append(t)

                Variable.objects.update_or_create(name=Variable.TELEPORT,
                    defaults={'value': json.dumps(filtered_list)})

        # load transition_trigger data
        if 'transition_trigger' in selected:
            transition_triggers = []
            try:
                transition_triggers_raw = data.get('map_transition_trigger', '[]')
                for key, val in self.tt_id_map.items():
                    transition_triggers_raw = transition_triggers_raw.replace('"_ttpk_%d"' % key, '"_ttpk_%d"' % val)

                transition_triggers_data = json.loads(transition_triggers_raw)
                selected_transition_trigger = selected['transition_trigger']
                selected_transition_trigger_id = [s['id'] for s in selected_transition_trigger]
                transition_triggers = [t for i, t in enumerate(transition_triggers_data) if i in selected_transition_trigger_id]
            except Exception:
                pass

            if self.mode == 'restore':
                Variable.objects.update_or_create(name=Variable.TRANSITION_TRIGGER,
                    defaults={'value': json.dumps(transition_triggers)})
            else:
                filtered_list = []
                local_transition_triggers = []
                local_transition_triggers_dict = {}
                try:
                    local_transition_triggers = json.loads(Variable.objects.get(name=Variable.TRANSITION_TRIGGER).value)
                    for idx, t in enumerate(local_transition_triggers):
                        e = (t['start'], t['end'])
                        local_transition_triggers_dict[e] = idx
                except Exception:
                    pass

                if self.mode == 'rename':
                    filtered_list = local_transition_triggers + transition_triggers
                elif self.mode == 'overwrite':
                    for t in transition_triggers:
                        e = (t['start'], t['end'])
                        if e in local_transition_triggers_dict:
                            local_transition_triggers[local_transition_triggers_dict[e]] = t
                            continue
                        filtered_list.append(t)
                    filtered_list += local_transition_triggers
                elif self.mode == 'preserve':
                    filtered_list = local_transition_triggers
                    for t in transition_triggers:
                        e = (t['start'], t['end'])
                        if e in local_transition_triggers_dict:
                            continue
                        filtered_list.append(t)

                Variable.objects.update_or_create(name=Variable.TRANSITION_TRIGGER,
                    defaults={'value': json.dumps(filtered_list)})

        # load map_param data
        if 'map_param' in selected:
            map_param = []
            try:
                map_param_raw = data.get('map_param', '[]')
                map_param_data = json.loads(map_param_raw)
                selected_map_param = selected['map_param']
                selected_map_param_id = [s['id'] for s in selected_map_param]
                map_param = [p for i, p in enumerate(map_param_data) if i in selected_map_param_id]
            except Exception:
                pass

            if self.mode != 'restore':
                local_map_param = []
                try:
                    local_map_param = json.loads(Variable.objects.get(name=Variable.MAP_PARAM).value)
                except Exception:
                    pass
                local_map_param_names = [p['name'] for p in local_map_param]

                for p in map_param:
                    if p['name'] in local_map_param_names:
                        conflict_idx = local_map_param_names.index(p['name'])
                        lp = local_map_param[conflict_idx]

                        if p['type'] == lp['type']:

                            if self.mode == 'preserve':
                                continue
                            elif self.mode == 'overwrite':
                                local_map_param_names.pop(conflict_idx)
                                local_map_param.pop(conflict_idx)
                            elif self.mode == 'rename':
                                name = p['name']
                                while True:
                                    p['name'] = self._unique_suffix(name)
                                    if p['name'] not in local_map_param_names:
                                        break
                    local_map_param.append(p)
                    local_map_param_names.append(p['name'])

                map_param = local_map_param

            Variable.objects.update_or_create(name=Variable.MAP_PARAM,
                defaults={'value': json.dumps(map_param)})

    def _advanced_load_map0(self, map):
        map['map'] = self._advanced_restore_map_session(map['map'])

        if not map['map']:
            return False

        # update or create map annotation
        if map.get('annotations'):
            MapAnnotation.objects.update_or_create(
                map=map['map'],
                defaults={
                    'annotations': map['annotations'],
                }
            )
            del map['annotations']

        # create map changeset
        map.update({
            'author': self.request.user,
        })
        MapChangeset.objects.create(**map)
        return True

    def _advanced_restore_map_session(self, data):
        # decode data
        data['created'] = self._from_timestamp(data['created'])

        name = data['name']
        while True:
            try:
                map_session = Map.objects.create(**data)
            except IntegrityError as ex:
                if self.mode == 'preserve':
                    return
                elif self.mode == 'overwrite':
                    return Map.objects.get(name=data['name'])
                elif self.mode == 'rename':
                    data['name'] = self._unique_suffix(name)
            else:
                break

        # overwrite automatic timestamp.
        if not Map.objects.filter(created=data['created']).exists():
            Map.objects.filter(pk=map_session.pk).update(created=data['created'])
        return map_session

    def advanced_load_task_template(self, data, selected):
        if 'task_template' in selected:
            if self.mode == 'restore':
                TaskTemplate.objects.all().delete()

            selected_task_template = selected['task_template']
            if selected_task_template:
                # get selected tt id
                task_template_active = selected_task_template.get('active', {})
                task_template_inactive = selected_task_template.get('inactive', {})
                combined_task_template = task_template_active.get('top_level', []) + \
                    task_template_active.get('low_level', []) + \
                    task_template_inactive.get('top_level', []) + \
                    task_template_inactive.get('low_level', [])
                selected_task_template_id = [t['id'] for t in combined_task_template]

                added_task_template_id = []
                for d in data.get('task_template', []):
                    if d.get('id') not in selected_task_template_id:
                        continue

                    original_tt_id = d['id']

                    allowed_users = d.pop('allowed_users', [])
                    allowed_groups = d.pop('allowed_groups', [])

                    while True:
                        try:
                            tt = TaskTemplate.objects.create(**d)
                        except IntegrityError:
                            if self.mode == 'restore':
                                raise
                            conflict_tt = self._resolve_task_template_import(d)
                            if conflict_tt is not None:
                                tt = None
                                if conflict_tt != original_tt_id:
                                    self.tt_id_map[original_tt_id] = conflict_tt
                                break
                        else:
                            break
                    if tt:
                        added_task_template_id.append(tt.pk)
                        if original_tt_id != tt.pk:
                            self.tt_id_map[original_tt_id] = tt.pk

                        tt.allowed_users.set(User.objects.filter(username__in=allowed_users))
                        tt.allowed_groups.set(Group.objects.filter(name__in=allowed_groups))

                for pk in added_task_template_id:
                    try:
                        tt = TaskTemplate.objects.get(pk=pk)
                        for key, val in self.tt_id_map.items():
                            tt.structure = tt.structure.replace('"_ttpk_%d"' % key, '"_ttpk_%d"' % val)
                        tt.save()
                    except IntegrityError:
                        pass

        # load global_param data
        if 'global_param' in selected:
            task_template_global_param = []
            try:
                global_param = json.loads(data.get('task_template_global_param', '[]'))
                selected_global_param = selected['global_param']
                selected_global_param_id = [g['id'] for g in selected_global_param]
                task_template_global_param = [g for i, g in enumerate(global_param) if i in selected_global_param_id]
            except Exception:
                pass

            if self.mode != 'restore':
                local_tt_gp = []
                try:
                    local_tt_gp = json.loads(Variable.objects.get(name=Variable.GLOBAL_PARAM).value)
                except Exception:
                    pass
                local_tt_gp_names = [d['name'] for d in local_tt_gp]

                for gp in task_template_global_param:
                    if gp['name'] in local_tt_gp_names:
                        conflict_idx = local_tt_gp_names.index(gp['name'])
                        local_gp = local_tt_gp[conflict_idx]

                        if gp['type'] == local_gp['type']:

                            if self.mode == 'preserve':
                                continue
                            elif self.mode == 'overwrite':
                                local_tt_gp_names.pop(conflict_idx)
                                local_tt_gp.pop(conflict_idx)
                            elif self.mode == 'rename':
                                name = gp['name']
                                while True:
                                    gp['name'] = self._unique_suffix(name)
                                    if gp['name'] not in local_tt_gp_names:
                                        break
                    local_tt_gp.append(gp)
                    local_tt_gp_names.append(gp['name'])

                task_template_global_param = local_tt_gp

            Variable.objects.update_or_create(name=Variable.GLOBAL_PARAM,
                defaults={'value': json.dumps(task_template_global_param)})

        # load variable data
        if 'variable' in selected:
            variable_data = []
            try:
                variable = json.loads(data.get('task_template_variable', '[]'))
                selected_variable = selected['variable']
                selected_variable_id = [v['id'] for v in selected_variable]
                variable_data = [v for i, v in enumerate(variable) if i in selected_variable_id]
            except Exception:
                pass

            if self.mode != 'restore':
                local_v = []
                try:
                    local_v = json.loads(Variable.objects.get(name=Variable.VARIABLE).value)
                except Exception:
                    pass
                local_v_name = [v['name'] for v in local_v]

                for v in variable_data:
                    if v['name'] in local_v_name:
                        conflict_idx = local_v_name.index(v['name'])
                        lv = local_v[conflict_idx]

                        if v['type'] == lv['type']:

                            if self.mode == 'preserve':
                                continue
                            elif self.mode == 'overwrite':
                                local_v_name.pop(conflict_idx)
                                local_v.pop(conflict_idx)
                            elif self.mode == 'rename':
                                name = v['name']
                                while True:
                                    v['name'] = self._unique_suffix(name)
                                    if v['name'] not in local_v_name:
                                        break
                    local_v.append(v)
                    local_v_name.append(v['name'])

                variable_data = local_v

            Variable.objects.update_or_create(name=Variable.VARIABLE,
                defaults={'value': json.dumps(variable_data)})

    def _resolve_task_template_import(self, data):
        # Resolve task template conflict or return id in preserve mode

        try:
            # Name Conflict.
            tt = TaskTemplate.objects.get(name=data['name'])
            tt_metadata = json.loads(tt.metadata)
            d_metadata = json.loads(data['metadata'])
            same_tt = False

            # Check Params and Outcome
            try:
                if set(tt_metadata['outcomes']) == set(d_metadata['outcomes']):
                    same_tt = True
            except Exception:
                pass

            if self.mode == 'rename' or not same_tt:
                name = data['name']
                try:
                    while True:
                        data['name'] = self._unique_suffix(name)
                        TaskTemplate.objects.get(name=data['name'])
                except Exception:
                    pass
                if data.get('id') == tt.pk:
                    del data['id']
                return

            elif self.mode == 'preserve':
                return tt.pk

            elif self.mode == 'overwrite':
                data['id'] = tt.pk
                tt.delete()
                return
        except TaskTemplate.DoesNotExist:
            pass

        try:
            # Check PK conflict
            tt = TaskTemplate.objects.get(pk=data.get('id'))
            del data['id']
            return
        except TaskTemplate.DoesNotExist:
            pass

        raise IntegrityError('Fail to resolve task template import conflict.')

    def advanced_load_parameter(self, data, selected):
        selected_parameter_id = [p['id'] for p in selected]
        for p in data.get('parameter', []):

            protected_id = '_protected_%s' % p['key']
            normal_parameter = p['key'] in selected_parameter_id
            protected_parameter = protected_id in selected_parameter_id

            if not normal_parameter and not protected_parameter:
                continue
            self._load_parameter0(p, normal_parameter, protected_parameter)

    def advanced_load_variable(self, data, selected):
        selected_variable_id = [v['id'] for v in selected]

        for v in data.get('variable', []):
            if v['name'] not in self.variable_backup_list or v['name'] not in selected_variable_id:
                continue
            Variable.objects.update_or_create(
                name=v['name'], defaults={'value': v['value']})

    def advanced_load_audio(self, data, selected):
        if not isinstance(selected, list):
            return
        try:
            for folder in default_storage.listdir('audio')[0]:
                for filename in default_storage.listdir(os.path.join('audio', folder))[1]:
                    default_storage.delete(os.path.join('audio', folder, filename))
        except Exception:
            pass

        for a in data['audio']:
            if 'param' in a:
                try:
                    audio_cfg = full_load(a['param']['value'])
                    for c in audio_cfg['strs']:
                        if c['name'] == 'media_root_':
                            c['value'] = default_storage.location
                            a['param']['value'] = yaml.dump(audio_cfg)
                            break
                except Exception:
                    pass
                self._load_parameter0(a['param'])
            else:
                path = os.path.join('audio', a['path'])
                content = base64.b64decode(a['bin'])
                default_storage.save(path, ContentFile(content))

    def build_restore_map_tree(self, data):
        # for backward compatibility
        if not isinstance(data.get('map', []), list):
            data['map'] = [data['map']]

        # Active Map
        active_map_tree = []
        for i, m in enumerate(data.get('map', [])):
            active_map_tree.append({
                'id': i,
                'name': m['map']['name'] if m['map']['name'] else
                ('~Untitled~ [%s]' % timezone.datetime.fromtimestamp(m['map']['created']).strftime('%Y-%m-%d %H:%M:%S')),
                'default': True,
            })

        # Inactive Map
        inactive_map_tree = []
        for i, m in enumerate(data.get('map_inactive', [])):
            inactive_map_tree.append({
                'id': i,
                'name': m['map']['name'] if m['map']['name'] else
                ('~Untitled~ [%s]' % timezone.datetime.fromtimestamp(m['map']['created']).strftime('%Y-%m-%d %H:%M:%S')),
                'default': False,
            })

        # Teleport
        teleport_tree = []
        if 'map_teleport' in data:
            try:
                teleports = json.loads(data['map_teleport'])
                for i, t in enumerate(teleports):
                    t_dep = []
                    if t.get('preAction') and t['preAction'].get('skillId') \
                            and t['preAction']['skillId'].startswith('_ttpk_'):
                        t_dep.append(t['preAction']['skillId'])
                    if t.get('action') and t['action'].get('skillId') \
                            and t['action']['skillId'].startswith('_ttpk_'):
                        t_dep.append(t['action']['skillId'])

                    if t.get('start') and t.get('end'):
                        teleport_tree.append({
                            'id': i,
                            'name': 'From "%s" to "%s" station' % (t['start'], t['end']),
                            'dep': t_dep,
                            'default': True,
                        })
            except ValueError:
                pass

        # Transition Trigger
        transition_trigger_tree = []
        if 'map_transition_trigger' in data:
            try:
                transition_triggers = json.loads(data['map_transition_trigger'])
                for i, t in enumerate(transition_triggers):
                    t_dep = []
                    if t.get('startAction') and t['startAction'].get('skillId') \
                            and t['startAction']['skillId'].startswith('_ttpk_'):
                        t_dep.append(t['startAction']['skillId'])
                    if t.get('endAction') and t['endAction'].get('skillId') \
                            and t['endAction']['skillId'].startswith('_ttpk_'):
                        t_dep.append(t['endAction']['skillId'])

                    if t.get('start') and t.get('end'):
                        transition_trigger_tree.append({
                            'id': i,
                            'name': 'From "%s" to "%s" station' % (t['start'], t['end']),
                            'dep': t_dep,
                            'default': True,
                        })
            except ValueError:
                pass

        # Map Param
        map_param_tree = []
        if 'map_param' in data:
            try:
                map_param = json.loads(data['map_param'])
                for i, p in enumerate(map_param):
                    map_param_tree.append({
                        'id': i,
                        'name': p['name'],
                        'default': True,
                    })
            except ValueError:
                pass

        # build tree
        tree = {}
        if active_map_tree:
            tree['active_map'] = active_map_tree
        if inactive_map_tree:
            tree['inactive_map'] = inactive_map_tree
        if teleport_tree:
            tree['teleport'] = teleport_tree
        if transition_trigger_tree:
            tree['transition_trigger'] = transition_trigger_tree
        if map_param_tree:
            tree['map_param'] = map_param_tree
        return tree

    def build_restore_task_template_tree(self, data):
        # Task Template
        tt_active = {
            'top_level': [],
            'low_level': [],
        }
        tt_inactive = {
            'top_level': [],
            'low_level': [],
        }
        for tt in data.get('task_template', []):
            # get task template it depend on.
            tt_dep = re.findall(r'\"(_ttpk_\d+)\"', tt['structure'])
            # remove duplicate
            tt_dep = list(dict.fromkeys(tt_dep))
            d = {
                'id': tt['id'],
                'gid': '_ttpk_%s' % tt['id'],
                'name': tt['name'],
                'dep': tt_dep,
                'default': True,
            }

            if tt['is_active']:
                tt_active['top_level' if tt['is_top_level'] else 'low_level'].append(d)
            else:
                tt_inactive['top_level' if tt['is_top_level'] else 'low_level'].append(d)

        # Global Param
        global_param_tree = []
        if 'task_template_global_param' in data:
            try:
                global_param = json.loads(data['task_template_global_param'])
                for i, t in enumerate(global_param):
                    global_param_tree.append({
                        'id': i,
                        'name': t['name'],
                        'default': True,
                    })
            except ValueError:
                pass

        # Variable
        variable_tree = []
        if 'task_template_variable' in data:
            try:
                variable = json.loads(data['task_template_variable'])
                for i, v in enumerate(variable):
                    variable_tree.append({
                        'id': i,
                        'name': '%s (%s)' % (v['name'], v['type']),
                        'default': True,
                    })
            except ValueError:
                pass

        # build tree
        tree = {}
        if not tt_active['top_level']:
            del tt_active['top_level']
        if not tt_active['low_level']:
            del tt_active['low_level']
        if not tt_inactive['top_level']:
            del tt_inactive['top_level']
        if not tt_inactive['low_level']:
            del tt_inactive['low_level']

        if tt_active or tt_inactive:
            tree['task_template'] = {}
        if tt_active:
            tree['task_template']['active'] = tt_active
        if tt_inactive:
            tree['task_template']['inactive'] = tt_inactive
        if global_param_tree:
            tree['global_param'] = global_param_tree
        if variable_tree:
            tree['variable'] = variable_tree
        return tree

    def build_restore_parameter_tree(self, data):
        parameter_tree = []
        protected_parameter = self.request.user.has_perm('system.change_protected_parameter')
        for p in data.get('parameter', []):
            parameter_tree.append({
                'id': p['key'],
                'name': p['key'],
                'default': True,
            })
            parameter_tree.append({
                'id': '_protected_%s' % p['key'],
                'name': '%s (Protected Parameter)' % p['key'],
                'default': False,
                'disabled': not protected_parameter,
            })

        parameter_tree.sort(key=lambda p: p['name'])
        return parameter_tree

    def build_restore_variable_tree(self, data):
        variable_tree = []
        for v in data.get('variable', []):
            if v['name'] not in self.variable_backup_list:
                continue

            variable_tree.append({
                'id': v['name'],
                'name': v['name'],
                'default': True,
            })
        variable_tree.sort(key=lambda x: x['name'])
        return variable_tree

    def build_restore_audio_tree(self, data):
        audio = data.get('audio', [])
        if len(audio) <= 0:
            return []
        return [{
            'id': 'audio',
            'name': 'Audio',
            'default': True,
        }]


class BackupDownloadViewMixin(BackupDownloadMixin, object):
    BACKUP_CHOICES = (
        ('map', 'Map'),
        ('task_template', 'Task Template'),
        ('audio', 'Audio'),
        ('parameter', 'Parameter'),
        ('variable', 'Variable')
    )

    @action(detail=False, methods=['POST'], serializer_class=serializers.Serializer)
    def download(self, request):
        backup_choices = request.data['backup_choices']
        data = {
            'contents': [],
            'magic': self.magic,
        }

        if 'map' in backup_choices:
            self.dump_map(data)

        if 'task_template' in backup_choices:
            self.dump_task_template(data)

        if 'parameter' in backup_choices:
            self.dump_parameter(data)

        if 'variable' in backup_choices:
            self.dump_variable(data)

        if 'audio' in backup_choices:
            self.dump_audio(data)

        self.dump_info(data)

        zbuf = BytesIO()
        try:
            with gzip.GzipFile(mode='wb', fileobj=zbuf) as zfile:
                zfile.write(json.dumps(data).encode('utf-8'))
        except Exception as ex:
            logger.error('Some errors happen while backing up settings to file: %s', ex)

        filename = 'backup_%s.gz' % timezone.localtime().strftime('%Y%m%d_%H%M%S')
        response = HttpResponse(zbuf.getvalue(), content_type='application/x-gzip')
        response['Content-Disposition'] = 'attachment; filename="%s"' % filename
        return response

    @download.mapping.get
    def get_download_options(self, request):
        return Response({
            'backup_choices': self.BACKUP_CHOICES,
        })

    def dump_map(self, data):
        data['contents'].append('map')
        data['map'] = []
        active_map = []
        try:
            active_map = [int(v) for v in Variable.objects.get(name=Variable.ACTIVE_MAP).value.split(',')]
        except Exception:
            pass

        for m in active_map:
            d = self._dump_map0(m)
            if d:
                data['map'].append(d)

        # backup teleport data
        try:
            data['map_teleport'] = Variable.objects.get(name=Variable.TELEPORT).value
        except Exception:
            data['map_teleport'] = ''

        # backup teleport data
        try:
            data['map_transition_trigger'] = Variable.objects.get(name=Variable.TRANSITION_TRIGGER).value
        except Exception:
            data['map_transition_trigger'] = ''

        # backup map_param data
        try:
            data['map_param'] = Variable.objects.get(name=Variable.MAP_PARAM).value
        except Exception:
            data['map_param'] = ''

    def dump_task_template(self, data):
        data['contents'].append('task_template')
        data['task_template'] = []
        for tt in TaskTemplate.objects.prefetch_related('allowed_groups', 'allowed_users'):
            d = {
                'id': tt.id,
                'name': tt.name,
                'metadata': tt.metadata,
                'structure': tt.structure,
                'category': tt.category,
                'is_top_level': tt.is_top_level,
                'is_active': tt.is_active,
                'allowed_groups': [g.name for g in tt.allowed_groups.all()],
                'allowed_users': [u.username for u in tt.allowed_users.all()],
            }
            data['task_template'].append(d)

        # backup global_param data
        try:
            data['task_template_global_param'] = Variable.objects.get(name=Variable.GLOBAL_PARAM).value
        except Exception:
            data['task_template_global_param'] = '[]'

        # backup variable data
        try:
            data['task_template_variable'] = Variable.objects.get(name=Variable.VARIABLE).value
        except Exception:
            data['task_template_variable'] = '[]'

    def dump_parameter(self, data):
        data['contents'].append('parameter')
        data['parameter'] = list(Parameter.objects.all().values('key', 'value').exclude(key=django_settings.AGV05_AUDIO_PLAYER))

    def dump_variable(self, data):
        data['contents'].append('variable')
        data['variable'] = list(Variable.objects.filter(pk__in=self.variable_backup_list).values('name', 'value'))

    def dump_audio(self, data):
        data['contents'].append('audio')
        data['audio'] = []
        try:
            data['audio'].append({
                'param': Parameter.objects.values('key', 'value').get(key=django_settings.AGV05_AUDIO_PLAYER)
            })
        except Exception:
            pass

        try:
            for folder in default_storage.listdir('audio')[0]:
                for filename in default_storage.listdir(os.path.join('audio', folder))[1]:
                    path = os.path.join(folder, filename)
                    audio_file = default_storage.open(os.path.join('audio', path))
                    d = {
                        'path': path,
                        'bin': base64.b64encode(audio_file.read()).decode(),
                    }
                    data['audio'].append(d)
        except Exception:
            pass


class BackupAdvancedDownloadViewMixin(BackupDownloadMixin, object):

    @action(detail=False, methods=['POST'], serializer_class=serializers.Serializer, url_path='advanced-download')
    def advanced_download(self, request):
        selected_tree = {}
        try:
            selected_tree = json.loads(request.data['selected_tree'])
        except Exception as ex:
            logger.error('Some errors happen while parsing selected option: %s', ex)

        # TODO: find better way instead of compute options_tree again
        # TODO: handle error
        error = self.validate_selected_tree(self.get_options_tree(), selected_tree)
        if error:
            logger.error('Error when validating backup options: %s', error)
            return HttpResponse('Error', status=501)

        data = {
            'contents': [],
            'magic': self.magic,
        }

        if 'map' in selected_tree:
            self.advanced_dump_map(data, selected_tree['map'])

        if 'task_template' in selected_tree:
            self.advanced_dump_task_template(data, selected_tree['task_template'])

        if 'parameter' in selected_tree:
            self.advanced_dump_parameter(data, selected_tree['parameter'])

        if 'variable' in selected_tree:
            self.advanced_dump_variable(data, selected_tree['variable'])

        if 'audio' in selected_tree:
            self.advanced_dump_audio(data, selected_tree['audio'])

        self.dump_info(data)

        zbuf = BytesIO()
        try:
            with gzip.GzipFile(mode='wb', fileobj=zbuf) as zfile:
                zfile.write(json.dumps(data).encode('utf-8'))
        except Exception as ex:
            logger.error('Some errors happen while backing up settings to file: %s', ex)

        filename = 'backup_%s.gz' % timezone.localtime().strftime('%Y%m%d_%H%M%S')
        response = HttpResponse(zbuf.getvalue(), content_type='application/x-gzip')
        response['Content-Disposition'] = 'attachment; filename="%s"' % filename
        return response

    @advanced_download.mapping.get
    def get_advanced_download_options(self, request, *args, **kwargs):
        return Response({
            'backup_choices': self.get_options_tree(),
        })

    def get_options_tree(self):
        return {
            'map': self.build_download_map_tree(),
            'task_template': self.build_download_task_template_tree(),
            'parameter': self.build_download_parameter_tree(),
            'variable': self.build_download_variable_tree(),
            'audio': self.build_download_audio_tree(),
        }

    def build_download_map_tree(self):
        # Map
        active_map_tree = []
        inactive_map_tree = []
        active_map = []
        try:
            active_map = [int(v) for v in Variable.objects.get(name=Variable.ACTIVE_MAP).value.split(',')]
        except Exception:
            pass

        for m in Map.objects.all():
            map = MapChangeset.objects.filter(map_id=m.pk).first()
            if not map:
                continue
            if m.pk in active_map:
                active_map_tree.append({
                    'id': m.pk,
                    'name': str(m),
                    'default': True,
                })
            else:
                inactive_map_tree.append({
                    'id': m.pk,
                    'name': str(m),
                    'default': False,
                })

        # Teleport
        teleport_tree = []
        try:
            raw = Variable.objects.get(name=Variable.TELEPORT).value
            teleports = json.loads(raw)
            for i, t in enumerate(teleports):
                t_dep = []
                if t.get('preAction') and t['preAction'].get('skillId') \
                        and t['preAction']['skillId'].startswith('_ttpk_'):
                    t_dep.append(t['preAction']['skillId'])
                if t.get('action') and t['action'].get('skillId') \
                        and t['action']['skillId'].startswith('_ttpk_'):
                    t_dep.append(t['action']['skillId'])

                if t.get('start') and t.get('end'):
                    teleport_tree.append({
                        'id': i,
                        'name': 'From "%s" to "%s" station' % (t['start'], t['end']),
                        'dep': t_dep,
                        'default': True,
                    })
        except Exception:
            pass

        # Transition Trigger
        transition_trigger_tree = []
        try:
            raw = Variable.objects.get(name=Variable.TRANSITION_TRIGGER).value
            transition_triggers = json.loads(raw)
            for i, t in enumerate(transition_triggers):
                t_dep = []
                if t.get('startAction') and t['startAction'].get('skillId') \
                        and t['startAction']['skillId'].startswith('_ttpk_'):
                    t_dep.append(t['startAction']['skillId'])
                if t.get('endAction') and t['endAction'].get('skillId') \
                        and t['endAction']['skillId'].startswith('_ttpk_'):
                    t_dep.append(t['endAction']['skillId'])

                if t.get('start') and t.get('end'):
                    transition_trigger_tree.append({
                        'id': i,
                        'name': 'From "%s" to "%s" station' % (t['start'], t['end']),
                        'dep': t_dep,
                        'default': True,
                    })
        except Exception:
            pass

        # Map Param
        map_param_tree = []
        try:
            raw = Variable.objects.get(name=Variable.MAP_PARAM).value
            map_param = json.loads(raw)
            for i, p in enumerate(map_param):
                map_param_tree.append({
                    'id': i,
                    'name': p['name'],
                    'default': True,
                })
        except Exception:
            pass

        # build tree
        tree = {}
        if active_map_tree:
            tree['active_map'] = active_map_tree
        if inactive_map_tree:
            tree['inactive_map'] = inactive_map_tree
        if teleport_tree:
            tree['teleport'] = teleport_tree
        if transition_trigger_tree:
            tree['transition_trigger'] = transition_trigger_tree
        if map_param_tree:
            tree['map_param'] = map_param_tree
        return tree

    def build_download_task_template_tree(self):
        # Task Template
        tt_active = {
            'top_level': [],
            'low_level': [],
        }
        tt_inactive = {
            'top_level': [],
            'low_level': [],
        }
        for tt in TaskTemplate.objects.all():
            # get task template it depend on.
            tt_dep = re.findall(r'\"(_ttpk_\d+)\"', tt.structure)
            # remove duplicate
            tt_dep = list(dict.fromkeys(tt_dep))
            d = {
                'id': tt.id,
                'gid': '_ttpk_%s' % tt.id,
                'name': tt.name,
                'dep': tt_dep,
                'default': True,
            }

            if tt.is_active:
                tt_active['top_level' if tt.is_top_level else 'low_level'].append(d)
            else:
                tt_inactive['top_level' if tt.is_top_level else 'low_level'].append(d)

        # Global Param
        global_param_tree = []
        try:
            raw = Variable.objects.get(name=Variable.GLOBAL_PARAM).value
            global_param = json.loads(raw)
            for i, t in enumerate(global_param):
                global_param_tree.append({
                    'id': i,
                    'name': t['name'],
                    'default': True,
                })
        except Exception:
            pass

        # Variable
        variable_tree = []
        try:
            raw = Variable.objects.get(name=Variable.VARIABLE).value
            variable = json.loads(raw)
            for i, v in enumerate(variable):
                variable_tree.append({
                    'id': i,
                    'name': '%s (%s)' % (v['name'], v['type']),
                    'default': True,
                })
        except Exception:
            pass

        # build tree
        tree = {}
        if not tt_active['top_level']:
            del tt_active['top_level']
        if not tt_active['low_level']:
            del tt_active['low_level']
        if not tt_inactive['top_level']:
            del tt_inactive['top_level']
        if not tt_inactive['low_level']:
            del tt_inactive['low_level']

        if tt_active or tt_inactive:
            tree['task_template'] = {}
        if tt_active:
            tree['task_template']['active'] = tt_active
        if tt_inactive:
            tree['task_template']['inactive'] = tt_inactive
        if global_param_tree:
            tree['global_param'] = global_param_tree
        if variable_tree:
            tree['variable'] = variable_tree
        return tree

    def build_download_parameter_tree(self):
        parameter_tree = []
        for p in Parameter.objects.exclude(key=django_settings.AGV05_AUDIO_PLAYER):
            parameter_tree.append({
                'id': p.key,
                'name': p.key,
                'default': True,
            })
        parameter_tree.sort(key=lambda x: x['name'])
        return parameter_tree

    def build_download_variable_tree(self):
        variable_tree = []
        for v in self.variable_backup_list:
            variable_tree.append({
                'id': v,
                'name': v,
                'default': True,
            })
        variable_tree.sort(key=lambda x: x['name'])
        return variable_tree

    def build_download_audio_tree(self):
        return [{
            'id': 'audio',
            'name': 'Audio',
            'default': True,
        }]

    def advanced_dump_map(self, data, selected):
        data['contents'].append('map')
        data['map'] = []
        data['map_inactive'] = []
        active_map = selected.get('active_map', [])
        inactive_map = selected.get('inactive_map', [])

        for m in active_map:
            d = self._dump_map0(m['id'])
            if d:
                data['map'].append(d)

        for m in inactive_map:
            d = self._dump_map0(m['id'])
            if d:
                data['map_inactive'].append(d)

        # backup teleport data
        try:
            raw = Variable.objects.get(name=Variable.TELEPORT).value
            teleports = json.loads(raw)
            selected_teleport = selected['teleport']
            selected_teleport_id = [s['id'] for s in selected_teleport]
            data['map_teleport'] = json.dumps([t for i, t in enumerate(teleports) if i in selected_teleport_id])
        except Exception:
            data['map_teleport'] = ''

        # backup transition_trigger data
        try:
            raw = Variable.objects.get(name=Variable.TRANSITION_TRIGGER).value
            transition_triggers = json.loads(raw)
            selected_transition_trigger = selected['transition_trigger']
            selected_transition_trigger_id = [s['id'] for s in selected_transition_trigger]
            data['map_transition_trigger'] = json.dumps([t for i, t in enumerate(transition_triggers) if i in selected_transition_trigger_id])
        except Exception:
            data['map_transition_trigger'] = ''

        # backup map_param data
        try:
            raw = Variable.objects.get(name=Variable.MAP_PARAM).value
            map_param = json.loads(raw)
            selected_map_param = selected['map_param']
            selected_map_param_id = [p['id'] for p in selected_map_param]
            data['map_param'] = json.dumps([p for i, p in enumerate(map_param) if i in selected_map_param_id])
        except Exception:
            data['map_param'] = ''

    def advanced_dump_task_template(self, data, selected):
        data['contents'].append('task_template')
        data['task_template'] = []

        selected_task_template = selected.get('task_template')
        if selected_task_template:
            task_template_active = selected_task_template.get('active', {})
            task_template_inactive = selected_task_template.get('inactive', {})
            combined_task_template = task_template_active.get('top_level', []) + \
                task_template_active.get('low_level', []) + \
                task_template_inactive.get('top_level', []) + \
                task_template_inactive.get('low_level', [])
            selected_task_template_id = [t['id'] for t in combined_task_template]

            for tt in TaskTemplate.objects.prefetch_related('allowed_groups', 'allowed_users'):
                if tt.id not in selected_task_template_id:
                    continue

                d = {
                    'id': tt.id,
                    'name': tt.name,
                    'metadata': tt.metadata,
                    'structure': tt.structure,
                    'category': tt.category,
                    'is_top_level': tt.is_top_level,
                    'is_active': tt.is_active,
                    'allowed_groups': [g.name for g in tt.allowed_groups.all()],
                    'allowed_users': [u.username for u in tt.allowed_users.all()],
                }
                data['task_template'].append(d)

        # backup global_param data
        try:
            raw = Variable.objects.get(name=Variable.GLOBAL_PARAM).value
            global_param = json.loads(raw)
            selected_global_param = selected['global_param']
            selected_global_param_id = [g['id'] for g in selected_global_param]
            data['task_template_global_param'] = json.dumps([g for i, g in enumerate(global_param) if i in selected_global_param_id])
        except Exception:
            data['task_template_global_param'] = '[]'

        # backup variable data
        try:
            raw = Variable.objects.get(name=Variable.VARIABLE).value
            variable = json.loads(raw)
            selected_variable = selected['variable']
            selected_variable_id = [v['id'] for v in selected_variable]
            data['task_template_variable'] = json.dumps([v for i, v in enumerate(variable) if i in selected_variable_id])
        except Exception:
            data['task_template_variable'] = '[]'

    def advanced_dump_parameter(self, data, selected):
        data['contents'].append('parameter')
        selected_parameter_id = [p['id'] for p in selected]
        data['parameter'] = list(Parameter.objects.filter(key__in=selected_parameter_id).values('key', 'value'))

    def advanced_dump_variable(self, data, selected):
        data['contents'].append('variable')
        selected_variable_id = [v['id'] for v in selected]
        data['variable'] = list(Variable.objects.filter(pk__in=selected_variable_id).values('name', 'value'))

    def advanced_dump_audio(self, data, selected):
        data['contents'].append('audio')
        data['audio'] = []
        try:
            data['audio'].append({
                'param': Parameter.objects.values('key', 'value').get(key=django_settings.AGV05_AUDIO_PLAYER)
            })
        except Exception:
            pass

        try:
            for folder in default_storage.listdir('audio')[0]:
                for filename in default_storage.listdir(os.path.join('audio', folder))[1]:
                    path = os.path.join(folder, filename)
                    audio_file = default_storage.open(os.path.join('audio', path))
                    d = {
                        'path': path,
                        'bin': base64.b64encode(audio_file.read()).decode(),
                    }
                    data['audio'].append(d)
        except Exception:
            pass


class BackupConfigViewSet(
    BackupAdvancedDownloadViewMixin,
    BackupDownloadViewMixin,
    BackupAdvancedRestoreViewMixin,
    BackupRestoreViewMixin,
    viewsets.ViewSet
):
    serializer_class = serializers.Serializer

    def list(self, request, *args, **kwargs):
        return Response(OrderedDict([
            ('download', reverse('app:config-api:backup-download', request=request)),
            ('advanced-download', reverse('app:config-api:backup-advanced-download', request=request)),
            ('restore', reverse('app:config-api:backup-restore', request=request)),
            ('advanced-restore', reverse('app:config-api:backup-advanced-restore', request=request)),
        ]))
