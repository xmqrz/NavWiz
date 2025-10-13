from __future__ import absolute_import
from __future__ import unicode_literals

from django.core.files.base import ContentFile
from django.db import IntegrityError
import base64

from agv05_webserver.app.api.config.backup_restore import \
    BackupRestoreMixin as AppBackupRestoreMixin, \
    BackupDownloadMixin as AppBackupDownloadMixin, \
    BackupDownloadViewMixin as AppBackupDownloadViewMixin, \
    BackupConfigViewSet as AppBackupConfigViewSet

from agv05_webserver.system.models import MapAnnotation
from agv05_webserver.systemx.models import MapOcg, MapChangeset

backup_file_magic = 'DF_AGV05X'
extra_variable_backup_list = []


class BackupRestoreMixin(object):
    magic = backup_file_magic
    variable_backup_list = AppBackupRestoreMixin.variable_backup_list + extra_variable_backup_list


class BackupDownloadMixin(AppBackupDownloadMixin):

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

            if map.ocg:
                try:
                    png_file = base64.b64encode(map.ocg.png_file.read()).decode()
                except Exception:
                    d['ocg'] = None
                else:
                    d['ocg'] = {
                        'name': map.ocg.name,
                        'metadata': map.ocg.metadata,
                        'png_file': png_file,
                        'is_autosaved': map.ocg.is_autosaved,
                        'created': self._timestamp(map.ocg.created),
                    }
            else:
                d['ocg'] = None

            if hasattr(map.map, 'mapannotation'):
                d['annotations'] = map.map.mapannotation.annotations
            return d


class BackupRestoreViewMixin(BackupRestoreMixin, AppBackupRestoreMixin):
    def _load_map0(self, map):
        if map and map.get('map'):
            map['map'] = self._restore_map_session(map['map'])
            if map.get('ocg'):
                map['ocg']['map'] = map['map']
                map['ocg'] = self._restore_map_ocg(map['ocg'])
            else:
                map['ocg'] = None

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

        if map.get('ocg'):
            map['ocg']['map'] = map['map']
            map['ocg'] = self._restore_map_ocg(map['ocg'])
        else:
            map['ocg'] = None

        # create map changeset
        map.update({
            'author': self.request.user,
        })
        MapChangeset.objects.create(**map)
        return True

    def _restore_map_ocg(self, data):
        # decode data
        png_file = base64.b64decode(data.pop('png_file'))
        data['created'] = self._from_timestamp(data['created'])

        # search for existing, matching ocg
        created = False
        ocgs = MapOcg.objects.filter(is_autosaved=data['is_autosaved'], created=data['created'])

        for ocg in ocgs:
            if ocg.metadata != data['metadata']:
                continue

            if ocg.png_file.read() != png_file:
                continue

            # update existing ocg if match found
            ocg.name = data['name']
            break
        else:
            # create new ocg if match not found
            ocg = MapOcg(**data)
            ocg.png_file.save('raw.png', ContentFile(png_file), save=False)
            created = True

        # generate unique name in case of collision
        while True:
            try:
                ocg.save()
            except IntegrityError:
                ocg.name = self._unique_suffix(data['name'])
            else:
                break

        # overwrite automatic timestamp.
        if created:
            MapOcg.objects.filter(pk=ocg.pk).update(created=data['created'])
        return ocg


class BackupDownloadViewMixin(BackupDownloadMixin, AppBackupDownloadViewMixin):
    def _load_map0(self, map):
        if map and map.get('map'):
            map['map'] = self._restore_map_session(map['map'])
            if map.get('ocg'):
                map['ocg']['map'] = map['map']
                map['ocg'] = self._restore_map_ocg(map['ocg'])
            else:
                map['ocg'] = None

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

        if map.get('ocg'):
            map['ocg']['map'] = map['map']
            map['ocg'] = self._restore_map_ocg(map['ocg'])
        else:
            map['ocg'] = None

        # create map changeset
        map.update({
            'author': self.request.user,
        })
        MapChangeset.objects.create(**map)
        return True

    def _restore_map_ocg(self, data):
        # decode data
        png_file = base64.b64decode(data.pop('png_file'))
        data['created'] = self._from_timestamp(data['created'])

        # search for existing, matching ocg
        created = False
        ocgs = MapOcg.objects.filter(is_autosaved=data['is_autosaved'], created=data['created'])

        for ocg in ocgs:
            if ocg.metadata != data['metadata']:
                continue

            if ocg.png_file.read() != png_file:
                continue

            # update existing ocg if match found
            ocg.name = data['name']
            break
        else:
            # create new ocg if match not found
            ocg = MapOcg(**data)
            ocg.png_file.save('raw.png', ContentFile(png_file), save=False)
            created = True

        # generate unique name in case of collision
        while True:
            try:
                ocg.save()
            except IntegrityError:
                ocg.name = self._unique_suffix(data['name'])
            else:
                break

        # overwrite automatic timestamp.
        if created:
            MapOcg.objects.filter(pk=ocg.pk).update(created=data['created'])


class BackupConfigViewSet(
    BackupRestoreViewMixin,
    BackupDownloadViewMixin,
    AppBackupConfigViewSet
):
    pass
