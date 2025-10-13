from __future__ import absolute_import
from __future__ import unicode_literals

from django.core.urlresolvers import reverse
from django.utils import timezone
from django.utils.encoding import force_text
import celery
import logging
import requests
import time
import ujson as json

from ..models import Cache, ExecutorMode, Map, Variable, redis_cache


logger = logging.getLogger(__name__)


class maintain_fms_pairing(celery.Task):
    ignore_result = True

    LOCK_TIMEOUT = 5 * 60
    POLLING_INTERVAL = [5, 10]
    POLLING_DURATION = [3 * 60, 9 * 60]  # cumulative

    UNIX_EPOCH = timezone.datetime.fromtimestamp(0, timezone.utc)

    def run(self, polling=False, upload_keys=set()):
        # ensure single process working on this task at one time
        if not redis_cache.add('maintain_fms_pairing_lock', True, self.LOCK_TIMEOUT):
            return

        try:
            result = self._run0(polling, upload_keys)
        except Exception as ex:
            result = force_text(ex)

        redis_cache.delete('maintain_fms_pairing_lock')
        return result

    def _run0(self, polling, upload_keys):
        try:
            fms_polling_active = bool(redis_cache.get('fms_polling_active'))
        except Exception:
            fms_polling_active = False

        logger.debug('polling: %s, polling_active: %s' % (polling, fms_polling_active))
        if not polling:
            if fms_polling_active:
                return
            upload_keys = set()

        try:
            result = self._sync_uploads(upload_keys)
        except Exception as ex:
            result = force_text(ex)

        if result and isinstance(result, set):
            redis_cache.set('fms_polling_active', True, self.LOCK_TIMEOUT)
            redis_cache.set('fms_polling_request', time.time(), None)
            # delay a little for subsequent uploads
            interval = 1 if not upload_keys else self.POLLING_INTERVAL[0]
            self.apply_async(kwargs={
                'polling': True,
                'upload_keys': result,
            }, countdown=interval, expires=20)
        else:
            dt = time.time() - redis_cache.get('fms_polling_request', 0)
            for interval, duration in zip(self.POLLING_INTERVAL, self.POLLING_DURATION):
                if dt < duration:
                    redis_cache.set('fms_polling_active', True, self.LOCK_TIMEOUT)
                    self.apply_async(kwargs={
                        'polling': True,
                    }, countdown=interval, expires=20)
                    break
            else:
                redis_cache.set('fms_polling_active', False, None)
        return result

    def _sync_uploads(self, upload_keys):
        vars_list = [Variable.AGV_NAME, Variable.EXECUTOR_MODE, Variable.FMS_METADATA]
        data = dict(Variable.objects.filter(pk__in=vars_list).values_list('name', 'value'))
        data['agv_uuid'] = Variable.get_agv_uuid()

        if int(data.get('executor_mode', 0)) != ExecutorMode.DFleet.value:
            return

        try:
            fms_metadata = json.loads(data['fms_metadata'])
            endpoint = fms_metadata['endpoint']

            headers = {
                'Authorization': 'AgvToken %s:%s' % (data['agv_uuid'], fms_metadata['token']),
                'Content-Type': 'application/json',
            }
            agv_data = {
                'name': data['agv_name'],
                # TODO: handle reverse.. using placeholder view?
                'config_panel': reverse('config-panel'),
                'skillset': Cache.get_models_skillset(),
                'skillset_md5': Cache.get_models_skillset_md5(),
                'uploads': self._prepare_uploads(upload_keys),
            }
            data = json.dumps(agv_data)

            # reset 'maintain_fms_pairing_lock' expiry proportionally to the size of data
            timeout_scale = len(data) // 1e7 + 1
            redis_cache.set('maintain_fms_pairing_lock', True, self.LOCK_TIMEOUT * timeout_scale)

            # send request to fms
            r = requests.put(endpoint, headers=headers, data=data, timeout=(3.05, 21 * timeout_scale),
                    allow_redirects=False, verify=False)
            if r.status_code == 200:
                try:
                    data = json.loads(r.content)['data']
                    upload_keys = data['uploads']

                    # upload_keys are originally represented as a `set of tuple` before jsonification.
                    assert (isinstance(upload_keys, list))
                    assert (all([isinstance(u, list) for u in upload_keys]))
                    upload_keys = set([tuple(u) for u in upload_keys])

                except Exception as ex:
                    raise RuntimeError('Data format error: %s' % ex)
                else:
                    return upload_keys
            else:
                try:
                    detail = json.loads(r.content)['detail']
                except Exception:
                    detail = r.reason
                raise RuntimeError('Got (%s) %s' % (r.status_code, detail))

        except requests.exceptions.ConnectionError:
            return 'Connection error'

        except Exception as ex:
            return force_text(ex)

    def _prepare_uploads(self, keys):
        data = []
        if not isinstance(keys, set):
            return data

        for k in keys:
            try:
                data.append([k, self._prepare_upload0(k)])
                # prevent bulky uploads by allowing one map_detail at a time
                if k[0] == 'map_detail':
                    break
            except Exception as ex:
                logger.error('Error preparing upload: %s', ex, exc_info=1)
        return data

    def _prepare_upload0(self, key):
        if key[0] == 'map_list':
            return [{
                'id': m.id,
                'name': force_text(m),
                'created': self._timestamp(m.created),
            } for m in Map.objects.all()[:50]]  # truncate to 50 objects

        elif key[0] == 'map_detail':
            return self._dump_map(key[1])

    def _dump_map(self, map_id):
        m = Map.objects.filter(id=map_id).first()
        if not m:
            return None

        d = {
            'name': m.name,
            'created': self._timestamp(m.created),
        }

        mc = m.mapchangeset_set.first()
        if mc:
            d['mapchangeset'] = {
                'metadata': mc.metadata,
                'structure': mc.structure,
                'stations': mc.stations,
            }

        if hasattr(m, 'mapannotation'):
            ma = m.mapannotation
            d['mapannotation'] = {
                'annotations': ma.annotations,
            }

        return d

    def _timestamp(self, t):
        return (t - self.UNIX_EPOCH).total_seconds()
