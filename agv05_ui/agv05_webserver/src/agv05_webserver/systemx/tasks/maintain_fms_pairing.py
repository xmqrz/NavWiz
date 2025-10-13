from __future__ import absolute_import
from __future__ import unicode_literals

import base64

from ...system.models import Map
from ...system.tasks.maintain_fms_pairing import maintain_fms_pairing as system_maintain_fms_pairing


class maintain_fms_pairing(system_maintain_fms_pairing):

    def _dump_map(self, map_id):
        m = Map.objects.filter(id=map_id).first()
        if not m:
            return None

        d = {
            'name': m.name,
            'created': self._timestamp(m.created),
        }

        ocg_id = None
        mc = m.mapchangeset_set.first()
        if mc:
            d['mapchangeset'] = {
                'metadata': mc.metadata,
                'structure': mc.structure,
                'stations': mc.stations,
            }
            try:
                ocg_id = mc.mapchangeset.ocg_id
            except Exception:
                pass

        d['mapocg_set'] = []
        for ocg in m.mapocg_set.all()[:50]:  # truncate to 50 objects
            try:
                png_file = base64.b64encode(ocg.png_file.read()).decode()
            except Exception:
                pass
            else:
                d['mapocg_set'].append({
                    'name': ocg.name,
                    'metadata': ocg.metadata,
                    'png_file': png_file,
                    'is_autosaved': ocg.is_autosaved,
                    'created': self._timestamp(ocg.created),
                })
                if ocg.id == ocg_id:
                    d['mapchangeset']['ocg_id'] = len(d['mapocg_set']) - 1

        if hasattr(m, 'mapannotation'):
            ma = m.mapannotation
            d['mapannotation'] = {
                'annotations': ma.annotations,
            }

        return d
