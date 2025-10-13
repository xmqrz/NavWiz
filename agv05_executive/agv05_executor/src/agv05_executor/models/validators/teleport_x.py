from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.systemx.models import Heading, PathShape

from .teleport import TeleportValidator


class TeleportXValidator(TeleportValidator):
    HEADING_NA = Heading.NA
    HEADING_WARNING = 'headingless'

    def _amend_graph(self):
        for idx, t in enumerate(self.teleports):
            j1 = self.stations_assoc[t['start']][0]
            j2 = self.stations_assoc[t['end']][0]

            kwargs = {
                'shape': PathShape.TELEPORT,
                'teleport': idx,
                'distance': t['distance'],
            }
            self.graph.add_edge(j1, j2, **kwargs)
