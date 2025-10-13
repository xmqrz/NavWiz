from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import PathFacing, Variable
from agv05_webserver.systemx.models import Heading, PathShape

from .agv import StandaloneValidator
from .validator import ValidationError
from ..map_tracker_x import MapTrackerX

AGV_PARAM = {
    'agv': {
        'name': 'AGV',
        'id': 'agv',
        'item': '',
    }
}


class StandaloneXValidator(StandaloneValidator):

    def _validate_home(self):
        # Validate agv_home
        try:
            self.agv_home = Variable.objects.get(pk=Variable.AGV_HOME).value
        except Exception:
            raise ValidationError('{agv} home is not specified.', params=AGV_PARAM)

        if self.agv_home not in self.models.tmp['station_names']:
            raise ValidationError('{agv} home is invalid.', params=AGV_PARAM)

        s = self.stations_assoc[self.agv_home]
        if s[1] == Heading.NA:
            raise ValidationError('{agv} home station cannot be headingless.', params=AGV_PARAM)

        for name, location in self.stations_assoc.items():
            if name == self.agv_home:
                continue
            if location[0] == s[0]:
                raise ValidationError('{agv} home station cannot overlap with another station.', params=AGV_PARAM)

        self._amend_graph()

    def _amend_graph(self):
        s = self.stations_assoc[self.agv_home]
        mt = MapTrackerX()
        mt.set_map(self.graph)

        for e in self.graph.out_edges_iter(s[0], data=True):
            if e[2]['shape'] == PathShape.TELEPORT:
                continue

            start_heading = mt.compute_edge_start_heading(e)
            start_heading_inv = mt.invert_heading(start_heading)

            if mt.is_heading_equal(start_heading, s[1]):
                if e[2]['facing'] == PathFacing.REVERSE_UNI:
                    raise ValidationError('Path\'s reverse motion from {agv} home station does not align with the direction of the station itself.', params=AGV_PARAM)
                e[2]['facing'] = PathFacing.FORWARD_UNI

            elif mt.is_heading_equal(start_heading_inv, s[1]):
                if e[2]['facing'] == PathFacing.FORWARD_UNI:
                    raise ValidationError('Path\'s forward motion from {agv} home station does not align with the direction of the station itself.', params=AGV_PARAM)
                e[2]['facing'] = PathFacing.REVERSE_UNI

            else:
                raise ValidationError('Path from {agv} home station must align with the direction of the station itself.', params=AGV_PARAM)

        for e in self.graph.in_edges_iter(s[0], data=True):
            if e[2]['shape'] == PathShape.TELEPORT:
                continue

            end_heading = mt.compute_edge_end_heading(e)
            end_heading_inv = mt.invert_heading(end_heading)

            if mt.is_heading_equal(end_heading, s[1]):
                if e[2]['facing'] == PathFacing.REVERSE_UNI:
                    raise ValidationError('Path\'s reverse motion to {agv} home station does not align with the direction of the station itself.', params=AGV_PARAM)
                e[2]['facing'] = PathFacing.FORWARD_UNI

            elif mt.is_heading_equal(end_heading_inv, s[1]):
                if e[2]['facing'] == PathFacing.FORWARD_UNI:
                    raise ValidationError('Path\'s forward motion to {agv} home station does not align with the direction of the station itself.', params=AGV_PARAM)
                e[2]['facing'] = PathFacing.REVERSE_UNI

            else:
                raise ValidationError('Path to {agv} home station must align with the direction of the station itself.', params=AGV_PARAM)
