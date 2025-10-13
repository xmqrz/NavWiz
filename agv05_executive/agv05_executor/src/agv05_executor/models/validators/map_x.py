from __future__ import absolute_import
from __future__ import unicode_literals

from PIL import Image
from agv05_webserver.system.models import PathBranch, PathFacing, PathFlow
from agv05_webserver.systemx.models import Cache, Heading, MapChangeset, OccupancyGrid, PathShape
from collections import defaultdict
from six.moves import zip
import agv05_msgs.msg
import base64
import geometry_msgs.msg
import math
import networkx as nx
import numpy as np
import rospy
import six
import time
import ujson as json

from .map import MapValidator, SingleMapValidator, ConstrainedMapValidator
from .validator import ValidationError
from ..map_tracker_x import MapTrackerX

MAPS_PARAM = {
    'map': {
        'name': 'Maps',
        'id': 'map-list',
        'item': '',
    }
}


class MapXValidator(MapValidator):
    cache = Cache
    provides = ['ocg', 'map_structure', 'station_names']
    extra_provides = ['stations_assoc', 'graph', 'landmarks', 'forbidden_zones', 'no_rotate_zones']

    map_changeset_cls = MapChangeset
    single_map_validator = staticmethod(lambda x: SingleMapXValidator(x))

    def validate(self):
        self._load_maps()

        self.ocg = []
        self.map_structure = []
        self.station_names = set()
        self.stations_assoc = {}
        self.graph = nx.DiGraph()
        self.landmarks = []
        self.forbidden_zones = []
        self.no_rotate_zones = []

        for i, m in enumerate(self.maps):
            v = self.single_map_validator(i * 100000)
            try:
                v.validate(m, self.models.tmp['map_params_assoc'])
            except ValidationError as ex:
                params = {
                    'map': {
                        'name': str(m.map),
                        'id': '_map_%s' % m.map.pk,
                        'item': '',
                    }
                }
                params.update(ex.params)
                raise ValidationError('Map "{map}": %s' % ex, params=params)

            self.ocg.append(v.ocg)
            self.map_structure.append(v.map_structure)

            dups = self.station_names.intersection(v.station_names)
            if dups:
                raise ValidationError('{maps} error: Duplicate station names: "%s".' % '", "'.join(["%s" % s for s in dups]), params=MAPS_PARAM)
            self.station_names.update(v.station_names)

            self._merge_graph(v)

        self.station_names = sorted(list(self.station_names))

    def assemble(self):
        self.landmarks = []
        self.forbidden_zones = []
        super(MapXValidator, self).assemble()
        # from_downloadables
        if self.ocg and isinstance(self.ocg[0], dict):
            self.ocg = [self._convert_downloadables_to_ocg(o) for o in self.ocg]

    def _merge_graph(self, v):
        super(MapXValidator, self)._merge_graph(v)
        self.landmarks.append(v.landmarks)
        self.forbidden_zones.append(v.forbidden_zones)

    def _convert_downloadables_to_ocg(self, o):
        ocg = OccupancyGrid()
        ocg.header.stamp = rospy.Time.from_sec(time.time())
        ocg.header.frame_id = o['frame_id']
        ocg.info.resolution = o['resolution']
        ocg.info.width = o['width']
        ocg.info.height = o['height']
        ocg.info.origin.position.x = o['x0']
        ocg.info.origin.position.y = o['y0']

        png_data = base64.b64decode(o['png'])
        ocg.data = np.frombuffer(png_data, dtype=np.int8)
        return ocg

    def provide_reloadable(self, provide, cur, tmp):
        if provide == 'ocg':
            # from_downloadables
            if tmp and isinstance(tmp[0], dict):
                tmp = [self._convert_downloadables_to_ocg(o) for o in tmp]
            for c, t in zip(cur, tmp):
                if (c.header.frame_id != t.header.frame_id or
                        c.info.resolution != t.info.resolution or
                        c.info.width != t.info.width or
                        c.info.height != t.info.height or
                        c.info.origin.position.x != t.info.origin.position.x or
                        c.info.origin.position.y != t.info.origin.position.y or
                        not np.array_equal(c.data, t.data)):
                    return False
            return True
        return super(MapXValidator, self).provide_reloadable(provide, cur, tmp)


class SingleMapXValidator(SingleMapValidator):

    def get_map_param(self, item=''):
        return {
            'map': {
                'name': str(self.map.map),
                'id': '_map_%s' % self.map.map.pk,
                'item': item,
            }
        }

    def validate(self, map, map_params_assoc):
        self.map = map

        if not self.map.ocg:
            raise ValidationError('The active map does not have a raw map.')

        max_image_pixels = Image.MAX_IMAGE_PIXELS
        Image.MAX_IMAGE_PIXELS = None
        try:
            ocg_metadata = json.loads(self.map.ocg.metadata)
            ocg_png = Image.open(self.map.ocg.png_file)

            if (ocg_metadata['width'], ocg_metadata['height']) != ocg_png.size:
                raise ValidationError('Dimensions of the raw map are not specified correctly in its metadata.')

            if ocg_png.mode != 'P':
                raise ValidationError('Raw map must be in 8-bit indexed color format.')

            self.ocg = OccupancyGrid()
            self.ocg.header.stamp = rospy.Time.from_sec(time.time())
            self.ocg.header.frame_id = ocg_metadata['frame_id']
            self.ocg.info.resolution = ocg_metadata['resolution']
            self.ocg.info.width = ocg_metadata['width']
            self.ocg.info.height = ocg_metadata['height']
            self.ocg.info.origin.position.x = ocg_metadata['x0']
            self.ocg.info.origin.position.y = ocg_metadata['y0']

            self.map.ocg.png_file.seek(0)  # reset read pointer
            png_data = self.map.ocg.png_file.read()
            self.ocg.data = np.frombuffer(png_data, dtype=np.int8)
        except ValidationError:
            raise
        except Exception as ex:
            raise ValidationError('Raw map data might have corrupted. %s' % ex)
        finally:
            Image.MAX_IMAGE_PIXELS = max_image_pixels

        super(SingleMapXValidator, self).validate(map, map_params_assoc)

        try:
            self.landmarks = self.map_structure.get('landmarks', [])
            assert isinstance(self.landmarks, list)
            self._validate_landmark()
            self._construct_landmark_msg()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('Landmark data might have corrupted.')

        try:
            self.forbidden_zones = self.map_structure.get('forbidden_zones', [])
            assert isinstance(self.forbidden_zones, list)
            self._validate_forbidden_zone()
            self._construct_forbidden_zone_msg()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('Forbidden zone data might have corrupted.')

    def _validate_path(self):
        tracked_paths_assocs = defaultdict(list)
        path_count = len(self.paths)
        for pid, p in enumerate(self.paths):
            if (p['j1'] < 0 or p['j1'] >= self.junction_count or
                    p['j2'] < 0 or p['j2'] >= self.junction_count):
                raise ValidationError('Junction index out of bounds.')

            if p['j1'] == p['j2']:
                raise ValidationError('Path cannot start and end at the same junction.')

            if p['flow'] < 0 or p['flow'] >= PathFlow.MAX_N:
                raise ValidationError('Path has invalid flow.')

            if 'facing' not in p:
                raise ValidationError('Map layout is in an older format (pre-v3.0). Save the map layout again to migrate it to the new format.')

            if p['facing'] < 0 or p['facing'] >= PathFacing.MAX_N:
                raise ValidationError('Path has invalid facing.')

            if p['shape'] < 0 or p['shape'] >= PathShape.MAX_N:
                raise ValidationError('Path has invalid shape.')

            if p['shape'] == PathShape.BEZIER:
                try:
                    cp1x = float(p['cp1']['x'])
                    cp1y = float(p['cp1']['y'])
                    cp2x = float(p['cp2']['x'])
                    cp2y = float(p['cp2']['y'])
                except Exception:
                    raise ValidationError('Bezier path does not have valid control points.')

            if p['shape'] == PathShape.TELEPORT:
                raise ValidationError('In-map teleport path is not supported yet.')

            if 'speedLimit' in p:
                raise ValidationError('Map layout is in an older format (pre-v1.14). Save the map layout again to migrate it to the new format.')

            raw = p['speed']
            if isinstance(raw, six.string_types) and raw.startswith('${'):
                if raw not in self.map_params_assoc:
                    raise ValidationError('Path speed refers to inherited param "%s" that is unavailable.' % raw)

                if self.map_params_assoc[raw]['type'] != 'double':
                    raise ValidationError('Path speed refers to inherited param "%s" of incompatible type.' % raw)

            else:
                speed = float(raw)
                if speed <= 0:
                    raise ValidationError('Path has invalid speed.')

            if p['distance'] <= 0:
                raise ValidationError('Path has invalid distance.')

            if p['tracked']:
                bj1 = int(p.get('bj1', 0))
                bj2 = int(p.get('bj2', 0))
                if bj1 < 0 or bj1 >= path_count or bj2 < 0 or bj2 >= path_count:
                    raise ValidationError('Path branch index out of bounds.')

                bj1 = int(p.get('bj1', -1))
                bj2 = int(p.get('bj2', -1))
                if bj1 >= 0:
                    p2 = self.paths[bj1]
                    if int(p2.get('bj1', -1)) != pid and int(p2.get('bj2', -1)) != pid:
                        raise ValidationError('Branch data invalid.', params=self.get_map_param('_jid_%d' % p['j1']))
                if bj2 >= 0:
                    p2 = self.paths[bj2]
                    if int(p2.get('bj1', -1)) != pid and int(p2.get('bj2', -1)) != pid:
                        raise ValidationError('Branch data invalid.', params=self.get_map_param('_jid_%d' % p['j2']))

                tracked_paths_assocs[p['j1']].append(
                    (self._get_path_forward_heading(p), p, pid)
                )
                if p['flow'] == PathFlow.BI_DIRECTIONAL:
                    tracked_paths_assocs[p['j2']].append(
                        (self._get_path_reverse_heading(p), p, pid)
                    )

        # validate angles between all connected tracked paths.
        for jid, paths_list in tracked_paths_assocs.items():
            if len(paths_list) <= 1:
                continue

            paths_list.sort(key=lambda a: a[0])

            same_heading = False
            h1, p1, pid1 = paths_list[-1]
            for h2, p2, pid2 in paths_list:
                if self._is_heading_equal(h1, h2):
                    if p1['shape'] == PathShape.STRAIGHT and p1['shape'] == p2['shape']:
                        raise ValidationError('Adjacent tracked straight paths must be 90 degrees or 180 degrees apart from one another. [%s]' %
                        ', '.join(['%.1f deg' % d[0] for d in paths_list]), params=self.get_map_param('_jid_%d' % jid))
                    if same_heading and len(paths_list) > 2:
                        raise ValidationError('More than two tracked paths shared the same heading. [%s]' %
                        ', '.join(['%.1f deg' % d[0] for d in paths_list]), params=self.get_map_param('_jid_%d' % jid))
                    same_heading = True

                elif not (self._is_heading_equal(h1 + 90, h2) or
                        self._is_heading_equal(h1 + 180, h2) or
                        self._is_heading_equal(h1 + 270, h2)):
                    raise ValidationError('Adjacent tracked paths must be 90 degrees or 180 degrees apart from one another. [%s]' %
                        ', '.join(['%.1f deg' % d[0] for d in paths_list]), params=self.get_map_param('_jid_%d' % jid))
                else:
                    same_heading = False

                h1 = h2
                p1 = p2
                pid1 = pid2

            if same_heading and len(paths_list) > 2:
                h2, p2, pid2 = paths_list[0]
                if self._is_heading_equal(h1, h2):
                    raise ValidationError('More than two tracked paths shared the same heading. [%s]' %
                    ', '.join(['%.1f deg' % d[0] for d in paths_list]), params=self.get_map_param('_jid_%d' % jid))

    def _validate_station(self):
        station_names = set()
        for s in self.stations:
            if not s['name']:
                raise ValidationError('Station name is empty.')

            if s['name'] in station_names:
                raise ValidationError('Duplicate station name: "%s".' % s['name'])
            station_names.add(s['name'])

            if s['j'] < 0 or s['j'] >= self.junction_count:
                raise ValidationError('Junction index out of bounds.')

            if s['heading'] < Heading.MIN or s['heading'] > Heading.MAX:
                raise ValidationError('Station has invalid heading.')

        if station_names != set(self.station_names):
            raise ValidationError('The `stations` field is not denormalized properly.')

    def _validate_landmark(self):
        for l in self.landmarks:
            x = float(l['x'])
            y = float(l['y'])

    def _validate_forbidden_zone(self):
        for fz in self.forbidden_zones:
            for p in fz['points']:
                x = float(p[0])
                y = float(p[1])

    def assemble(self, map_structure):
        super(SingleMapXValidator, self).assemble(map_structure)
        self.landmarks = self.map_structure.get('landmarks', [])
        self._construct_landmark_msg()
        self.forbidden_zones = self.map_structure.get('forbidden_zones', [])
        self._construct_forbidden_zone_msg()

    def _construct_graph(self):
        self.stations_assoc = {
            s['name']: (self.id + s['j'], s['heading']) for s in self.stations
        }

        self.graph = nx.DiGraph()
        for idx, j in enumerate(self.junctions):
            x = float(j['x'])
            y = float(j['y'])
            kwargs = {
                'x': x,
                'y': y,
                'location_hint': self._get_location_hint(x, y),
            }
            self.graph.add_node(self.id + idx, **kwargs)

        for p in self.paths:
            j1 = self.id + p['j1']
            j2 = self.id + p['j2']
            flow = p['flow']
            facing = p['facing']
            fw_branch = PathBranch.NONE
            rv_branch = PathBranch.NONE
            if p['tracked']:
                fw_branch = self._get_forward_branch(p)
                rv_branch = self._get_reverse_branch(p)

            kwargs = {
                'cp1': p['cp1'],
                'cp2': p['cp2'],
                'facing': [0, 1, 2, 1, 2][facing],
                'shape': p['shape'],
                'dynamic': p.get('dynamic', False),
                'tracked': p['tracked'],
                'distance': p['distance'],
                'speed': p['speed'],
                'branch': fw_branch,
                'rv_branch': rv_branch,
            }
            self.graph.add_edge(j1, j2, **kwargs)

            if flow == PathFlow.BI_DIRECTIONAL:
                kwargs['cp1'] = p['cp2']
                kwargs['cp2'] = p['cp1']
                kwargs['facing'] = [0, 2, 1, 1, 2][facing]
                kwargs['branch'] = rv_branch
                kwargs['rv_branch'] = fw_branch
                self.graph.add_edge(j2, j1, **kwargs)

    def _construct_landmark_msg(self):
        msg = geometry_msgs.msg.PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'

        for l in self.landmarks:
            p = geometry_msgs.msg.Pose()
            p.position.x = float(l['x'])
            p.position.y = float(l['y'])
            p.orientation.w = 1.0
            msg.poses.append(p)

        self.landmarks = msg

    def _construct_forbidden_zone_msg(self):
        msg = agv05_msgs.msg.PolygonArray()
        for fz in self.forbidden_zones:
            polygon = geometry_msgs.msg.Polygon()
            polygon.points = [geometry_msgs.msg.Point32(float(p[0]), float(p[1]), 0) for p in fz['points']]
            msg.polygons.append(polygon)

        self.forbidden_zones = msg

    def _cross_product(self, a, b):
        return a[0] * b[1] - a[1] * b[0]

    def _get_forward_branch(self, path):
        if path.get('bj1') is None:
            return PathBranch.NONE

        branchP = self.paths[path['bj1']]
        branchJ = branchP['j1'] if branchP['j2'] == path['j1'] else branchP['j2']

        cJ = self.junctions[path['j1']]
        pJ = self.junctions[path['j2']]
        bJ = self.junctions[branchJ]

        return self._get_branch_direction(cJ, pJ, bJ)

    def _get_reverse_branch(self, path):
        if path.get('bj2') is None:
            return PathBranch.NONE

        branchP = self.paths[path['bj2']]
        branchJ = branchP['j1'] if branchP['j2'] == path['j2'] else branchP['j2']

        cJ = self.junctions[path['j2']]
        pJ = self.junctions[path['j1']]
        bJ = self.junctions[branchJ]

        return self._get_branch_direction(cJ, pJ, bJ)

    def _get_branch_direction(self, cJ, pJ, bJ):
        vP = [pJ['x'] - cJ['x'], pJ['y'] - cJ['y']]
        vB = [bJ['x'] - cJ['x'], bJ['y'] - cJ['y']]
        cPB = self._cross_product(vP, vB)
        if cPB > 0:
            return PathBranch.RIGHT
        elif cPB < 0:
            return PathBranch.LEFT
        return PathBranch.NONE

    def _get_line_heading(self, p1, p2):
        dx = p2['x'] - p1['x']
        dy = p2['y'] - p1['y']
        return math.degrees(math.atan2(dy, dx)) % 360

    def _get_path_forward_heading(self, p):
        j1 = self.junctions[p['j1']]
        p1 = {
            'x': float(j1['x']),
            'y': float(j1['y']),
        }
        if p['shape'] == PathShape.BEZIER:
            p2 = {
                'x': float(p['cp1']['x']),
                'y': float(p['cp1']['y']),
            }
            return self._get_line_heading(p1, p2)
        else:
            j2 = self.junctions[p['j2']]
            p2 = {
                'x': float(j2['x']),
                'y': float(j2['y']),
            }
            return self._get_line_heading(p1, p2)

    def _get_path_reverse_heading(self, p):
        j2 = self.junctions[p['j2']]
        p1 = {
            'x': float(j2['x']),
            'y': float(j2['y']),
        }
        if p['shape'] == PathShape.BEZIER:
            p2 = {
                'x': float(p['cp2']['x']),
                'y': float(p['cp2']['y']),
            }
            return self._get_line_heading(p1, p2)
        else:
            j1 = self.junctions[p['j1']]
            p2 = {
                'x': float(j1['x']),
                'y': float(j1['y']),
            }
            return self._get_line_heading(p1, p2)

    def _is_heading_equal(self, a, b):
        return (a - b) % 360 < 5.0 or (b - a) % 360 < 5.0


class ConstrainedMapXValidator(ConstrainedMapValidator):
    map_tracker_cls = MapTrackerX
