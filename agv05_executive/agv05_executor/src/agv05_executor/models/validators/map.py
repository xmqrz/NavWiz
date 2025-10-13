from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, Direction, ExecutorMode, \
    Map, MapChangeset, PathBranch, PathFacing, PathFlow, PathShape, Variable
from shapely.affinity import affine_transform
from shapely.strtree import STRtree
import math
import networkx as nx
import shapely.geometry
import six
import ujson as json

from .validator import ValidationError, Validator
from ..map_tracker import MapTracker, Motion, T_DISTANCE

MAPS_PARAM = {
    'maps': {
        'name': 'Maps',
        'id': 'map-list',
        'item': '',
    }
}
ACTIVE_MAP_PARAM = {
    'active_map': {
        'name': 'Active map',
        'id': 'map-active',
        'item': '',
    }
}


class MapValidator(Validator):
    cache = Cache
    provides = ['map_structure', 'station_names']
    extra_provides = ['stations_assoc', 'graph', 'rfid_assoc', 'rfid_suffix_assoc', 'no_rotate_zones']

    map_changeset_cls = MapChangeset
    single_map_validator = staticmethod(lambda x: SingleMapValidator(x))

    rfid_suffix_assoc = {
        'N': Direction.NORTH,
        'S': Direction.SOUTH,
        'E': Direction.EAST,
        'W': Direction.WEST,
    }

    def is_enabled(self, pre_validate=False):
        executor_mode = self.models.tmp['executor_mode'] if pre_validate else self.models.executor_mode
        return executor_mode == ExecutorMode.Standalone.value

    def validate(self):
        self._load_maps()

        self.map_structure = []
        self.station_names = set()
        self.stations_assoc = {}
        self.rfids = set()
        self.rfid_assoc = {}
        self.graph = nx.DiGraph()
        self.no_rotate_zones = []

        for i, m in enumerate(self.maps):
            v = self.single_map_validator(i * 100000)
            params = {
                'map': {
                    'name': str(m.map),
                    'id': '_map_%s' % m.map.pk,
                    'item': '',
                }
            }
            try:
                v.validate(m, self.models.tmp['map_params_assoc'])
            except ValidationError as ex:
                params.update(ex.params)
                raise ValidationError('Map "{map}": %s' % ex, params=params)

            self.map_structure.append(v.map_structure)

            dups = self.station_names.intersection(v.station_names)
            if dups:
                raise ValidationError('{maps} error: Duplicate station names: "%s".' % '", "'.join(["%s" % s for s in dups]), params=MAPS_PARAM)
            self.station_names.update(v.station_names)

            dups = self.rfids.intersection(v.rfid_assoc.keys())
            if dups:
                raise ValidationError('{maps} error: Duplicate RFIDs: "%s".' % '", "'.join(["%s" % s for s in dups]), params=MAPS_PARAM)
            self.rfids.update(v.rfid_assoc.keys())

            self._merge_graph(v)

        self.station_names = sorted(list(self.station_names))

    def _load_maps(self):
        try:
            active_map = Variable.objects.get(pk=Variable.ACTIVE_MAP).value.split(',')
            active_map = Map.objects.filter(pk__in=active_map)
        except Exception:
            raise ValidationError('{active_map} is not set correctly.', params=ACTIVE_MAP_PARAM)

        self.maps = [self.map_changeset_cls.objects.filter(map=m).first() for m in active_map]
        self.maps = [m for m in self.maps if m]
        if not self.maps:
            raise ValidationError('{maps} error: No map found.', params=MAPS_PARAM)

    def assemble(self):
        self.stations_assoc = {}
        self.rfid_assoc = {}
        self.graph = nx.DiGraph()
        self.no_rotate_zones = []

        for i, st in enumerate(self.map_structure):
            v = self.single_map_validator(i * 100000)
            v.assemble(st)
            self._merge_graph(v)

    def _merge_graph(self, v):
        self.stations_assoc.update(v.stations_assoc)
        if 'rfid_assoc' in self.extra_provides:
            self.rfid_assoc.update(v.rfid_assoc)
        self.graph.add_nodes_from(v.graph.nodes_iter(data=True))
        self.graph.add_edges_from(v.graph.edges_iter(data=True))
        self.no_rotate_zones.append(v.no_rotate_zones)


class SingleMapValidator(object):

    def __init__(self, id):
        self.id = id

    def validate(self, map, map_params_assoc):
        self.map = map
        self.map_params_assoc = map_params_assoc

        try:
            self.map_structure = json.loads(self.map.structure)
            self.map_structure['name'] = '%s' % self.map.map
            self.station_names = json.loads(self.map.stations)

            # map annotations
            try:
                self.map_annotations = json.loads(self.map.map.mapannotation.annotations)
            except Exception:
                self.map_annotations = {
                    'textAnnotations': [],
                    'polygonAnnotations': [],
                    'iconAnnotations': [],
                }
            self.map_structure['annotations'] = self.map_annotations

            self.junction_count = self.map_structure['junction_count']
            self.junctions = self.map_structure['junctions']
            self.paths = self.map_structure['paths']
            self.path_count = len(self.paths)
            self.stations = self.map_structure['stations']
            self.text_annotations = self.map_annotations['textAnnotations']
            self.polygon_annotations = self.map_annotations['polygonAnnotations']
            self.icon_annotations = self.map_annotations['iconAnnotations']

            assert isinstance(self.station_names, list)
            assert isinstance(self.junction_count, int)
            assert isinstance(self.junctions, list)
            assert isinstance(self.paths, list)
            assert isinstance(self.stations, list)
            assert isinstance(self.text_annotations, list)
            assert isinstance(self.polygon_annotations, list)
            assert isinstance(self.icon_annotations, list)
        except Exception:
            raise ValidationError('Map data might have corrupted.')

        try:
            self._validate_junction()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('Junction data might have corrupted.')

        try:
            self._validate_path()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('Path data might have corrupted.')

        try:
            self._validate_station()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('Station data might have corrupted.')

        try:
            self._validate_annotations()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('Annotations data might have corrupted.')

        try:
            self.loc_hint_zones = self.map_structure.get('loc_hint_zones', [])
            assert isinstance(self.loc_hint_zones, list)
            self._validate_loc_hint_zone()
            self._construct_loc_hint_zone_geometry()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('Location hint zone data might have corrupted.')

        try:
            self._construct_graph()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('Map structure data might have corrupted.')

        try:
            self.no_rotate_zones = self.map_structure.get('no_rotate_zones', [])
            assert isinstance(self.no_rotate_zones, list)
            self._validate_no_rotate_zone()
            self._construct_no_rotate_zone_geometry()
        except ValidationError:
            raise
        except Exception:
            raise ValidationError('No rotate zone data might have corrupted.')

    def _validate_junction(self):
        rfids = set()
        for j in self.junctions:
            x = float(j['x'])
            y = float(j['y'])

            if j.get('rfid'):
                if j['rfid'] in rfids:
                    raise ValidationError('Duplicate RFID: "%s".' % j['rfid'])
                rfids.add(j['rfid'])

        if self.junction_count != len(self.junctions):
            raise ValidationError('Junction count does not match.')

    def _validate_path(self):
        for p in self.paths:
            if (p['j1'] < 0 or p['j1'] >= self.junction_count or
                    p['j2'] < 0 or p['j2'] >= self.junction_count):
                raise ValidationError('Junction index out of bounds.')

            if (int(p.get('bj1', -1)) >= self.path_count or int(p.get('bj2', -1)) >= self.path_count):
                raise ValidationError('Branch refers to invalid path.')

            if p['j1'] == p['j2']:
                raise ValidationError('Path cannot start and end at the same junction.')

            if p['direction'] < 0 or p['direction'] >= Direction.MAX_N:
                raise ValidationError('Path has invalid direction.')

            if p['direction'] == Direction.NA and p['shape'] != PathShape.TELEPORT:
                raise ValidationError('Only path of teleport type can be directionless.')

            if p['flow'] < 0 or p['flow'] >= PathFlow.MAX_N:
                raise ValidationError('Path has invalid flow.')

            if 'facing' not in p:
                raise ValidationError('Map layout is in an older format (pre-v3.0). Save the map layout again to migrate it to the new format.')

            if p['facing'] < 0 or p['facing'] >= PathFacing.MAX_N:
                raise ValidationError('Path has invalid facing.')

            if p['shape'] < 0 or p['shape'] >= PathShape.MAX_N:
                raise ValidationError('Path has invalid shape.')

            if p['shape'] == PathShape.TELEPORT and p['direction'] != Direction.NA:
                raise ValidationError('Path of teleport type must be directionless.')

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

            if s['direction'] < 0 or s['direction'] >= Direction.MAX_N:
                raise ValidationError('Station has invalid direction.')

        if station_names != set(self.station_names):
            raise ValidationError('The `stations` field is not denormalized properly.')

    def _validate_annotations(self):
        for a in self.text_annotations:
            try:
                a['x'] = float(a['x'])
                a['y'] = float(a['y'])
            except Exception:
                raise ValidationError('Text annotation coordinate data is corrupted.')
            try:
                a['size'] = float(a['size'])
            except Exception:
                raise ValidationError('Text annotation font size data is invalid.')
            if not a['content'] or not isinstance(a['content'], six.string_types):
                raise ValidationError('Text annotation content is invalid.')

        for a in self.polygon_annotations:
            for p in a['points']:
                x = float(p[0])
                y = float(p[1])

        for a in self.icon_annotations:
            try:
                a['x'] = float(a['x'])
                a['y'] = float(a['y'])
            except Exception:
                raise ValidationError('Icon annotation coordinate data is corrupted.')
            if not a['type'] or not isinstance(a['type'], six.string_types):
                raise ValidationError('Icon annotation type is invalid.')

    def _validate_no_rotate_zone(self):
        for z in self.no_rotate_zones:
            for p in z['points']:
                x = float(p[0])
                y = float(p[1])

    def _validate_loc_hint_zone(self):
        for z in self.loc_hint_zones:
            if not z['name']:
                raise ValidationError('Location hint zone has missing name.')
            for p in z['points']:
                x = float(p[0])
                y = float(p[1])

    def assemble(self, map_structure):
        self.map_structure = map_structure
        self.junction_count = self.map_structure['junction_count']
        self.junctions = self.map_structure['junctions']
        self.paths = self.map_structure['paths']
        self.stations = self.map_structure['stations']
        self.loc_hint_zones = self.map_structure.get('loc_hint_zones', [])
        self._construct_loc_hint_zone_geometry()
        self._construct_graph()
        self.no_rotate_zones = self.map_structure.get('no_rotate_zones', [])
        self._construct_no_rotate_zone_geometry()

    def _construct_graph(self):
        self.stations_assoc = {
            s['name']: (self.id + s['j'], s['direction']) for s in self.stations
        }
        self.rfid_assoc = {
            j['rfid']: (self.id + idx, Direction.NA) for idx, j in enumerate(self.junctions) if j.get('rfid')
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
            direction = p['direction']
            flow = p['flow']
            facing = p['facing']
            shape = p['shape']
            transform = [0, 0, 3, 1, 0][shape]
            fw_branch = self._get_forward_branch(p)
            rv_branch = self._get_reverse_branch(p, transform)

            kwargs = {
                'direction': direction,
                'transform': transform,
                'facing': [0, 1, 2, 1, 2][facing],
                'shape': shape,
                'distance': p['distance'],
                'speed': p['speed'],
                'branch': fw_branch,
                'rv_branch': rv_branch,
            }
            self.graph.add_edge(j1, j2, **kwargs)

            if flow == PathFlow.BI_DIRECTIONAL:
                kwargs['direction'] = (direction + transform + 2) % 4
                kwargs['transform'] = (4 - transform) % 4
                kwargs['facing'] = [0, 2, 1, 1, 2][facing]
                kwargs['shape'] = [0, 1, 3, 2, 4][shape]
                kwargs['branch'] = rv_branch
                kwargs['rv_branch'] = fw_branch
                self.graph.add_edge(j2, j1, **kwargs)

    def _get_location_hint(self, x, y):
        point = shapely.geometry.Point(x, y)
        for lhz in self.loc_hint_zones:
            if lhz['polygon'].contains(point):
                return lhz['name']
        return ''

    def _construct_no_rotate_zone_geometry(self):
        polygons = []
        for z in self.no_rotate_zones:
            polygons.append(shapely.geometry.Polygon(
                [(float(p[0]), float(p[1])) for p in z['points']]
            ))
        self.no_rotate_zones = STRtree(polygons) if polygons else None

    def _construct_loc_hint_zone_geometry(self):
        lhz = []
        for z in self.loc_hint_zones:
            p = shapely.geometry.Polygon(
                [(float(p[0]), float(p[1])) for p in z['points']]
            )
            lhz.append({
                'name': z['name'],
                'polygon': p,
            })
        self.loc_hint_zones = lhz

    def _get_forward_branch(self, path):
        bj1 = int(path.get('bj1', -1))
        if (bj1 < 0):
            return PathBranch.NONE
        branchP = self.paths[bj1]

        branchJ = branchP['j1'] if branchP['j2'] == path['j1'] else branchP['j2']
        pJ = self.junctions[path['j2']]
        bJ = self.junctions[branchJ]
        cJ = self.junctions[path['j1']]
        direction = path['direction']

        return self._get_branch(path, pJ, branchP, bJ, cJ, direction)

    def _get_reverse_branch(self, path, transform):
        bj2 = int(path.get('bj2', -1))
        if (bj2 < 0):
            return PathBranch.NONE

        branchP = self.paths[bj2]

        branchJ = branchP['j1'] if branchP['j2'] == path['j2'] else branchP['j2']
        pJ = self.junctions[path['j1']]
        bJ = self.junctions[branchJ]
        cJ = self.junctions[path['j2']]
        direction = (path['direction'] + transform + 2) % 4

        return self._get_branch(path, pJ, branchP, bJ, cJ, direction)

    def _get_branch(self, path, pathJ, branchP, branchJ, commonJ, direction):

        if direction == Direction.NORTH:  # negative y (x right)
            if (pathJ['x'] > commonJ['x']) != (branchJ['x'] > commonJ['x']) or path['shape'] == PathShape.STRAIGHT or branchP['shape'] == PathShape.STRAIGHT or pathJ['y'] == branchJ['y']:
                isRight = pathJ['x'] > branchJ['x']
            else:
                # if same side
                isRight = pathJ['y'] > branchJ['y']
                isRight = isRight if pathJ['x'] > commonJ['x'] else not isRight

        elif direction == Direction.SOUTH:  # positive y (-x right)
            if (pathJ['x'] > commonJ['x']) != (branchJ['x'] > commonJ['x']) or path['shape'] == PathShape.STRAIGHT or branchP['shape'] == PathShape.STRAIGHT or pathJ['y'] == branchJ['y']:
                isRight = pathJ['x'] < branchJ['x']
            else:
                # if same side
                isRight = pathJ['y'] < branchJ['y']
                isRight = isRight if pathJ['x'] < commonJ['x'] else not isRight

        elif direction == Direction.EAST:  # positive x (y right)
            if (pathJ['y'] > commonJ['y']) != (branchJ['y'] > commonJ['y']) or path['shape'] == PathShape.STRAIGHT or branchP['shape'] == PathShape.STRAIGHT or pathJ['x'] == branchJ['x']:
                isRight = pathJ['y'] > branchJ['y']
            else:
                # if same side
                isRight = pathJ['x'] < branchJ['x']
                isRight = isRight if pathJ['y'] > commonJ['y'] else not isRight

        elif direction == Direction.WEST:  # negative x (-y right)
            if (pathJ['y'] > commonJ['y']) != (branchJ['y'] > commonJ['y']) or path['shape'] == PathShape.STRAIGHT or branchP['shape'] == PathShape.STRAIGHT or pathJ['x'] == branchJ['x']:
                isRight = pathJ['y'] < branchJ['y']
            else:
                # if same side
                isRight = pathJ['x'] > branchJ['x']
                isRight = isRight if pathJ['y'] < commonJ['y'] else not isRight

        return PathBranch.RIGHT if isRight else PathBranch.LEFT


class ConstrainedMapValidator(Validator):
    cache = Cache
    provides = []
    extra_provides = ['flgraphs', 'rlgraphs', 'geoms']

    map_tracker_cls = MapTracker

    def is_enabled(self, pre_validate=False):
        executor_mode = self.models.tmp['executor_mode'] if pre_validate else self.models.executor_mode
        return executor_mode == ExecutorMode.Standalone.value

    def validate(self):
        self.graph = self.models.tmp['graph']
        self.stations_assoc = self.models.tmp['stations_assoc']
        self.teleports = self.models.tmp['teleports']
        self.no_rotate_zones = self.models.tmp['no_rotate_zones']
        self.geoms = self.models.tmp['geoms']
        self.allowed_motions = self.models.tmp['allowed_motions']
        try:
            self._construct_map_tracker()
            self._construct_no_rotate_nodes()
            self._construct_line_graphs()
        except Exception:
            raise ValidationError('Constrained graph construct error.')

    def assemble(self):
        self.graph = self.models.graph
        self.stations_assoc = self.models.stations_assoc
        self.teleports = self.models.teleports
        self.no_rotate_zones = self.models.no_rotate_zones
        self.geoms = self.models.geoms
        self.allowed_motions = self.models.allowed_motions
        self._construct_map_tracker()
        self._construct_no_rotate_nodes()
        self._construct_line_graphs()

    def _construct_map_tracker(self):
        self.mt = self.map_tracker_cls()
        self.mt.set_map(self.graph)
        self.mt.stations_assoc = self.stations_assoc
        self.mt.teleports = self.teleports
        self.mt.geoms = self.geoms
        self.mt.allowed_motions = self.allowed_motions

    def _construct_no_rotate_nodes(self):
        sin_cos_th = [(math.sin(th), math.cos(th)) for th in (math.radians(h) for h in range(0, 360, 10))]
        geoms = []
        geoms2 = []
        max_r = 0
        for q, g in self.geoms.items():
            if not isinstance(q, tuple):
                continue
            if not isinstance(q[0], tuple):
                geoms.append((q, g))
                max_r = max(max_r, g.r)
            else:
                geoms2.append((q, g))
            g.nrn = {}

        for n, pt in self.graph.nodes_iter(data=True):
            nrz = self.no_rotate_zones[self.mt.map_idx(n)]
            if not nrz:
                continue

            x = pt['x']
            y = pt['y']
            if not nrz.query(shapely.geometry.LineString([(x - max_r, y - max_r), (x + max_r, y + max_r)])):
                continue

            nrq = {}
            for q, g in geoms:
                guesses = nrz.query(shapely.geometry.LineString([(x - g.r, y - g.r), (x + g.r, y + g.r)]))
                if not guesses:
                    continue

                gts = (affine_transform(g, [c, -s, s, c, x, y]) for s, c in sin_cos_th)
                nr = tuple(any(gt.intersects(gg) for gg in guesses) for gt in gts)
                nr = sum(1 << i for i, b in enumerate(nr) if b)
                if not nr:
                    continue

                if not self.robot.trackless:
                    nr = (nr & ((1 << 36) - (1 << 27) | 1) != 0,
                        nr & (1 << 10) - (1 << 0) != 0,
                        nr & (1 << 19) - (1 << 9) != 0,
                        nr & (1 << 28) - (1 << 18) != 0)  # NESW
                    nr = sum(1 << i for i, b in enumerate(nr) if b)
                    nr = nr | (nr & 3) << 4
                if hasattr(g, 'a'):
                    g.nrn[n] = nr
                nrq[q] = nr

            for q, g in geoms2:
                nr1 = nrq.get(q[0], 0)
                nr2 = nrq.get(q[1], 0)

                # merge the nr's
                nr = nr1 | nr2
                if nr:
                    g.nrn[n] = nr

    def _construct_line_graphs(self):
        self.flgraphs = {}
        self.rlgraphs = {}
        for p, q in self.geoms.items():
            if isinstance(p, int) and q not in self.flgraphs:
                self._construct_line_graph(p, q)
                if self.models.short:  # short the validator
                    return

    def _construct_line_graph(self, p, q):
        self.mt.dimension_profile = p

        # check ability to perform straight movement
        restricted = [True, 'forward' not in self.allowed_motions, 'reverse' not in self.allowed_motions]
        if all(restricted):
            self.flgraphs[q] = nx.DiGraph()
            self.rlgraphs[q] = nx.DiGraph()
            return

        # forward
        restricted[0] = restricted[1]
        self.flgraphs[q] = nx.DiGraph([
            e for e in nx.line_graph(self.graph).edges_iter(data=True) if
            self._check_transition_and_set_distance(e, False, restricted)])

        # reverse
        restricted[0] = restricted[2]
        self.rlgraphs[q] = nx.DiGraph([
            e for e in nx.line_graph(self.graph).edges_iter(data=True) if
            self._check_transition_and_set_distance(e, True, restricted)])

    def _check_transition_and_set_distance(self, e, reverse, restricted):
        (j11, j12), (j21, j22), d = e
        p1 = self.graph[j11][j12]
        p2 = self.graph[j21][j22]
        assert j11 != j12 and j21 != j22 and j12 == j21

        if not self.mt.is_teleport(p1) and restricted[p1['facing']]:
            return False
        if not self.mt.is_teleport(p2) and restricted[p2['facing']]:
            return False

        d['transition'] = transition = self.mt.check_transition((j11, j12, p1), (j21, j22, p2), reverse=reverse)
        if not transition:
            return False

        d['distance'] = p1['distance'] + T_DISTANCE[transition]
        if (transition == Motion.ZERO and
                not self.mt.is_teleport(p1) and not self.mt.is_teleport(p2) and
                (reverse, False, True)[p1['facing']] != (reverse, False, True)[p2['facing']]):
            d['distance'] += 0.5
        return transition
