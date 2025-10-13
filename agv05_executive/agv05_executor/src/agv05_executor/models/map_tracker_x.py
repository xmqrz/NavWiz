from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import PathBranch
from agv05_webserver.systemx.models import Heading, OccupancyGrid, PathShape
import agv05_msgs.msg
import geometry_msgs.msg
import math
import rospy

from .map_tracker import MapTracker, Motion


class MapTrackerX(MapTracker):
    map_topic = '/map'
    landmarks_topic = '/map_landmarks'
    forbidden_zones_topic = '/map/forbidden_zone'

    def __init__(self):
        super(MapTrackerX, self).__init__()
        self.ocg = None
        self.landmarks = None
        self.forbidden_zones = None
        self.__map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1, latch=True)
        self.__landmarks_pub = rospy.Publisher(self.landmarks_topic, geometry_msgs.msg.PoseArray, queue_size=1, latch=True)
        self.__forbidden_zones_pub = rospy.Publisher(self.forbidden_zones_topic, agv05_msgs.msg.PolygonArray, queue_size=1, latch=True)

    def set_initial_location(self, location):
        if not self.graph or not self.ocg:
            rospy.logerr('MapTracker does not have a map yet.')
            return

        try:
            if location[0] not in self.graph or not self.is_valid_heading(location[1]):
                rospy.logerr('Invalid location given to MapTracker.')
            else:
                self.location = location
                self.prev_location = location
                self.aware = True
                self.motion = None
                self.tracked_heading = True
                self._publish_landmarks()
                self._publish_forbidden_zones()
                self._publish_map()
                self._publish_map_layout()
                self._publish_motion()
        except Exception:
            rospy.logerr('Invalid location given to MapTracker.')

    def _lookup_next_location(self, forward, hint=None, fine_hint=True, branch=PathBranch.NONE):
        m = self.location[0]
        if forward:
            heading = self.location[1]
        else:
            heading = self.invert_heading(self.location[1])

        for e in self.graph.out_edges_iter(m, data=True):
            try:
                n = e[1]
                start_heading = self.compute_edge_start_heading(e)
                if self.is_heading_equal(start_heading, heading):
                    end_heading = self.compute_edge_end_heading(e)
                    if not forward:
                        end_heading = self.invert_heading(end_heading)

                    if (not hint and e[2]['tracked'] and e[2]['branch'] == branch or
                            hint and self.is_position_equal(hint, self.graph.node[n], fine_hint)):
                        self.prev_location = self.location
                        self.location = (n, end_heading)
                        self.tracked_heading = e[2]['tracked']
                        return True
                    elif (not hint and e[2]['tracked'] and (e[2]['branch'] == PathBranch.NONE or (branch == PathBranch.NONE and e[2]['branch'] == PathBranch.LEFT))):
                        self.prev_location = self.location
                        self.location = (n, end_heading)
                        self.tracked_heading = e[2]['tracked']
                        rospy.logwarn('MapTracker: Path traversed in incorrect branching direction.')
                        return True
            except Exception:
                pass

        heading = self.invert_heading(heading)
        for e in self.graph.in_edges_iter(m, data=True):
            try:
                n = e[0]
                end_heading = self.compute_edge_end_heading(e)
                if self.is_heading_equal(end_heading, heading):
                    start_heading = self.compute_edge_start_heading(e)
                    if forward:
                        start_heading = self.invert_heading(start_heading)

                    if (not hint and e[2]['tracked'] and e[2]['rv_branch'] == branch or
                            hint and self.is_position_equal(hint, self.graph.node[n], fine_hint)):
                        self.prev_location = self.location
                        self.location = (n, start_heading)
                        self.tracked_heading = e[2]['tracked']
                        rospy.logwarn('MapTracker: Uni-directional path traversed in the illegal direction.')
                        return True
                    elif (not hint and e[2]['tracked'] and (e[2]['rv_branch'] == PathBranch.NONE or (branch == PathBranch.NONE and e[2]['rv_branch'] == PathBranch.LEFT))):
                        self.prev_location = self.location
                        self.location = (n, start_heading)
                        self.tracked_heading = e[2]['tracked']
                        rospy.logwarn('MapTracker: Uni-directional path traversed in the illegal direction with incorrect branching direction.')
                        return True
            except Exception:
                pass

        self.tracked_heading = False

    def rotate_left_to_heading(self, heading):
        self.motion = 'Rotate Left'
        self._rotate_to_heading(heading)

    def rotate_right_to_heading(self, heading):
        self.motion = 'Rotate Right'
        self._rotate_to_heading(heading)

    def _rotate_to_heading(self, heading):
        if not self.graph or not self.location:
            rospy.logerr('MapTracker does not have a map or location yet.')
        else:
            self.prev_location = self.location
            self.location = (self.location[0], heading % 360)
            self.tracked_heading = False
            self._publish_motion()

    def rotate_left(self):
        self.motion = 'Rotate Left'
        self._rotate(90)

    def rotate_right(self):
        self.motion = 'Rotate Right'
        self._rotate(270)

    def uturn_left(self):
        self.motion = 'Rotate Left'
        self._rotate(180)

    def uturn_right(self):
        self.motion = 'Rotate Right'
        self._rotate(180)

    def rotate3q_left(self):
        self.motion = 'Rotate Left'
        self._rotate(270)

    def rotate3q_right(self):
        self.motion = 'Rotate Right'
        self._rotate(90)

    def _rotate(self, transform):
        if not self.graph or not self.location:
            rospy.logerr('MapTracker does not have a map or location yet.')
        else:
            self.prev_location = self.location
            self.location = (self.location[0], (self.location[1] + transform) % 360)
            self.tracked_heading = True
            self._publish_motion()

    def get_pose(self):
        if not self.graph or not self.location or not self.aware:
            return
        j = self.graph.node[self.location[0]]
        pose = geometry_msgs.msg.Pose2D()
        pose.x = j['x']
        pose.y = j['y']
        pose.theta = math.radians(self.location[1])
        return pose

    def patch_models(self, models, exposed_methods=None):
        exposed_methods = exposed_methods or []
        exposed_methods += [
            'is_heading_equal',
            'invert_heading',
            'compute_edge_start_heading',
            'compute_edge_end_heading'
        ]
        super(MapTrackerX, self).patch_models(models, exposed_methods)

    def is_position_equal(self, p1, p2, fine):
        dx = p2['x'] - p1['x']
        dy = p2['y'] - p1['y']
        eps = 1e-6 if fine else 0.1
        return abs(dx) < eps and abs(dy) < eps

    def is_valid_heading(self, a):
        return a >= Heading.MIN and a < Heading.NA

    def is_heading_equal(self, a, b):
        return (a - b) % 360 < 5.0 or (b - a) % 360 < 5.0

    def invert_heading(self, a):
        return (a + 180) % 360

    def compute_edge_start_heading(self, e):
        j1 = self.graph.node[e[0]]
        j2 = self.graph.node[e[1]]
        p = e[2]

        if p['shape'] == PathShape.STRAIGHT:
            return self._get_line_heading(j1, j2)

        elif p['shape'] == PathShape.BEZIER:
            return self._get_line_heading(j1, p['cp1'])

    def compute_edge_end_heading(self, e):
        j1 = self.graph.node[e[0]]
        j2 = self.graph.node[e[1]]
        p = e[2]

        if p['shape'] == PathShape.STRAIGHT:
            return self._get_line_heading(j1, j2)

        elif p['shape'] == PathShape.BEZIER:
            return self._get_line_heading(p['cp2'], j2)

    def _get_line_heading(self, p1, p2):
        dx = p2['x'] - p1['x']
        dy = p2['y'] - p1['y']
        return math.degrees(math.atan2(dy, dx)) % 360

    def is_teleport(self, e):
        return e['shape'] == PathShape.TELEPORT

    def check_transition(self, e1, e2, j=None, reverse=False, zero=False):
        """
        `j`: Junction id, if both e1 and e2 are headings
        `reverse`: The travelling direction
        `zero`: Whether to test for zero transition instead
        """
        if isinstance(e1, int) or isinstance(e1, float):  # e1 is heading
            h1 = e1
            assert h1 != Heading.NA, 'Unknown initial heading.'
        else:
            j = e1[1]
            if self.is_teleport(e1[2]):  # teleport path
                teleport = self.teleports[e1[2]['teleport']]
                h1 = self.stations_assoc[teleport['end']][1]
                assert h1 != Heading.NA, 'Unknown teleport end heading.'
            else:
                h1 = (self.compute_edge_end_heading(e1) + (
                    180 if (reverse, False, True)[e1[2]['facing']] else 0)) % 360

        final = False
        if isinstance(e2, int) or isinstance(e2, float):  # e2 is heading
            h2 = e2
            if h2 == Heading.NA:  # no alignment needed
                return Motion.ZERO
            final = True
        else:
            j = e2[0]
            if self.is_teleport(e2[2]):  # teleport path
                teleport = self.teleports[e2[2]['teleport']]
                h2 = self.stations_assoc[teleport['start']][1]
                if h2 == Heading.NA:  # no alignment needed
                    return Motion.ZERO
                if not teleport['nonStopTransition']:
                    final = True
            else:
                h2 = (self.compute_edge_start_heading(e2) + (
                    180 if (reverse, False, True)[e2[2]['facing']] else 0)) % 360

        nr = self.geoms[self.geoms[self.dimension_profile]].nrn.get(j, 0) if j is not None else 0

        diff_dir = (h2 - h1) % 360
        if self.is_heading_equal(diff_dir, 0):
            return Motion.ZERO

        elif zero:
            return False

        elif diff_dir <= 175:
            if final or diff_dir > 95:
                if 'uturn_left' not in self.allowed_motions:  # rotate at destination
                    return 0

            if final or diff_dir <= 95:
                if 'rotate_left' not in self.allowed_motions:
                    return 0

            if nr and nr & self.__nr_mask(h1, diff_dir):  # no left rotate
                if 'uturn_right' not in self.allowed_motions or nr & self.__nr_mask(h2, -diff_dir):  # no right rotate
                    return 0
                return Motion.R3

            return Motion.L0 if diff_dir < 45 else Motion.L1 if diff_dir <= 135 else Motion.L2

        elif diff_dir >= 185:
            if final or diff_dir < 265:
                if 'uturn_right' not in self.allowed_motions:  # rotate at destination
                    return 0

            if final or diff_dir >= 265:
                if 'rotate_right' not in self.allowed_motions:
                    return 0

            if nr and nr & self.__nr_mask(h2, -diff_dir):  # no right rotate
                if 'uturn_left' not in self.allowed_motions or nr & self.__nr_mask(h1, diff_dir):  # no left rotate
                    return 0
                return Motion.L3

            return Motion.R0 if diff_dir > 315 else Motion.R1 if diff_dir >= 225 else Motion.R2

        else:
            if 'uturn_left' in self.allowed_motions:
                if nr and nr & self.__nr_mask(h1, diff_dir):  # no L2
                    pass
                else:
                    return Motion.L2

            if 'uturn_right' in self.allowed_motions:
                if nr and nr & self.__nr_mask(h2, -diff_dir):  # no R2
                    pass
                else:
                    return Motion.R2

            return 0

    def __nr_mask(self, h, d):
        d = d % 360
        start = int(math.ceil(h / 10.0))
        end = int(math.floor((h + d) / 10.0)) + 1
        mask = (1 << end) - (1 << start)
        return mask & (1 << 36) - 1 | mask >> 36

    def fetch_models(self, models):
        super(MapTrackerX, self).fetch_models(models)
        ocg = models.ocg
        if isinstance(ocg, list) and all([isinstance(o, OccupancyGrid) for o in ocg]):
            self.ocg = ocg
        else:
            rospy.logerr('Invalid raw map given to MapTracker.')

        landmarks = models.landmarks
        if isinstance(landmarks, list) and all([isinstance(l, geometry_msgs.msg.PoseArray) for l in landmarks]):
            self.landmarks = landmarks
        else:
            rospy.logerr('Invalid landmarks given to MapTracker.')

        forbidden_zones = models.forbidden_zones
        if isinstance(forbidden_zones, list) and all([isinstance(fz, agv05_msgs.msg.PolygonArray) for fz in forbidden_zones]):
            self.forbidden_zones = forbidden_zones
        else:
            rospy.logerr('Invalid forbidden zones given to MapTracker.')

    def _publish_map(self):
        map_idx = self._get_map_idx()
        if self.ocg and len(self.ocg) > map_idx:
            self.__map_pub.publish(self.ocg[map_idx])

    def _publish_landmarks(self):
        map_idx = self._get_map_idx()
        if self.landmarks and len(self.landmarks) > map_idx:
            self.__landmarks_pub.publish(self.landmarks[map_idx])

    def _publish_forbidden_zones(self):
        map_idx = self._get_map_idx()
        if self.forbidden_zones and len(self.forbidden_zones) > map_idx:
            self.__forbidden_zones_pub.publish(self.forbidden_zones[map_idx])
