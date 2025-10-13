from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Direction, PathBranch, PathShape
from enumfields import IntEnum
import geometry_msgs.msg
import math
import networkx as nx
import rospy
import std_msgs.msg
import ujson as json

# Equivalent distance corresponding to the motions below.
T_DISTANCE = (0, 0.001, 1.25, 2, 3.5, 5, 1.25, 2, 3.5, 5)


class Motion(object):
    ZERO = 1
    L0 = 2
    L1 = 3
    L2 = 4
    L3 = 5
    R0 = 6
    R1 = 7
    R2 = 8
    R3 = 9


class MapTracker(object):
    map_layout_topic = '/map_layout'
    robot_motion_topic = '/robot_motion'
    robot_svg_topic = '/robot_svg'

    def __init__(self):
        self.graph = None
        self.map_structure = None
        self.location = None
        self.prev_location = None
        self.aware = False
        self.motion = None
        self.tracked_heading = False
        self.dimension = None
        self.dimension_profile = 0

        self.__map_layout_pub = rospy.Publisher(self.map_layout_topic, std_msgs.msg.String, queue_size=1, latch=True)
        self.__robot_motion_pub = rospy.Publisher(self.robot_motion_topic, std_msgs.msg.String, queue_size=1, latch=True)
        self.__robot_svg_pub = rospy.Publisher(self.robot_svg_topic, std_msgs.msg.String, queue_size=1, latch=True)

    def set_map(self, graph):
        if isinstance(graph, nx.DiGraph):
            self.graph = graph
            self.location = None
            self.prev_location = None
            self.aware = False
            self.motion = None
            self.tracked_heading = False
        else:
            rospy.logerr('Invalid map given to MapTracker.')

    def set_initial_location(self, location):
        if not self.graph:
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
                self._publish_map_layout()
                self._publish_motion()
        except Exception:
            rospy.logerr('Invalid location given to MapTracker.')

    def set_confused(self):
        self.aware = False
        self.tracked_heading = False

    def forward(self, hint=None):
        self.motion = 'Forward'
        if not self.graph or not self.location:
            rospy.logerr('MapTracker does not have a map or location yet.')
            return

        if not self._lookup_next_location(forward=True, hint=hint):
            rospy.logerr('MapTracker is confused by invalid forward path.')
            self.aware = False
        else:
            self._publish_motion()

    def branch_forward_right(self, hint=None):
        self.motion = 'Branch Forward Right'
        if not self.graph or not self.location:
            rospy.logerr('MapTracker does not have a map or location yet.')
            return

        if not self._lookup_next_location(forward=True, hint=hint, branch=PathBranch.RIGHT):
            rospy.logerr('MapTracker is confused by invalid branch forward right path.')
            self.aware = False
        else:
            self._publish_motion()

    def branch_forward_left(self, hint=None):
        self.motion = 'Branch Forward Left'
        if not self.graph or not self.location:
            rospy.logerr('MapTracker does not have a map or location yet.')
            return

        if not self._lookup_next_location(forward=True, hint=hint, branch=PathBranch.LEFT):
            rospy.logerr('MapTracker is confused by invalid branch forward left path.')
            self.aware = False
        else:
            self._publish_motion()

    def reverse(self, hint=None):
        self.motion = 'Reverse'
        if not self.graph or not self.location:
            rospy.logerr('MapTracker does not have a map or location yet.')
            return

        if not self._lookup_next_location(forward=False, hint=hint):
            rospy.logerr('MapTracker is confused by invalid reverse path.')
            self.aware = False
        else:
            self._publish_motion()

    def branch_reverse_right(self, hint=None):
        self.motion = 'Branch Reverse Right'
        if not self.graph or not self.location:
            rospy.logerr('MapTracker does not have a map or location yet.')
            return

        if not self._lookup_next_location(forward=False, hint=hint, branch=PathBranch.RIGHT):
            rospy.logerr('MapTracker is confused by invalid branch reverse right path.')
            self.aware = False
        else:
            self._publish_motion()

    def branch_reverse_left(self, hint=None):
        self.motion = 'Branch Reverse Left'
        if not self.graph or not self.location:
            rospy.logerr('MapTracker does not have a map or location yet.')
            return

        if not self._lookup_next_location(forward=False, hint=hint, branch=PathBranch.LEFT):
            rospy.logerr('MapTracker is confused by invalid branch reverse left path.')
            self.aware = False
        else:
            self._publish_motion()

    def _lookup_next_location(self, forward, hint=None, fine_hint=True, branch=PathBranch.NONE):
        m = self.location[0]
        if forward:
            eq_dir = self.location[1]
        else:
            eq_dir = (self.location[1] + 2) % 4

        for e in self.graph.out_edges_iter(m, data=True):
            try:
                n = e[1]
                if e[2]['direction'] == eq_dir and e[2]['branch'] == branch:
                    self.prev_location = self.location
                    self.location = (n, (self.location[1] + e[2]['transform']) % 4)
                    self.tracked_heading = True
                    return True
                elif e[2]['direction'] == eq_dir and (e[2]['branch'] == PathBranch.NONE or (branch == PathBranch.NONE and e[2]['branch'] == PathBranch.LEFT)):
                    self.prev_location = self.location
                    self.location = (n, (self.location[1] + e[2]['transform']) % 4)
                    self.tracked_heading = True
                    rospy.logwarn('MapTracker: Path traversed in incorrect branching direction.')
                    return True
            except Exception:
                pass

        eq_dir = (eq_dir + 2) % 4
        for e in self.graph.in_edges_iter(m, data=True):
            try:
                n = e[0]
                if (e[2]['direction'] + e[2]['transform']) % 4 == eq_dir and e[2]['rv_branch'] == branch:
                    self.prev_location = self.location
                    self.location = (n, (self.location[1] - e[2]['transform']) % 4)
                    self.tracked_heading = True
                    rospy.logwarn('MapTracker: Uni-directional path traversed in the illegal direction.')
                    return True
                elif (e[2]['direction'] + e[2]['transform']) % 4 == eq_dir and (e[2]['rv_branch'] == PathBranch.NONE or (branch == PathBranch.NONE and e[2]['rv_branch'] == PathBranch.LEFT)):
                    self.prev_location = self.location
                    self.location = (n, (self.location[1] - e[2]['transform']) % 4)
                    self.tracked_heading = True
                    rospy.logwarn('MapTracker: Uni-directional path traversed in the illegal direction with incorrect branching direction.')
                    return True
            except Exception:
                pass

        self.tracked_heading = False

    def peek_next_location(self, forward, hint=None):
        saved_location = self.location
        saved_prev_location = self.prev_location
        saved_tracked_heading = self.tracked_heading
        result = self._lookup_next_location(forward, hint, fine_hint=False)
        if result:
            result = self.location
        else:
            result = None
        self.location = saved_location
        self.prev_location = saved_prev_location
        self.tracked_heading = saved_tracked_heading
        return result

    def rotate_left(self):
        self.motion = 'Rotate Left'
        self._rotate(3)

    def rotate_right(self):
        self.motion = 'Rotate Right'
        self._rotate(1)

    def uturn_left(self):
        self.motion = 'Rotate Left'
        self._rotate(2)

    def uturn_right(self):
        self.motion = 'Rotate Right'
        self._rotate(2)

    def rotate3q_left(self):
        self.motion = 'Rotate Left'
        self._rotate(1)

    def rotate3q_right(self):
        self.motion = 'Rotate Right'
        self._rotate(3)

    def _rotate(self, transform):
        if not self.graph or not self.location:
            rospy.logerr('MapTracker does not have a map or location yet.')
        else:
            self.prev_location = self.location
            self.location = (self.location[0], (self.location[1] + transform) % 4)
            self.tracked_heading = True
            self._publish_motion()

    def stop(self):
        self.motion = None
        if not self.graph or not self.location:
            rospy.logerr('MapTracker does not have a map or location yet.')
        else:
            self.prev_location = self.location
            self._publish_motion()

    def get_location(self):
        return self.location

    def get_prev_location(self):
        return self.prev_location

    def is_aware(self):
        return self.aware

    def get_location_hint(self):
        if not self.graph or not self.location or not self.aware:
            return ''
        j = self.graph.node[self.location[0]]
        return j.get('location_hint', '')

    # to keep track if the last action is tracked
    def is_tracked_heading(self):
        return self.tracked_heading

    def get_motion(self):
        return self.motion

    def get_pose(self):
        if not self.graph or not self.location or not self.aware:
            return
        j = self.graph.node[self.location[0]]
        pose = geometry_msgs.msg.Pose2D()
        pose.x = j['x']
        pose.y = j['y']
        pose.theta = (self.location[1] - 1) * math.pi / 2
        return pose

    # dimension
    def set_dimension_profile(self, profile):
        if not self.dimension or profile < 0 or profile > 5:
            return
        self.dimension_profile = profile
        svg = {
            'body': self.dimension['body']['svgPath'],
            'payload': self.dimension['payloads'][profile - 1]['svgPath'] if profile else '',
            'robot_radius': self.rr[profile][0],
        }
        self.__robot_svg_pub.publish(std_msgs.msg.String(json.dumps(svg)))
        return True

    def clear_dimension_profile(self):
        if self.dimension_profile:
            self.set_dimension_profile(0)

    def get_dimension_profile(self):
        return self.dimension_profile

    def patch_models(self, models, exposed_methods=None):
        exposed_methods = exposed_methods or []
        exposed_methods += [
            'get_dimension_profile',
            'is_valid_heading',
            'is_teleport',
            'check_transition',
            'map_idx',
        ]
        for m in exposed_methods:
            setattr(models, m, getattr(self, m))

    def is_valid_heading(self, a):
        return a >= 0 and a < Direction.NA

    def is_teleport(self, e):
        return e['shape'] == PathShape.TELEPORT

    def check_transition(self, e1, e2, j=None, reverse=False, zero=False):
        """
        `j`: Junction id, if both e1 and e2 are headings
        `reverse`: The travelling direction
        `zero`: Whether to test for zero transition instead
        """
        if isinstance(e1, int):  # e1 is heading
            h1 = e1
            assert h1 != Direction.NA, 'Unknown initial direction.'
        else:
            j = e1[1]
            if self.is_teleport(e1[2]):  # teleport path
                teleport = self.teleports[e1[2]['teleport']]
                h1 = self.stations_assoc[teleport['end']][1]
                assert h1 != Direction.NA, 'Unknown teleport end direction.'
            else:
                h1 = (e1[2]['direction'] + e1[2]['transform'] + (
                    2 if (reverse, False, True)[e1[2]['facing']] else 0)) % 4

        final = False
        if isinstance(e2, int):  # e2 is heading
            h2 = e2
            if h2 == Direction.NA:  # no alignment needed
                return Motion.ZERO
            final = True
        else:
            j = e2[0]
            if self.is_teleport(e2[2]):  # teleport path
                teleport = self.teleports[e2[2]['teleport']]
                h2 = self.stations_assoc[teleport['start']][1]
                if h2 == Direction.NA:  # no alignment needed
                    return Motion.ZERO
                if not teleport['nonStopTransition']:
                    final = True
            else:
                h2 = (e2[2]['direction'] + (
                    2 if (reverse, False, True)[e2[2]['facing']] else 0)) % 4

        nr = self.geoms[self.geoms[self.dimension_profile]].nrn.get(j, 0) if j is not None else 0
        diff_dir = (h2 - h1) % 4
        if diff_dir == 0:
            return Motion.ZERO

        elif zero:
            return False

        elif diff_dir == 1:
            if (final and 'uturn_right' not in self.allowed_motions  # rotate at destination
                    or 'rotate_right' not in self.allowed_motions):
                return 0

            if nr and nr & 0b1 << h1:  # no R1
                if 'uturn_left' not in self.allowed_motions or nr & 0b111 << h2:  # no L3
                    return 0
                return Motion.L3

            return Motion.R1

        elif diff_dir == 2:
            if 'uturn_left' in self.allowed_motions:
                if nr and nr & 0b11 << h2:  # no L2
                    pass
                else:
                    return Motion.L2

            if 'uturn_right' in self.allowed_motions:
                if nr and nr & 0b11 << h1:  # no R2
                    pass
                else:
                    return Motion.R2

            return 0

        elif diff_dir == 3:
            if (final and 'uturn_left' not in self.allowed_motions  # rotate at destination
                    or 'rotate_left' not in self.allowed_motions):
                return 0

            if nr and nr & 0b1 << h2:  # no L1
                if 'uturn_right' not in self.allowed_motions or nr & 0b111 << h1:  # no R3
                    return 0
                return Motion.R3

            return Motion.L1

    def fetch_models(self, models):
        map_structure = models.map_structure
        if isinstance(map_structure, list) and all(map_structure):
            self.map_structure = map_structure
        else:
            rospy.logerr('MapTracker: Invalid map structure in models.')

        self.stations_assoc = models.stations_assoc
        self.teleports = models.teleports
        self.geoms = models.geoms
        self.rr = models.rr
        self.dimension = models.dimension
        self.allowed_motions = models.allowed_motions

    def _publish_map_layout(self):
        map_idx = self._get_map_idx()
        if self.map_structure and len(self.map_structure) > map_idx:
            self.__map_layout_pub.publish(std_msgs.msg.String(
                data=json.dumps(self.map_structure[map_idx])))

    def _publish_motion(self):
        self.__robot_motion_pub.publish(json.dumps({
            'location': self.location,
            'prev_location': self.prev_location,
            'motion': self.motion,
        }))

    def map_idx(self, j):
        return j // 100000

    def _get_map_idx(self):
        return self.map_idx(self.location[0])
