from __future__ import absolute_import
from __future__ import unicode_literals

from io import BytesIO
from rospy.numpy_msg import numpy_msg
from wraptor.context import throttle as throttler
import base64
import distutils.util
import geometry_msgs.msg
import math
import os
import rospy
import sensor_msgs.msg
import std_msgs.msg
import tf
import threading
import time
import ujson as json

from ...models.validators.agv_x import StandaloneXValidator as ExecutorStandaloneXValidator
from ...models.validators.map_param import MapParamValidator as ExecutorMapParamValidator
from ...models.validators.map_x import ConstrainedMapXValidator as ExecutorConstrainedMapXValidator, \
    MapXValidator as ExecutorMapXValidator
from ...models.validators.validator import ValidationError
from ...state_machine import StateConstructError, state_factory
from .live_robot import LiveRobot
from agv05_webserver.system.models import ExecutorMode, Map
from agv05_webserver.systemx.models import MapOcg, MapChangeset, OccupancyGrid


HEARTBEAT_TIMEOUT = 3


class DisableValidatorCacheMixin(object):
    cache = True

    def _set_cache(self, *args):
        pass


class StandaloneXValidator(DisableValidatorCacheMixin, ExecutorStandaloneXValidator):
    def _validate_home(self):
        # Disable home validation
        self.agv_home = 'no-home'


class MapXValidator(DisableValidatorCacheMixin, ExecutorMapXValidator):
    def _load_maps(self):
        # Hack to disable load map from database.
        pass

    def assign_maps(self, maps):
        self.maps = maps


class MapParamValidator(DisableValidatorCacheMixin, ExecutorMapParamValidator):
    pass


class ConstrainedMapXValidator(DisableValidatorCacheMixin, ExecutorConstrainedMapXValidator):
    pass


class TracklessAgv(LiveRobot):
    id = 'trackless-agv'
    png_scheme = 'data:image/png;base64,'

    nav_skill_id = 'agv05_skills.plan_x.NavigateToX'
    reverse_nav_skill_id = 'agv05_skills.plan_x.ReverseNavigateToX'
    nav_skill_param_defaults = {
        'align_station_type': -1,
        'next_motion': 0,
        'next_speed': 0.0,
        'sense_line': 0,
    }

    map_topic = '/map'

    def __init__(self, *args, **kwargs):
        super(TracklessAgv, self).__init__(*args, **kwargs)
        self.running = False
        self.paused = False
        self.tf_listener = tf.TransformListener()
        self.tf_frames = ['laser']  # for scan
        self.tf_frames2 = []  # for reflector

        try:
            self.show_lidar_intensities = distutils.util.strtobool(os.getenv('SHOW_LIDAR_INTENSITIES'))
        except Exception:
            self.show_lidar_intensities = False

        self.__map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1, latch=True)

        self.robot_pose = None
        self.robot_svg = None

        self._lock = threading.Lock()
        self._execute_thread = None
        self._executing_uuid = None
        self._executed_uuid = None
        self._deadman_thread = None
        self._deadman_time = time.time()
        self._deadman_ev = threading.Event()
        self._executing = False
        self._sm = None

        # force robot in standalone mode
        self.robot.models.executor_mode = ExecutorMode.Standalone.value
        self.robot.models.get_agv_home_location = lambda: [9999999, 0]
        self.robot.fms_manager = None

        # clear transition trigger
        self.robot.models.transition_triggers = dict()

        self.standalone_validator = StandaloneXValidator(self.robot.models)
        self.map_param_validator = MapParamValidator(self.robot.models)
        self.map_validator = MapXValidator(self.robot.models)

        geoms = self.robot.models.geoms  # prevent geoms from being overwritten
        self.constrained_map_validator = ConstrainedMapXValidator(self.robot.models)
        self.robot.models.geoms = geoms

    def start(self):
        self.running = True
        self.__pose_timer = rospy.Timer(rospy.Duration(0.5), self.handle_pose)
        self.laser_pose = {
            'laser': {'x': 0, 'y': 0, 'theta': 0, 'flip': False},
        }

        self.__laser_subs = [rospy.Subscriber(topic, numpy_msg(sensor_msgs.msg.LaserScan), self.handle_laser_scan, queue_size=1) for
            topic in rospy.get_param('nav2d_scan_topics', 'scan').split()]
        self.__reflectors_sub = rospy.Subscriber('/reflectors', geometry_msgs.msg.PoseArray, self.handle_reflectors, queue_size=1)
        self.__robot_svg_sub = rospy.Subscriber('/robot_svg', std_msgs.msg.String, self.handle_robot_svg, queue_size=1)

    def stop(self):
        self.running = False
        self.__pose_timer.shutdown()
        self.out({
            'id': 'stop',
        })

        for sub in self.__laser_subs:
            sub.unregister()
        self.__reflectors_sub.unregister()
        self.__robot_svg_sub.unregister()

        if self._execute_thread:
            t = self._execute_thread
            t.join(1)
            while t.is_alive():
                self.robot.base.stop()
                t.join(1)
            self._execute_thread = None
        self._executing_uuid = None
        self._deadman_ev.set()
        if self._deadman_thread:
            self._deadman_thread.join()
            self._deadman_thread = None

    def handle_in_pipe(self, data):
        try:
            if data['command'] == 'set_station':
                self.set_station(data)
            elif data['command'] == 'set_pose':
                self.set_pose(data)
            elif data['command'] == 'nav_to':
                self.set_nav_to(data)
            elif data['command'] == 'abort':
                # only nav to abortable.
                self.abort_nav_to()
            elif data['command'] == 'heartbeat':
                if data['uuid'] == self._executing_uuid:
                    self._reset_deadman_time()
                elif data['uuid'] == self._executed_uuid:
                    self._send_action_completed(data['uuid'])
            elif data['command'] == 'retrieve_all':
                self._retrieve_all()
        except Exception as ex:
            rospy.logerr('[LiveRobot] uncaught exception in TracklessAgv in pipe: %s', ex)

    def handle_pose(self, timer_event):
        self._send_robot_status()
        self.handle_robot_pose()
        self.handle_laser_pose()
        self.tf_frames = []  # for scan
        self.tf_frames2 = []  # for reflector

    def handle_robot_pose(self):
        pose = self.robot.base.get_pose()

        if not pose:
            return

        self.robot_pose = pose
        self._send_robot_pose()

    def handle_robot_svg(self, msg):
        try:
            self.robot_svg = json.loads(msg.data)
        except Exception:
            pass
        else:
            self._send_robot_svg()

    def handle_laser_pose(self):
        updated = False

        for frame_id in set(self.tf_frames + self.tf_frames2):
            try:
                (trans, rot) = self.tf_listener.lookupTransform('base', frame_id, rospy.Time())
                rpy = tf.transformations.euler_from_quaternion(rot)
                pose = {
                    'x': trans[0],
                    'y': trans[1],
                    'theta': rpy[2],
                    'flip': abs(abs(rpy[0]) - math.pi) < 1e-3,
                }
            except Exception as ex:
                with self._get_throttler(frame_id + '_pose_error', 5.0):
                    rospy.logerr('[LiveRobot] error obtaining transformation of laser frame: %s', ex)
            else:
                if self.laser_pose.get(frame_id) != pose:
                    self.laser_pose[frame_id] = pose
                    updated = True

        if updated:
            self._send_laser_pose()

    def handle_laser_scan(self, msg):
        frame_id = msg.header.frame_id.lstrip('/')

        # a throttler in disguise
        if frame_id not in self.tf_frames:
            self.tf_frames.append(frame_id)

            self.laser_scan = {
                'frame_id': frame_id,
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'ranges': base64.b64encode(msg.ranges.tobytes()).decode(),
            }

            if self.show_lidar_intensities and msg.intensities:
                self.laser_scan['intensities'] = base64.b64encode(msg.intensities.tobytes()).decode()

            self._send_laser_scan()

    def handle_reflectors(self, msg):
        frame_id = msg.header.frame_id.lstrip('/')

        # a throttler in disguise
        if frame_id not in self.tf_frames2:
            self.tf_frames2.append(frame_id)

            self.reflectors = {
                'frame_id': frame_id,
                'points': [(p.position.x, p.position.y) for p in msg.poses],
            }
            self._send_reflectors()

    def set_station(self, data):
        with self._lock:
            if self._executing:
                return
            self._executing = True

        # Apply Map Data
        if not self.validate_and_apply_map(data):
            rospy.logerr('[LiveRobot] invalid map data received for set station.')
            self._executing = False
            return

        # Get station location
        stations = json.loads(data['stations'])
        station = stations[data['station_idx']]
        location = self.robot.models.get_location_from_station(station)

        # Validate location
        if not self.robot.base.map_tracker.is_valid_heading(location[1]):
            rospy.logerr('[LiveRobot] invalid station heading for set station.')
            self._executing = False
            return

        self.robot.base.set_initial_map_and_location_tracker(self.robot.models.graph, location)
        self.robot.base.set_initial_pose(self.robot.base.map_tracker.get_pose(), precise=True)

        self._executing = False

    def set_pose(self, data):
        with self._lock:
            if self._executing:
                return
            self._executing = True

        try:
            pose = self._get_pose(data['pose'])
        except Exception as e:
            rospy.logerr('[LiveRobot] invalid pose data: %s' % e)
            self._executing = False
            return

        if not self.validate_and_apply_map(data):
            rospy.logerr('[LiveRobot] invalid map data received for set pose.')
            self._executing = False
            return

        location = [0, 0]
        self.robot.base.set_initial_map_and_location_tracker(self.robot.models.graph, location)
        self.robot.base.set_initial_pose(pose, precise=True)
        self._executing = False

    def set_nav_to(self, data):
        with self._lock:
            if self._executing:
                return
            self._executing = True

        if not data.get('uuid'):
            return

        self.resume_robot()
        self._executing_uuid = data['uuid']
        self._execute_thread = threading.Thread(target=self.execute_nav_to, args=(data, ))
        self._execute_thread.start()

        self._reset_deadman_time()
        self._deadman_ev.clear()
        self._deadman_thread = threading.Thread(target=self.deadman_monitor, args=(data['uuid'], ))
        self._deadman_thread.start()

    def _reset_deadman_time(self):
        self._deadman_time = time.time()

    def deadman_monitor(self, monitor_uuid):
        while not self._deadman_ev.wait(1):
            if self._executing_uuid != monitor_uuid:
                return
            if time.time() - self._deadman_time > HEARTBEAT_TIMEOUT:
                self.pause_robot()
            else:
                self.resume_robot()

    def pause_robot(self):
        if self.paused:
            return
        self.paused = True
        self.robot.base.pause()
        self.robot.panel.toggle_led_paused(True)

    def resume_robot(self):
        if not self.paused:
            return
        self.paused = False
        self.robot.base.resume()
        self.robot.panel.toggle_led_paused(False)

    def execute_nav_to(self, data):

        def on_return():
            self._executing = False
            self._executing_uuid = None
            self._deadman_ev.set()

        # Apply Map Data
        if not self.validate_and_apply_map(data):
            rospy.logerr('[LiveRobot] invalid map data received for nav to.')
            on_return()
            return

        # Init location tracker
        try:
            location = data['location']
            self.robot.base.set_initial_map_and_location_tracker(self.robot.models.graph, location)
        except Exception as ex:
            rospy.logerr('[LiveRobot] fail to init location tracker before nav to: %s' % ex)
            on_return()
            return

        # Build and execute skill
        try:
            with self.robot.models.read_lock:
                params = data['params']
                skill_id = self.nav_skill_id if not data['reverse'] else self.reverse_nav_skill_id
                self._sm = state_factory(self.robot, skill_id, dict(self.nav_skill_param_defaults, **params))
        except StateConstructError as ex:
            rospy.logerr('[LiveRobot] fail to construct nav: %s' % ex)
            on_return()
            return

        # TODO: handle abort task for navigation failure.
        try:
            self._sm.execute({})
        except Exception as ex:
            rospy.logerr('[LiveRobot] error while executing nav to skill: %s' % ex)
        else:
            self._executed_uuid = data['uuid']
            self._send_action_completed(data['uuid'])
        finally:
            self._sm = None
            on_return()

    def abort_nav_to(self):
        sm = self._sm
        if not sm:
            return

        sm.request_preempt()
        self.robot.base.stop()

    def validate_and_apply_map(self, data):
        # Genrate Map Data
        m = MapChangeset()
        m.metadata = data['metadata']
        m.structure = data['structure']
        m.stations = data['stations']
        m.map = Map()
        ocg_pk = int(data['ocg'])
        map_ocg = None
        if self.module.fms_manager:
            ocg = self.module.fms_manager.get_map_ocg(ocg_pk)
            if not ocg:
                return

            png_data = base64.b64decode(ocg['png_file'].split(self.png_scheme)[1])
            map_ocg = MapOcg(
                metadata=ocg['metadata'],
                map=m.map,
            )
            map_ocg.png_file = BytesIO(png_data)
        else:
            map_ocg = MapOcg.objects.get(pk=ocg_pk)
        m.ocg = map_ocg
        self.map_validator.assign_maps([m])

        # Validate Map Data
        models = self.robot.models
        with models._lock:
            models.tmp = {}
            models.tmp['executor_mode'] = ExecutorMode.Standalone.value
            models.tmp['teleports'] = []
            models.tmp['geoms'] = models.geoms
            models.tmp['allowed_motions'] = models.allowed_motions
            # No geom if in dfleet mode but inactive.
            no_geoms = not models.geoms
            try:
                self.map_param_validator.validate_and_share_tmp()
                self.map_validator.validate_and_share_tmp()
                if no_geoms:
                    self.standalone_validator.validate_and_share_tmp()
                self.constrained_map_validator.validate_and_share_tmp()
                self.map_param_validator.apply()
                self.map_validator.apply()
                if no_geoms:
                    self.standalone_validator.apply()
                self.constrained_map_validator.apply()
            except ValidationError as ex:
                # TODO: notify user of validation error
                rospy.logerr('[LiveRobot] validation error: %s', ex.error_msg)
                return
            finally:
                models.tmp = None

        return True

    def _get_throttler(self, name, seconds=1):
        if not hasattr(self, '_throttlers'):
            self._throttlers = {}
        if name not in self._throttlers:
            self._throttlers[name] = throttler(seconds)
        return self._throttlers[name]

    def _get_pose(self, p):
        pose = geometry_msgs.msg.Pose2D()
        pose.x = p['x']
        pose.y = p['y']
        pose.theta = p['theta']
        return pose

    def _send_robot_status(self):
        data = {
            'id': 'robot_status',
            'working': self._executing,
        }
        self.out(data)

    def _send_robot_pose(self):
        data = {
            'id': 'robot_pose',
            'robot_pose': {
                'x': self.robot_pose.x,
                'y': self.robot_pose.y,
                'theta': self.robot_pose.theta,
            },
        }
        self.out(data)

    def _send_robot_svg(self):
        if not self.robot_svg:
            return
        data = {
            'id': 'robot_svg',
            'robot_svg': self.robot_svg,
        }
        self.out(data)

    def _send_laser_pose(self):
        data = {
            'id': 'laser_pose',
            'laser_pose': self.laser_pose,
        }
        self.out(data)

    def _send_laser_scan(self):
        data = {
            'id': 'laser_scan',
            'laser_scan': self.laser_scan,
        }
        self.out(data)

    def _send_reflectors(self):
        data = {
            'id': 'reflectors',
            'reflectors': self.reflectors,
        }
        self.out(data)

    def _send_action_completed(self, uuid):
        data = {
            'id': 'action_completed',
            'uuid': uuid,
        }
        self.out(data)

    def _retrieve_all(self):
        self._send_robot_svg()
        self._send_laser_pose()
