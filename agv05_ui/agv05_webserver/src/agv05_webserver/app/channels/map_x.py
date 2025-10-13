from __future__ import absolute_import
from __future__ import unicode_literals

from PIL import Image
from agv05_webserver.system.models import db_auto_reconnect, Map
from agv05_webserver.systemx.models import MapOcg, OccupancyGrid
from django.core.files.base import ContentFile
from io import BytesIO
from rospy.numpy_msg import numpy_msg
from wraptor.context import throttle as throttler
import base64
import distutils.util
import geometry_msgs.msg
import logging
import math
import nav_msgs.msg
import os
import rospy
import sensor_msgs.msg
import std_msgs.msg
import tf
import ujson as json

from .channel import Channel

logger = logging.getLogger(__name__)

PNG_MAGIC_BYTES = b'\x89PNG\r\n\x1a\n'


class MapXChannel(Channel):
    id = 'mapx'

    def __init__(self, *args, **kwargs):
        super(MapXChannel, self).__init__(*args, **kwargs)
        self.tf_frames = ['laser']  # for scan
        self.tf_frames2 = []  # for reflector
        self.tf_frames3 = []  # for docking reflector
        self.tf_cloud = False

        try:
            self.show_lidar_intensities = distutils.util.strtobool(os.environ['SHOW_LIDAR_INTENSITIES'])
        except Exception:
            self.show_lidar_intensities = False

        # palette index.
        # Format: 768 ints. Each group of 3 values represent the RGB values for the index.
        # OCG map: Occupancy probabilities are in the range [0 - 100]. Unknown space is -1 (255).
        self._map_palette = [200 - i for i in range(101) for j in range(3)]
        self._map_palette[0:3] = [255, 255, 255]  # white (free space)
        self._map_palette[100 * 3:101 * 3] = [0, 0, 0]  # black (obstacle)

        # Costmap: Costs are in the range [0 - 100]. Lethal cost is 100. Inscribed cost is 99. Unknown space is -1 (255).
        self._costmap_palette = [j for i in range(101) for j in [100 + i, 0, 200 - i]]
        self._costmap_palette[0:3] = [255, 255, 255]  # white (free space)
        self._costmap_palette[99 * 3:100 * 3] = [0, 255, 255]  # cyan (inscribed)
        self._costmap_palette[100 * 3:101 * 3] = [255, 255, 0]  # yellow (lethal)

    def on_subscribe(self, connection):
        if self.get_num_subscribers() <= 1:
            self._start()

    def on_unsubscribe(self, connection):
        if self.get_num_subscribers() <= 1:
            self._stop()

    def _start(self):
        self.status = 'idle'
        self.laser_pose = {
            'laser': {'x': 0, 'y': 0, 'theta': 0, 'flip': False},
        }
        self.costmap = None
        self.raw_map = None
        self.map_layout = None
        self.robot_motion = None
        self.robot_svg = None
        self.robot_path = None
        self.docking_plan = None
        self._map = None
        self._last_saved = None

        self.__robot_running_sub = rospy.Subscriber(self.agv05_executor + '/robot_running', std_msgs.msg.UInt8, self.handle_robot_running, queue_size=1)
        self.__laser_subs = [rospy.Subscriber(topic, numpy_msg(sensor_msgs.msg.LaserScan), self.handle_laser_scan, queue_size=1) for
            topic in rospy.get_param('nav2d_scan_topics', 'scan').split()]
        self.__costmap_sub = rospy.Subscriber('/agv05_navx/costmap/costmap', OccupancyGrid, self.handle_costmap, queue_size=1)
        self.__map_sub = rospy.Subscriber('/map', OccupancyGrid, self.handle_map, queue_size=1)
        self.__map_layout_sub = rospy.Subscriber('/map_layout', std_msgs.msg.String, self.handle_map_layout, queue_size=1)
        self.__reflectors_sub = rospy.Subscriber('/reflectors', geometry_msgs.msg.PoseArray, self.handle_reflectors, queue_size=1)
        self.__docking_reflectors_sub = rospy.Subscriber('/docking_reflectors', geometry_msgs.msg.PoseArray, self.handle_docking_reflectors, queue_size=1)
        self.__cloud_flatten_sub = rospy.Subscriber('/cloud_flatten', sensor_msgs.msg.PointCloud2, self.handle_docking_cloud, queue_size=1)
        self.__robot_motion_sub = rospy.Subscriber('/robot_motion', std_msgs.msg.String, self.handle_robot_motion, queue_size=1)
        self.__robot_svg_sub = rospy.Subscriber('/robot_svg', std_msgs.msg.String, self.handle_robot_svg, queue_size=1)
        self.__robot_path_sub = rospy.Subscriber('/agv05_navx/planner/plan', nav_msgs.msg.Path, self.handle_robot_path, queue_size=1)
        self.__docking_plan_sub = rospy.Subscriber('/agv05_navx/docking/plan', nav_msgs.msg.Path, self.handle_docking_plan, queue_size=1)
        self.__pose_timer = rospy.Timer(rospy.Duration(0.5), self.handle_pose)
        self.__save_timer = None

    def _stop(self):
        self.__robot_running_sub.unregister()
        for sub in self.__laser_subs:
            sub.unregister()
        self.__costmap_sub.unregister()
        self.__map_sub.unregister()
        self.__map_layout_sub.unregister()
        self.__reflectors_sub.unregister()
        self.__robot_motion_sub.unregister()
        self.__robot_svg_sub.unregister()
        self.__robot_path_sub.unregister()
        self.__docking_plan_sub.unregister()
        self.__pose_timer.shutdown()
        self._stop_mapping()

    def on_message(self, data, connection):
        try:
            if data['id'] == 'retrieve_all':
                self._retrieve_all(connection)
            elif data['id'] == 'start_mapping':
                self._start_mapping()
            elif data['id'] == 'save_map':
                self._save_map()
            elif data['id'] == 'stop_mapping':
                self._stop_mapping()
        except KeyError as ex:
            pass

    def handle_robot_running(self, msg):
        running_mode = msg.data
        if running_mode != 2:
            self._stop_mapping()

        if running_mode == 0:
            self.status = 'idle'
        elif running_mode in (1, 3):
            self.status = 'amcl'

    def handle_laser_pose(self):
        updated = False

        for frame_id in set(self.tf_frames + self.tf_frames2 + self.tf_frames3):
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
                    logger.error('Error obtaining transformation of laser frame: %s', ex)
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

    def handle_docking_reflectors(self, msg):
        frame_id = msg.header.frame_id.lstrip('/')

        # a throttler in disguise
        if frame_id not in self.tf_frames3:
            self.tf_frames3.append(frame_id)

            self.docking_reflectors = {
                'frame_id': frame_id,
                'points': [(p.position.x, p.position.y) for p in msg.poses],
            }
            self._send_docking_reflectors()

    def handle_docking_cloud(self, msg):
        frame_id = msg.header.frame_id.lstrip('/')

        # a throttler in disguise
        if self.tf_cloud or not frame_id:  # send to clear display if frame_id empty
            self.tf_cloud = False

            if self.get_num_subscribers() >= 1:
                self.docking_cloud = {
                    'frame_id': frame_id,
                    'cloud': base64.b64encode(msg.data).decode(),
                }
                self._send_docking_cloud()

    def handle_costmap(self, msg):
        if self.status == 'idle':
            return

        data = msg.data.tobytes()
        if data[:len(PNG_MAGIC_BYTES)] == PNG_MAGIC_BYTES:
            png = data
        else:
            # convert costmap data to png
            png = BytesIO()
            im = Image.frombytes('P', (msg.info.width, msg.info.height), data)
            im.putpalette(self._costmap_palette)
            im.save(png, 'png', transparency=255)  # set palette index 255 (unknown space) as transparent
            png = png.getvalue()

        self.costmap = {
            'frame_id': msg.header.frame_id,
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'x0': msg.info.origin.position.x,
            'y0': msg.info.origin.position.y,
            'png': base64.b64encode(png).decode(),
        }
        self._send_costmap()

    def handle_map(self, msg):
        if self.status == 'idle':
            return
        if self.status == 'amcl' and self.raw_map and self._raw_map_stamp == msg.header.stamp:
            return

        data = msg.data.tobytes()
        if data[:len(PNG_MAGIC_BYTES)] == PNG_MAGIC_BYTES:
            png = data
        else:
            # convert map data to png
            png = BytesIO()
            im = Image.frombytes('P', (msg.info.width, msg.info.height), data)
            im.putpalette(self._map_palette)
            im.save(png, 'png', transparency=255)  # set palette index 255 (unknown space) as transparent
            png = png.getvalue()

        self.raw_map = {
            'frame_id': msg.header.frame_id,
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'x0': msg.info.origin.position.x,
            'y0': msg.info.origin.position.y,
            'png': base64.b64encode(png).decode(),
        }
        self._map_ocg = dict(self.raw_map, png=png)
        self._raw_map_stamp = msg.header.stamp
        self._send_raw_map()

    def handle_map_layout(self, msg):
        if self.status != 'amcl':
            return
        if self.map_layout == msg.data:
            return
        self.map_layout = msg.data
        self._send_map_layout()

    def handle_robot_motion(self, msg):
        if self.status != 'amcl':
            return
        try:
            self.robot_motion = json.loads(msg.data)
        except Exception:
            pass
        else:
            self._send_robot_motion()

    def handle_robot_svg(self, msg):
        try:
            self.robot_svg = json.loads(msg.data)
        except Exception:
            pass
        else:
            self._send_robot_svg()

    def handle_robot_path(self, msg):
        self.robot_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._send_robot_path()

    def handle_robot_pose(self):
        if self.status == 'idle':
            return

        pose = None
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base', rospy.Time())
            rpy = tf.transformations.euler_from_quaternion(rot)
            pose = {
                'x': trans[0],
                'y': trans[1],
                'theta': rpy[2],
            }
        except Exception as ex:
            with self._get_throttler('pose_error', 5.0):
                logger.error('Error obtaining transformation of base frame: %s', ex)

        if not pose:
            return

        self.robot_pose = pose
        self._send_robot_pose()

    def handle_docking_plan(self, msg):
        self.docking_plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._send_docking_plan()

    def handle_target_pose(self):
        if self.status == 'idle':
            return

        pose = None
        try:
            t = self.tf_listener.getLatestCommonTime('map', 'marker')
            if (rospy.Time.now() - t).to_sec() <= 1:
                (trans, rot) = self.tf_listener.lookupTransform('map', 'marker', rospy.Time())
                rpy = tf.transformations.euler_from_quaternion(rot)
                pose = {
                    'x': trans[0],
                    'y': trans[1],
                    'theta': rpy[2],
                }
        except Exception as ex:
            pass

        if not pose and not getattr(self, 'marker_pose', None):
            return

        self.marker_pose = pose

        pose = None
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'target', rospy.Time())
            rpy = tf.transformations.euler_from_quaternion(rot)
            pose = {
                'x': trans[0],
                'y': trans[1],
                'theta': rpy[2],
            }
        except Exception as ex:
            pass

        self.target_pose = pose
        self._send_target_pose()

    def handle_pose(self, timer_event):
        self.handle_robot_pose()
        self.handle_target_pose()
        self.handle_laser_pose()
        self.tf_frames = []   # for scan
        self.tf_frames2 = []  # for reflector
        self.tf_frames3 = []  # for docking reflector
        self.tf_cloud = True

    def _get_throttler(self, name, seconds=1):
        if not hasattr(self, '_throttlers'):
            self._throttlers = {}
        if name not in self._throttlers:
            self._throttlers[name] = throttler(seconds)
        return self._throttlers[name]

    def _start_mapping(self):
        if self.status == 'mapping':
            return

        self.status = 'mapping'
        with db_auto_reconnect():
            self._map = Map.objects.create()
        self.__save_timer = rospy.Timer(rospy.Duration(30.0), self._save_map)
        self._send_updates()
        logger.info('Mapping started.')

    def _stop_mapping(self):
        if self.status != 'mapping':
            return

        self.__save_timer.shutdown()
        self._save_map('auto')
        self._map = None
        self._last_saved = None
        self.raw_map = None
        self.status = 'idle'
        self._send_updates()
        logger.info('Mapping stopped.')

    def _save_map(self, timer_event=None):
        if self.status != 'mapping' or not self._map or not self.raw_map:
            return

        # assign a local copy to prevent race condition.
        metadata = self._map_ocg

        # the metadata will have the 'png' key if it hasn't been saved.
        if 'png' in metadata:
            with self._get_throttler('save_map', 2.0), db_auto_reconnect():
                map_ocg = MapOcg(map=self._map)
                if timer_event:
                    map_ocg.is_autosaved = True

                png = metadata['png']
                del metadata['png']
                map_ocg.metadata = json.dumps(metadata)
                map_ocg.png_file.save('raw.png', ContentFile(png))  # auto-save model.

                self._last_saved = int(rospy.get_time())

        self._send_save_map()

    def _send_laser_pose(self, connection=None):
        data = {
            'id': 'laser_pose',
            'laser_pose': self.laser_pose,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_laser_scan(self):
        data = {
            'id': 'laser_scan',
            'laser_scan': self.laser_scan,
        }
        self.broadcast(data)

    def _send_reflectors(self):
        data = {
            'id': 'reflectors',
            'reflectors': self.reflectors,
        }
        self.broadcast(data)

    def _send_docking_reflectors(self):
        data = {
            'id': 'docking_reflectors',
            'docking_reflectors': self.docking_reflectors,
        }
        self.broadcast(data)

    def _send_docking_cloud(self):
        data = {
            'id': 'docking_cloud',
            'docking_cloud': self.docking_cloud,
        }
        self.broadcast(data)

    def _send_map_layout(self, connection=None):
        if not self.map_layout:
            return

        data = {
            'id': 'map_layout',
            'map_layout': self.map_layout,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_robot_motion(self, connection=None):
        if not self.robot_motion:
            return

        data = {
            'id': 'robot_motion',
            'robot_motion': self.robot_motion,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_robot_svg(self, connection=None):
        if not self.robot_svg:
            return

        data = {
            'id': 'robot_svg',
            'robot_svg': self.robot_svg,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_raw_map(self, connection=None):
        if not self.raw_map:
            return

        data = {
            'id': 'raw_map',
            'raw_map': self.raw_map,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_costmap(self, connection=None):
        if not self.costmap:
            return

        data = {
            'id': 'costmap',
            'costmap': self.costmap,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_robot_path(self, connection=None):
        data = {
            'id': 'robot_path',
            'robot_path': self.robot_path,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_robot_pose(self):
        data = {
            'id': 'robot_pose',
            'robot_pose': self.robot_pose,
        }
        self.broadcast(data)

    def _send_docking_plan(self, connection=None):
        data = {
            'id': 'docking_plan',
            'docking_plan': self.docking_plan,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_target_pose(self):
        data = {
            'id': 'target_pose',
            'marker_pose': self.marker_pose,
            'target_pose': self.target_pose,
        }
        self.broadcast(data)

    def _send_save_map(self, connection=None):
        if not self._last_saved:
            return

        data = {
            'id': 'save_map',
            'timestamp': self._last_saved,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _send_updates(self, connection=None):
        data = {
            'id': 'status',
            'status': self.status,
        }
        if connection:
            self.send(data, connection)
        else:
            self.broadcast(data)

    def _retrieve_all(self, connection):
        self._send_laser_pose(connection)
        self._send_map_layout(connection)
        self._send_robot_motion(connection)
        self._send_robot_svg(connection)
        self._send_robot_path(connection)
        self._send_docking_plan(connection)
        self._send_raw_map(connection)
        self._send_save_map(connection)
        self._send_updates(connection)
