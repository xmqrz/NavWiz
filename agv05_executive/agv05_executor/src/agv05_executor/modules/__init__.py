from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import val_lic
from agv05_webserver.system.tasks.dispatch_webhook import dispatch_webhook
import agv05_executive_msgs.srv
import importlib
import rospy
import std_msgs.msg
import std_srvs.srv
import ujson as json

from .module import Module
from .hw_app import HardwareApp

# Monkey-patch: celery pool shared with fms_comm.
celery_app = dispatch_webhook._get_app()
celery_app.conf.broker_pool_limit = 1


class ModuleManager(object):
    modules = [
        'agv05_executor.modules.calibration.Calibration',
        'agv05_executor.modules.dev_mode.DevelopmentMode',
        'agv05_executor.modules.hardware_test.HardwareTest',
        'agv05_executor.modules.manual_control.ManualControl',
        'agv05_executor.modules.manual_line_follow.ManualLineFollow',
        'agv05_executor.modules.self_test.SelfTest',
        'agv05_executor.modules.skill_test.SkillTest',
        'agv05_executor.modules.task_runner.TaskRunner',
        'agv05_executor.modules.wifi_test.WifiTest',
    ]
    tracked_modules = [
        'agv05_executor.modules.homing.Homing',
    ]
    trackless_modules = [
        'agv05_executor.modules.homing_x.HomingX',
        'agv05_executor.modules.map_creator.MapCreator',
        'agv05_executor.modules.live_app.LiveApp',
    ]

    def __init__(self, robot):
        self.robot = robot
        self.modules = {}
        self.running = False
        self.robot.models.agv = self._get_agv_status()

        modules = type(self).modules
        if self.robot.trackless:
            modules += type(self).trackless_modules
        else:
            modules += type(self).tracked_modules
        for m in modules:
            self._add_module(m)

        for app_desc in self.robot.app_manager.app_descriptions:
            self._add_hw_app(app_desc)

        self.__in_pipe = rospy.Subscriber('~module_in', std_msgs.msg.String, self.handle_in_pipe)
        self.__out_pipe = rospy.Publisher('~module_out', std_msgs.msg.String, queue_size=10)
        self.__module_running_pub = rospy.Publisher('~module_running', std_msgs.msg.Bool, queue_size=1, latch=True)
        self.__hot_reloaded_sub = rospy.Subscriber('~hot_reloaded', std_msgs.msg.Empty, self.handle_hot_reloaded)

        self.__set_module_running_service = rospy.Service('~set_module_running', agv05_executive_msgs.srv.SetModuleRunning, self.handle_set_module_running)
        self.__get_module_running_service = rospy.Service('~get_module_running', std_srvs.srv.Trigger, self.handle_get_module_running)
        self.__get_agv_status_service = rospy.Service('~get_agv_status', std_srvs.srv.Trigger, self.handle_get_agv_status)
        self.__set_module_running = rospy.ServiceProxy('~set_module_running', agv05_executive_msgs.srv.SetModuleRunning, persistent=False)

        self.__agv_update_hook = None
        self.default_module = None
        self.__default_module_timer = None

        self.__module_running_pub.publish(std_msgs.msg.Bool(data=False))

    def _add_module(self, module_path):
        try:
            module_name, cls_name = module_path.rsplit('.', 1)
            module = importlib.import_module(module_name)
            cls = getattr(module, cls_name)

            if not issubclass(cls, Module):
                rospy.logerr('Not a valid module [%s].', cls)
                return

            if cls.id in self.modules:
                rospy.logerr('Another module with the same id as this module [%s] has been registered.', cls)
                return

            self.modules[cls.id] = cls(self)
        except Exception as ex:
            rospy.logerr('Cannot load module [%s]: %s', module_path, ex)

    def _add_hw_app(self, app_desc):
        try:
            app_id = app_desc['id']
            cls = self.robot.app_manager.apps[app_id]
            if app_id in self.modules:
                rospy.logerr('Another module with the same id as this hw app [%s] has been registered.', app_desc['id'])
                return

            self.modules[app_id] = HardwareApp(cls, app_desc, self)
        except Exception as ex:
            rospy.logerr('Cannot load hw app [%s]: %s', app_desc['id'], ex)

    def setup_agv_status_update(self, diagnostic):
        self._diagnostic = diagnostic
        self.__agv_update_timer = rospy.Timer(rospy.Duration(1.0), self.on_get_agv_update)

    def validate(self):
        self.__r = val_lic()
        return self.__r

    def start(self):
        if self.running or not self.__r[0]:
            return
        self.running = True
        if self.robot.fms_manager:
            self._executor_cfg_md5 = self.robot.models.downloadables_md5['executor_cfg'] if \
                self.robot.fms_manager.is_sync_ready() else None
        self.default_module = self._collect_default_module()
        self.__default_module_timer = rospy.Timer(rospy.Duration(1.0), self._countdown_default_module)

    def stop(self):
        if not self.running:
            return
        self.running = False
        self._cancel_default_module(join=True)
        reservation = self.robot.get_reservation()
        if reservation:
            rospy.logwarn('Forcing to stop robot while the module "%s" is still running.', reservation)
            self.robot.unreserve(reservation)
            self.modules[reservation].stop()

    def handle_manager_in_pipe(self, data):
        try:
            cmd = data['command']
            if cmd == 'active_module':
                self._send_active_module()
                self._send_default_module()
                self._resend_fms_status()
            elif cmd == 'cancel_default_module':
                self._cancel_default_module()

        except KeyError as ex:
            rospy.logerr('[_ModuleManager_] invalid input: %s', ex)

    def _start_module(self, module_id):
        success = False
        if module_id in self.modules:
            if self.robot.get_reservation() == module_id:
                success = True
            elif self.robot.reserve(module_id):
                rospy.loginfo('Starting module "%s".', module_id)
                if self.robot.fms_manager:
                    self.robot.fms_manager.apply_downloadables()
                    self.robot.fms_manager.cancel_occupation_request()
                    self.robot.variable.init()
                try:
                    self.modules[module_id].start()
                    self.__module_running_pub.publish(std_msgs.msg.Bool(data=True))
                    success = True
                except Exception as e:
                    rospy.logerr('Fail to start module "%s".', module_id)
                    try:
                        self.robot.unreserve(module_id)
                        self.modules[module_id].stop()
                    except Exception:
                        pass

        self.manager_out({
            'command': 'start_module',
            'module_id': module_id,
            'success': success,
        })
        if success:
            self._send_active_module()
            self._cancel_default_module()
        return success

    def _stop_module(self, module_id):
        success = False
        if module_id in self.modules:
            if self.robot.unreserve(module_id):
                rospy.loginfo('Stopping module "%s".', module_id)
                self.modules[module_id].stop()
                if self.robot.fms_manager:
                    self.robot.fms_manager.apply_downloadables()
                    self.robot.fms_manager.cancel_occupation_request()
                    self.robot.variable.init()
                self.__module_running_pub.publish(std_msgs.msg.Bool(data=False))
                success = True

        self.manager_out({
            'command': 'stop_module',
            'module_id': module_id,
            'success': success,
        })
        if success:
            self._send_active_module()
        return success

    def _cancel_default_module(self, join=False):
        self.default_module = None
        t = self.__default_module_timer
        if t:
            t.shutdown()
            if join:
                t.join()
            self.__default_module_timer = None
        self._send_default_module()

    def _countdown_default_module(self, timer_event=None):
        if self.robot.fms_manager:
            if not self.robot.fms_manager.is_sync_ready():
                return
            if self._executor_cfg_md5 != self.robot.models.downloadables_md5['executor_cfg']:
                self._executor_cfg_md5 = self.robot.models.downloadables_md5['executor_cfg']
                self.default_module = self._collect_default_module()
                self._send_default_module()
                return

        if not self.default_module:
            return

        self.default_module['countdown'] -= 1
        self._send_default_module()

        if self.default_module['countdown'] <= 0:
            # pipe it through ROS service so that the operation is synchronized (run in a single thread)
            self.__set_module_running(operation=1, module_id=self.default_module['module'])

    def _collect_default_module(self):
        dm = self.robot.models.default_app
        if not dm:
            return None
        return {'module': dm['app'], 'countdown': dm['timeout']}

    def _send_active_module(self):
        self.manager_out({
            'command': 'active_module',
            'module_id': self.robot.get_reservation(),
        })

    def _send_default_module(self):
        self.manager_out({
            'command': 'default_module',
            'default_module': self.default_module,
        })

    def _resend_fms_status(self):
        if self.robot.fms_manager:
            self.robot.fms_manager.resend_status()

    def manager_out(self, data):
        self.out({
            'id': '__',
            'data': data,
        })

    def handle_in_pipe(self, msg):
        if not self.running:
            return
        try:
            msg = json.loads(msg.data)
            module_id = msg['id']

            if module_id == '__':
                self.handle_manager_in_pipe(msg['data'])
            elif module_id in self.modules and module_id == self.robot.get_reservation():
                self.modules[module_id].handle_in_pipe(msg['data'])
        except KeyError:
            rospy.logwarn('Module input pipe format error.')

    def out(self, data):
        if not self.running:
            return
        self.__out_pipe.publish(std_msgs.msg.String(data=json.dumps(data)))

    def handle_set_module_running(self, req):
        if not self.running:
            return False, 'Robot controller is not running.'

        if req.operation == 1:
            success = self._start_module(req.module_id)
        else:
            success = self._stop_module(req.module_id)
        return success, ''

    def handle_get_module_running(self, req):
        if not self.running:
            return False, ''

        reservation = self.robot.get_reservation()
        if reservation:
            return True, reservation
        else:
            return False, ''

    def handle_get_agv_status(self, req):
        agv = self.robot.models.agv
        return True, json.dumps(agv)

    def on_get_agv_update(self, timer_event):
        agv = self._get_agv_status()

        if self.robot.fms_manager:
            fms_status, _, fms_status_code = self.robot.fms_manager.get_status()
            if not agv['error_code']:
                agv['error_code'] = fms_status_code
            agv['fms_status'] = fms_status
        else:
            agv['fms_status'] = None

        if agv != self.robot.models.agv:
            dispatch_webhook.apply_async(('agv_update', agv), expires=30)
            self.robot.models.agv = agv
            self._diagnostic.update(agv)
        else:
            self._diagnostic.update()

    def _get_agv_status(self):
        agv = {
            'uuid': self.robot.models.agv_uuid,
            'name': self.robot.models.agv_name,
            'status': 'Powered Off',
            'action': '',
            'location': None,
            'prev_location': None,
            'location_hint': '',
            'pose': None,
            'motion': None,
            'velocity': [0.0, 0.0, 0.0],
            'dimension_profile': 0,
            'error_code': 0,
            'safety_message': None,
            'user_message': None,
            'charging': bool(self.robot.power.charging_status),
            'battery': self.robot.power.battery_percentage,
            'task_counter': self.robot.models.task_counter,
            'mileage': self.robot.base.mileage,
            'fms_status': None,
        }

        try:
            agv = self.hook_agv_update(agv)
        except Exception as ex:
            rospy.logerr('Unhandled exception in agv_update_hook: %s', ex)
        return agv

    def hook_agv_update(self, agv):
        if not self.running:
            return agv

        v = self.robot.base.cmd_vel
        agv['status'] = 'Not Ready'
        agv['action'] = ''
        agv['location'] = self.robot.base.get_location()
        agv['prev_location'] = self.robot.base.get_prev_location()
        agv['location_hint'] = self.robot.base.get_location_hint()
        agv['motion'] = self.robot.base.get_motion()
        agv['velocity'] = [v.linear.x, v.linear.y, v.angular.z]
        agv['dimension_profile'] = self.robot.base.get_dimension_profile()
        agv['error_code'] = self.robot.robot_control.panel_control.get_error_code()
        agv['safety_message'] = self.robot.robot_control.panel_control.get_safety_message()
        agv['user_message'] = self.robot.robot_control.panel_control.get_user_message()

        if hasattr(self.robot.base, 'get_pose') and self.robot.is_alive():
            pose = self.robot.base.get_pose()
            if pose:
                # convert pose so that it is json-serializable
                agv['pose'] = {k: getattr(pose, k) for k in pose.__slots__}

        reservation = self.robot.get_reservation()
        if not reservation:
            pass
        elif self.__agv_update_hook:
            self.__agv_update_hook(agv)
        elif reservation == 'task-runner':
            # Hack: special handling to prevent status overwrite
            pass
        elif self.robot.fms_manager and self.robot.fms_manager.is_sync_pending():
            agv['status'] = 'Running App (Sync Pending)'
        else:
            agv['status'] = 'Running App'
        return agv

    def set_agv_update_hook(self, cb):
        self.__agv_update_hook = cb

    def handle_hot_reloaded(self, msg):
        with self.robot.lock:
            reservation = self.robot.get_reservation()
            if reservation:
                self.modules[reservation].handle_in_pipe({
                    'command': 'hot_reloaded'
                })
