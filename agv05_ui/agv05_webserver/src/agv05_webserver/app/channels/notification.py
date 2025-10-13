from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, Variable, db_auto_reconnect
from django.conf import settings as django_settings
from django.utils import timezone
from tornado.ioloop import PeriodicCallback
import agv05_msgs.msg
import linux_wifi.msg
import linux_wifi.srv
import logging
import rospy
import std_msgs.msg
import std_srvs.srv
import subprocess
import ujson as json

from .channel import Channel
from agv05_webserver.system.views.auth import get_user_info


logger = logging.getLogger(__name__)
wifi_manager = getattr(django_settings, 'WIFI_MANAGER')


def service_adhoc(enable):
    try:
        srv = rospy.ServiceProxy(wifi_manager + '/adhoc', std_srvs.srv.SetBool, persistent=False)
        return srv(not not enable)
    except Exception as ex:
        logger.error('Failed to %s adhoc wifi: %s', 'enable' if enable else 'disable', ex)


class NotificationChannel(Channel):
    id = 'notification'
    power_off_timeout = 120
    extended_power_off_timeout = 1800

    def __init__(self, *args, **kwargs):
        super(NotificationChannel, self).__init__(*args, **kwargs)
        self.data = {
            'robot_running': {
                'id': 'robot_running',
                'value': False,
            },
            'robot_name': None,
            'tracker': None,
            'battery': None,
            'disk_space': None,
            'maintenance': None,
            'wifi': None,
            'adhoc': None,
        }

        self.__is_robot_running = False
        self.__is_countdown_running = False
        self.charging = False
        self.manual_charging = False
        self.countdown_counter = self.power_off_timeout
        self.countdown_timer = PeriodicCallback(self.handle_countdown_callback, 1000)

        # Create all required ROS publishers and subscribers
        self.__robot_running_sub = rospy.Subscriber(self.agv05_executor + '/robot_running', std_msgs.msg.UInt8, self.handle_robot_running, queue_size=1)
        self.__downtime_tracker_pub = rospy.Publisher(self.agv05_executor + '/downtime_tracker_in', std_msgs.msg.String, queue_size=1)
        self.__downtime_tracker_sub = rospy.Subscriber(self.agv05_executor + '/downtime_tracker_out', std_msgs.msg.String, self.handle_downtime_tracker, queue_size=1)
        self.__battery_charging_sub = rospy.Subscriber('agv05/power/battery_state', agv05_msgs.msg.BatteryState, self.handle_battery_state, queue_size=1)
        self.__wifi_status_sub = rospy.Subscriber('/wifi_manager/status', linux_wifi.msg.WifiStatus, self.handle_wifi_status, queue_size=1)

        self.maintenance_timer = PeriodicCallback(self.check_maintenance, 30 * 60 * 1000)  # 30 mins
        self.maintenance_timer.start()
        self.check_maintenance()

    def on_subscribe(self, connection):
        self.retrieve_all(connection)
        self.handle_auth_update(connection)

        if (not django_settings.NO_AUTO_POWER_OFF and self.countdown_timer is not None and
                not self.__is_robot_running and not self.__is_countdown_running):
            rospy.set_param('/agv05_sockserver/extend_countdown', False)
            self.countdown_counter = self.power_off_timeout
            self.countdown_timer.start()
            self.__is_countdown_running = True

        self.handle_countdown_update(self.countdown_counter)

    def on_unsubscribe(self, connection):
        pass

    def on_message(self, data, connection):
        try:
            if data['id'] == 'retrieve_all':
                self.retrieve_all(connection)
                self.handle_auth_update(connection)
            elif data['id'] == 'tracker':
                self.__downtime_tracker_pub.publish(std_msgs.msg.String(data=json.dumps(data)))
            elif data['id'] == 'adhoc':
                state = data['enable']
                self.cur_state = service_adhoc(state)
                self.handle_adhoc(self.cur_state)
            elif data['id'] == 'auth_token':
                token = data['token']
                connection.auth_manager.setup_token_user(connection, token)
                self.handle_auth_update(connection)
        except KeyError:
            pass

    def retrieve_all(self, connection):
        self._send_time(connection)

        if self.data['robot_running']:
            self.send(self.data['robot_running'], connection)

        if self.data['robot_name']:
            self.send(self.data['robot_name'], connection)

        if self.data['tracker']:
            self.send(self.data['tracker'], connection)

        if self.data['battery']:
            self.send(self.data['battery'], connection)

        if self.data['disk_space']:
            self.send(self.data['disk_space'], connection)

        if self.data['maintenance']:
            self.send(self.data['maintenance'], connection)

        if self.data['wifi']:
            self.send(self.data['wifi'], connection)

        if self.data['adhoc']:
            self.send(self.data['adhoc'], connection)

    def handle_robot_running(self, msg):
        data = {
            'id': 'robot_running',
            'value': msg.data
        }
        self.broadcast(data)
        self.data['robot_running'] = data
        self.handle_robot_name()
        self.__is_robot_running = msg.data

        if self.countdown_timer is not None:
            if self.__is_robot_running:
                self.countdown_timer.stop()
                self.__is_countdown_running = False
            elif not django_settings.NO_AUTO_POWER_OFF and not self.__is_countdown_running:
                rospy.set_param('/agv05_sockserver/extend_countdown', False)
                self.countdown_counter = self.power_off_timeout
                self.countdown_timer.start()
                self.__is_countdown_running = True

    def handle_downtime_tracker(self, msg):
        msg = json.loads(msg.data)
        data = dict(msg, id='tracker')
        self.broadcast(data)
        self.data['tracker'] = data

    def handle_adhoc(self, stat):
        data = {
            'id': 'adhoc',
            'value': stat,
        }
        self.broadcast(data)
        self.data['adhoc'] = data

    def handle_auth_update(self, connection):
        data = get_user_info(connection.user)
        data['id'] = 'auth'
        self.send(data, connection)

    @db_auto_reconnect()
    def handle_robot_name(self):
        data = {
            'id': 'robot_name',
            'value': Cache.get_agv_name(),
        }
        self.broadcast(data)
        self.data['robot_name'] = data

    def handle_battery_state(self, msg):
        data = {
            'id': 'battery',
            'value': int(5 * round(float(msg.percentage) / 5)),
            'state': msg.state in [agv05_msgs.msg.BatteryState.AUTO_CHARGING, agv05_msgs.msg.BatteryState.MANUAL_CHARGING],
            'manual': msg.state == agv05_msgs.msg.BatteryState.MANUAL_CHARGING,
        }
        if self.data['battery'] != data:
            self.broadcast(data)
            self.data['battery'] = data

    def handle_wifi_status(self, msg):
        data = {
            'id': 'wifi',
            'valueip': msg.ip,
            'valuestate': msg.state,
            'ssid': msg.ssid,
            'mac': msg.mac,
            'signallevel': msg.signallevel,
        }
        if self.data['wifi'] != data:
            self.broadcast(data)
            self.data['wifi'] = data

    def handle_countdown_update(self, msg):
        data = {
            'id': 'countdown',
            'value': int(msg),
        }
        self.broadcast(data)
        self.data['countdown'] = data

    def handle_countdown_callback(self):
        if rospy.get_param('/agv05_sockserver/extend_countdown', False):
            rospy.set_param('/agv05_sockserver/extend_countdown', False)
            self.countdown_counter = self.extended_power_off_timeout
        if self.countdown_counter:
            self.handle_countdown_update(self.countdown_counter)
            self.countdown_counter = self.countdown_counter - 1
        else:
            self.power_off()

    @db_auto_reconnect()
    def check_maintenance(self):
        # check hard disk space
        try:
            p = subprocess.Popen(['df', '--output=pcent', '/'], stdout=subprocess.PIPE, universal_newlines=True)
            output = p.communicate()[0]

            free_space = 100 - int(output.split()[-1].replace('%', ''))
            data = {
                'id': 'disk_space',
                'remaining': free_space,
                'warning': free_space < 40,
            }
            self.broadcast(data)
            self.data['disk_space'] = data
        except Exception:
            pass

        # check maintenance
        variable_info = [Variable.NEXT_PM_DUE, Variable.NEXT_PM_MILEAGE]
        info = dict(Variable.shared_qs().filter(pk__in=variable_info).values_list('name', 'value'))

        try:
            next_pm_due = Variable.parse_date(info[Variable.NEXT_PM_DUE]) or timezone.datetime(2020, 8, 1).date()
        except Exception:
            next_pm_due = timezone.datetime(2020, 8, 1).date()

        try:
            next_pm_mileage = int(info[Variable.NEXT_PM_MILEAGE])
        except Exception:
            next_pm_mileage = 0

        mileage = Variable.get_mileage()

        if timezone.localdate() >= next_pm_due:
            data = {
                'id': 'maintenance',
                'warning': 'Preventive maintenance was due on %s. Please arrange for maintenance.' % next_pm_due.strftime('%d/%m/%Y'),
            }
            self.broadcast(data)
            self.data['maintenance'] = data

        elif mileage >= next_pm_mileage:
            data = {
                'id': 'maintenance',
                'warning': 'Preventive maintenace was due for mileage %sm. Please arrange for maintenance.' % '{:,.0f}'.format(next_pm_mileage),
            }
            self.broadcast(data)
            self.data['maintenance'] = data

        else:
            data = {
                'id': 'maintenance',
            }
            self.broadcast(data)
            self.data['maintenance'] = data

    def power_off(self):
        # Turn off embedded first, followed by PC itself.
        try:
            shutdown = rospy.ServiceProxy(self.agv05_executor + '/shutdown', std_srvs.srv.Trigger, persistent=False)
            shutdown()
        except Exception:
            pass

        subprocess.Popen(['sudo', 'poweroff'])

    def _send_time(self, connection):
        data = {
            'id': 'server_time',
            'now': timezone.localtime().isoformat()
        }
        self.send(data, connection)
