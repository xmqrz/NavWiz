from __future__ import absolute_import
from __future__ import unicode_literals

import agv05_msgs.msg
import capabilities.srv
import contextlib
import rospy
import six
import std_msgs.msg
import threading
import time

from .logger import MessageLogger
from .robot import CapabilityLauncherMixin, Robot


class Agv05Panel(object):
    # Lowest id has the highest priority
    START_ID = 1
    STOP_ID = 0
    KEYPAD_ID = 2
    MAX_ID = 3
    start_button_topic = '/agv05/panel/control/button_start'
    stop_button_topic = '/agv05/panel/control/button_stop'
    keypad_topic = '/agv05/panel/control/keypad'
    popup_topic = '/agv05/panel/control/popup'
    start_led_topic = '/agv05/panel/control/led_start'
    stop_led_topic = '/agv05/panel/control/led_stop'
    led_control_user_topic = '/agv05/led/control/user'
    led_control_nav_topic = '/agv05/led/control/navigation'

    def __init__(self, robot):
        self.robot = robot
        self.logger = MessageLogger(robot)
        self.ev = threading.Event()
        self.button_state = [False for i in range(self.MAX_ID)]
        self.prev_button_state = [False for i in range(self.MAX_ID)]
        self.in_context = False
        self._paused = None

        # Provide callbacks
        self.__button_release_callbacks = set()
        self.register_button_release_callback = lambda cb: self.__button_release_callbacks.add(cb)
        self.unregister_button_release_callback = lambda cb: self.__button_release_callbacks.discard(cb)

        # ROS subscribers and publishers
        self.__start_button_sub = rospy.Subscriber(self.start_button_topic, std_msgs.msg.Bool, self._handle_start_button, queue_size=10)
        self.__stop_button_sub = rospy.Subscriber(self.stop_button_topic, std_msgs.msg.Bool, self._handle_stop_button, queue_size=10)
        self.__keypad_sub = rospy.Subscriber(self.keypad_topic, std_msgs.msg.UInt8, self._handle_keypad)
        self.__popup_pub = rospy.Publisher(self.popup_topic, agv05_msgs.msg.UIPopup, queue_size=1, latch=True)
        self.__start_led_pub = rospy.Publisher(self.start_led_topic, std_msgs.msg.Bool, queue_size=1, latch=True)
        self.__stop_led_pub = rospy.Publisher(self.stop_led_topic, std_msgs.msg.Bool, queue_size=1, latch=True)
        self.__led_control_user_pub = rospy.Publisher(self.led_control_user_topic, agv05_msgs.msg.LedControl, queue_size=1, latch=True)
        self.__led_control_nav_pub = rospy.Publisher(self.led_control_nav_topic, agv05_msgs.msg.LedControl, queue_size=1, latch=True)

        # clear previous popup
        self.popup_message = None
        self.__popup_pub.publish(agv05_msgs.msg.UIPopup(type=agv05_msgs.msg.UIPopup.POPUP_NONE, message='', timer=-1))

    def wait_button_press(self, timeout=None):
        if not self.in_context:
            self.button_state = [False for i in range(self.MAX_ID)]
            self.prev_button_state = [False for i in range(self.MAX_ID)]
            self.ev.clear()

        if timeout is not None:
            wait_end = rospy.get_time() + timeout

        while self.ev.wait(timeout and (wait_end - rospy.get_time())):
            self.ev.clear()
            if self.button_state[self.STOP_ID] and not self.prev_button_state[self.STOP_ID]:
                self.prev_button_state[self.STOP_ID] = False
                return 'stop_button'

            if self.button_state[self.START_ID] and not self.prev_button_state[self.START_ID]:
                self.prev_button_state[self.START_ID] = False
                return 'start_button'

    def wait_button_release(self, timeout=None):
        if not self.in_context:
            self.button_state = [False for i in range(self.MAX_ID)]
            self.prev_button_state = [False for i in range(self.MAX_ID)]
            self.ev.clear()

        if timeout is not None:
            wait_end = rospy.get_time() + timeout

        while self.ev.wait(timeout and (wait_end - rospy.get_time())):
            self.ev.clear()
            if not self.button_state[self.STOP_ID] and self.prev_button_state[self.STOP_ID]:
                self.prev_button_state[self.STOP_ID] = False
                return 'stop_button'

            if not self.button_state[self.START_ID] and self.prev_button_state[self.START_ID]:
                self.prev_button_state[self.START_ID] = False
                return 'start_button'

    def wait_keypad(self, timeout=None):
        if not self.in_context:
            self.button_state[self.KEYPAD_ID] = False
            self.ev.clear()

        if timeout is not None:
            wait_end = rospy.get_time() + timeout

        while self.ev.wait(timeout and (wait_end - rospy.get_time())):
            self.ev.clear()
            if self.button_state[self.KEYPAD_ID]:
                return 'key_%s' % self.keypad

    def _handle_start_button(self, state):
        self.prev_button_state[self.START_ID] = self.button_state[self.START_ID]
        self.button_state[self.START_ID] = state.data
        if not self.button_state[self.START_ID] and self.prev_button_state[self.START_ID]:
            if not self.in_context:
                for cb in self.__button_release_callbacks:
                    cb('start_button')
        self.ev.set()

    def _handle_stop_button(self, state):
        self.prev_button_state[self.STOP_ID] = self.button_state[self.STOP_ID]
        self.button_state[self.STOP_ID] = state.data
        if not self.button_state[self.STOP_ID] and self.prev_button_state[self.STOP_ID]:
            if not self.in_context:
                for cb in self.__button_release_callbacks:
                    cb('stop_button')
        self.ev.set()

    def _handle_keypad(self, keypad):
        k = chr(keypad.data)
        if k in '0123456789AB':
            self.keypad = k
            self.button_state[self.KEYPAD_ID] = True
            self.ev.set()

    def show_popup_non_interactive(self, message, timer):
        return self.show_popup(agv05_msgs.msg.UIPopup.POPUP_NON_INTERACTIVE, message, timer)

    def show_popup_alert(self, message, timer):
        return self.show_popup(agv05_msgs.msg.UIPopup.POPUP_ALERT, message, timer, start_led=True)

    def show_popup_confirm(self, message, timer):
        return self.show_popup(agv05_msgs.msg.UIPopup.POPUP_CONFIRM, message, timer, start_led=True, stop_led=True)

    def show_popup_keypad(self, message, timer):
        return self.show_popup(agv05_msgs.msg.UIPopup.POPUP_KEYPAD, message, timer)

    @contextlib.contextmanager
    def show_popup(self, type, message, timer, start_led=False, stop_led=False):
        self.popup_message = {
            'message': message,
            'type': type,
        }
        self.__popup_pub.publish(agv05_msgs.msg.UIPopup(type=type, message=six.ensure_str(message), timer=(timer or -1)))
        self.__led_control_user_pub.publish(agv05_msgs.msg.LedControl(mode=agv05_msgs.msg.LedControl.STATION_WAIT))
        self.set_button_led(start=start_led, stop=stop_led)
        self.button_state = [False for i in range(self.MAX_ID)]
        self.prev_button_state = [False for i in range(self.MAX_ID)]
        self.ev.clear()
        self.in_context = True
        yield
        self.in_context = False
        self.popup_message = None
        self.__popup_pub.publish(agv05_msgs.msg.UIPopup(type=agv05_msgs.msg.UIPopup.POPUP_NONE, message='', timer=-1))
        self.__led_control_user_pub.publish(agv05_msgs.msg.LedControl(mode=agv05_msgs.msg.LedControl.OFF))
        self.set_button_led()

    def toggle_led_paused(self, paused):
        self._paused = paused
        if self.in_context or self.robot.robot_control.panel_control.get_safety_message():
            return
        self.__led_control_nav_pub.publish(agv05_msgs.msg.LedControl(
            mode=agv05_msgs.msg.LedControl.PAUSE if paused else agv05_msgs.msg.LedControl.OFF))
        self.set_button_led()

    def set_button_led(self, start=None, stop=None):
        if start is None:
            start = self._paused and self.robot.config.start_button_will_resume_action
        if stop is None:
            stop = self._paused is False and self.robot.config.stop_button_will_pause_action
        self.__start_led_pub.publish(std_msgs.msg.Bool(data=start))
        self.__stop_led_pub.publish(std_msgs.msg.Bool(data=stop))

    def log(self, level, message):
        self.logger.log(level, message)


class Agv05Io(object):
    MAX_PORTS = 8
    MAX_BITS = 16
    input_topic = '/agv05/io/port%s/input'
    output_topic = '/agv05/io/port%s/output'

    def __init__(self, robot):
        self.__inputs = [0] * self.MAX_PORTS
        self.__input_sub = []
        self.__output_pub = []
        for port in range(1, self.MAX_PORTS + 1):
            self.__input_sub.append(rospy.Subscriber(
                self.input_topic % port, std_msgs.msg.UInt16,
                self._handle_input, port, queue_size=1))
            self.__output_pub.append(rospy.Publisher(
                self.output_topic % port, agv05_msgs.msg.OutputPort,
                queue_size=10, latch=False))

    def _handle_input(self, state, port):
        self.__inputs[port - 1] = state.data

    def write_output(self, port, bit, state):
        if port <= 0 or port > self.MAX_PORTS:
            return
        if bit < 0 or bit >= self.MAX_BITS:
            return
        self.__output_pub[port - 1].publish(agv05_msgs.msg.OutputPort(
            1 << bit if state else 0, 1 << bit))

    def write_output_register(self, port, output, output_mask):
        output = output & 0xFFFF  # Ensures output is within the uint16 range
        if port <= 0 or port > self.MAX_PORTS:
            return
        self.__output_pub[port - 1].publish(agv05_msgs.msg.OutputPort(
            output, output_mask))

    def read_input(self, port, bit):
        if port <= 0 or port > self.MAX_PORTS:
            return False
        if bit < 0 or bit >= self.MAX_BITS:
            return False
        return (self.__inputs[port - 1] & 1 << bit) != 0

    def read_input_register(self, port):
        if port <= 0 or port > self.MAX_PORTS:
            return 0
        return self.__inputs[port - 1]


class Agv05RemoteIo(object):
    CYCLE_SIZE = 256
    request_topic = '/agv05/peripheral/remote_io/request'
    reply_topic = '/agv05/peripheral/remote_io/reply'
    communication_timeout_topic = '/agv05/peripheral/remote_io/communication_timeout'

    def __init__(self):
        self.__lock = threading.Lock()
        self.__request_id = 0
        self.__timeout = 3.0

        self.__expires = [0] * self.CYCLE_SIZE
        self.__results = [None] * self.CYCLE_SIZE

        self.__communication_timeout_sub = rospy.Subscriber(
            self.communication_timeout_topic, std_msgs.msg.Float32,
            self._handle_communication_timeout_value, queue_size=1)
        self.__reply_sub = rospy.Subscriber(
            self.reply_topic, agv05_msgs.msg.RemoteIoReply,
            self._handle_reply, queue_size=10)
        self.__request_pub = rospy.Publisher(
            self.request_topic, agv05_msgs.msg.RemoteIoRequest,
            queue_size=1, latch=False)

    def write_output(self, remote_io, data, mask):
        data = data & 0xFFFF  # Ensures data is within the uint16 range
        request_id = self._generate_request_id()
        self.__request_pub.publish(agv05_msgs.msg.RemoteIoRequest(
            request_id=request_id,
            remote_io=remote_io,
            type=agv05_msgs.msg.RemoteIoRequest.TYPE_WRITE,
            data=data,
            data_mask=mask))
        return request_id

    def read_input(self, remote_io):
        request_id = self._generate_request_id()
        self.__request_pub.publish(agv05_msgs.msg.RemoteIoRequest(
            request_id=request_id,
            remote_io=remote_io,
            type=agv05_msgs.msg.RemoteIoRequest.TYPE_READ))
        return request_id

    def get_result(self, request_id):
        if request_id < 1 or request_id >= self.CYCLE_SIZE:
            return False
        with self.__lock:
            if rospy.get_time() > self.__expires[request_id]:
                return False
            return self.__results[request_id]

    def _handle_communication_timeout_value(self, data):
        self.__timeout = data.data

    def _handle_reply(self, msg):
        if msg.request_id < 1 or msg.request_id >= self.CYCLE_SIZE:
            return
        with self.__lock:
            self.__results[msg.request_id] = msg.data

    def _generate_request_id(self):
        with self.__lock:
            self.__request_id += 1
            if self.__request_id >= self.CYCLE_SIZE:
                self.__request_id = 1
            self.__expires[self.__request_id] = rospy.get_time() + self.__timeout + 0.1
            self.__results[self.__request_id] = None
            return self.__request_id


class Agv05Modbus(object):
    CYCLE_SIZE = 256
    request_topic = '/agv05/peripheral/modbus/request'
    reply_topic = '/agv05/peripheral/modbus/reply'
    communication_timeout_topic = '/agv05/peripheral/modbus/communication_timeout'

    def __init__(self):
        self.__lock = threading.Lock()
        self.__request_id = 0
        self.__timeout = 3.0

        self.__expires = [0] * self.CYCLE_SIZE
        self.__results = [None] * self.CYCLE_SIZE

        self.__communication_timeout_sub = rospy.Subscriber(
            self.communication_timeout_topic, std_msgs.msg.Float32,
            self._handle_communication_timeout_value, queue_size=1)
        self.__reply_sub = rospy.Subscriber(
            self.reply_topic, agv05_msgs.msg.ModbusReply,
            self._handle_reply, queue_size=10)
        self.__request_pub = rospy.Publisher(
            self.request_topic, agv05_msgs.msg.ModbusRequest,
            queue_size=1, latch=False)

    def read(self, modbus, type, address):
        request_id = self._generate_request_id()
        self.__request_pub.publish(agv05_msgs.msg.ModbusRequest(
            request_id=request_id,
            modbus=modbus,
            type=type,
            address=address))
        return request_id

    def write(self, modbus, type, address, data, data_mask=65535):
        data = data & 0xFFFF  # Ensures data is within the uint16 range
        request_id = self._generate_request_id()
        self.__request_pub.publish(agv05_msgs.msg.ModbusRequest(
            request_id=request_id,
            modbus=modbus,
            type=agv05_msgs.msg.ModbusRequest.TYPE_WRITE_COIL + type,
            address=address,
            data=data,
            data_mask=data_mask))
        return request_id

    def get_result(self, request_id):
        if request_id < 1 or request_id >= self.CYCLE_SIZE:
            return False
        with self.__lock:
            if rospy.get_time() > self.__expires[request_id]:
                return False
            return self.__results[request_id]

    def _handle_communication_timeout_value(self, data):
        self.__timeout = data.data

    def _handle_reply(self, msg):
        if msg.request_id < 1 or msg.request_id >= self.CYCLE_SIZE:
            return
        with self.__lock:
            self.__results[msg.request_id] = msg.data

    def _generate_request_id(self):
        with self.__lock:
            self.__request_id += 1
            if self.__request_id >= self.CYCLE_SIZE:
                self.__request_id = 1
            self.__expires[self.__request_id] = rospy.get_time() + self.__timeout + 0.1
            self.__results[self.__request_id] = None
            return self.__request_id


class Agv05Power(object):
    battery_state_topic = '/agv05/power/battery_state'
    enable_charger_topic = '/agv05/power/auto_charging_trigger'
    recalibrate_battery_topic = '/agv05/power/battery_percentage_reset'
    low_battery_level_param = '/agv05_power/low_battery_level'

    def __init__(self, robot):
        self.robot = robot
        self.charging_status = False
        self.battery_percentage = 0

        self.__battery_state_sub = rospy.Subscriber(
            self.battery_state_topic, agv05_msgs.msg.BatteryState, self._handle_battery_state, queue_size=1)
        self.__enable_charger_pub = rospy.Publisher(self.enable_charger_topic, std_msgs.msg.Bool, queue_size=1)
        self.__recalibrate_battery_pub = rospy.Publisher(
            self.recalibrate_battery_topic, std_msgs.msg.Bool, queue_size=1)

    def start(self):
        rospy.set_param(self.low_battery_level_param, self.robot.models.min_battery_level if
            self.robot.models.min_battery_level is not None else 30)

    def _handle_battery_state(self, data):
        self.charging_status = data.state in [
            agv05_msgs.msg.BatteryState.AUTO_CHARGING,
            agv05_msgs.msg.BatteryState.MANUAL_CHARGING
        ] if data.state not in [
            agv05_msgs.msg.BatteryState.AUTO_CHARGER_CONNECTION_FAILED,
            agv05_msgs.msg.BatteryState.AUTO_CHARGER_DISENGAGING
        ] else None
        self.battery_percentage = data.percentage

    def enable_charging(self, state):
        self.__enable_charger_pub.publish(std_msgs.msg.Bool(data=state))

    def recalibrate_battery(self):
        self.__recalibrate_battery_pub.publish(std_msgs.msg.Bool(data=True))


class Agv05Audio(object):
    AC = agv05_msgs.msg.AudioControl
    music_topic = '/agv05/audio/music_control'
    alarm_topic = '/agv05/audio/alarm_control'

    def __init__(self):
        self.__music_pub = rospy.Publisher(self.music_topic, self.AC, queue_size=1)
        self.__alarm_pub = rospy.Publisher(self.alarm_topic, self.AC, queue_size=1)

    def play_music(self, playlist):
        self.__music_pub.publish(self.AC(operation=self.AC.PLAY, playlist=playlist))

    def pause_music(self):
        self.__music_pub.publish(self.AC(operation=self.AC.PAUSE))

    def stop_music(self):
        self.__music_pub.publish(self.AC(operation=self.AC.STOP))

    def play_alarm(self):
        self.__alarm_pub.publish(self.AC(operation=self.AC.PLAY))

    def stop_alarm(self):
        self.__alarm_pub.publish(self.AC(operation=self.AC.STOP))


class Agv05Rfid(object):
    rfid_front_topic = '/agv05/rfid/rfid_front'
    rfid_rear_topic = '/agv05/rfid/rfid_rear'
    front_rfid = ''
    rear_rfid = ''

    def __init__(self):
        self.__rfid_read_front_sub = rospy.Subscriber(
            self.rfid_front_topic, std_msgs.msg.String,
            self._handle_front_rfid, queue_size=1)
        self.__rfid_read_rear_sub = rospy.Subscriber(
            self.rfid_rear_topic, std_msgs.msg.String,
            self._handle_rear_rfid, queue_size=1)

        self.__front_rfid_pub = rospy.Publisher(self.rfid_front_topic, std_msgs.msg.String, queue_size=1)
        self.__rear_rfid_pub = rospy.Publisher(self.rfid_rear_topic, std_msgs.msg.String, queue_size=1)

    def _handle_front_rfid(self, data):
        self.front_rfid = six.ensure_str(data.data).strip('\0').strip()

    def _handle_rear_rfid(self, data):
        self.rear_rfid = six.ensure_str(data.data).strip('\0').strip()

    def _reset_rfid(self, rfid_location):
        if rfid_location == 0:
            self.__front_rfid_pub.publish('')
        elif rfid_location == 1:
            self.__rear_rfid_pub.publish('')


class Agv05(CapabilityLauncherMixin, Robot):
    name = 'agv05'
    io_cls = Agv05Io
    panel_cls = Agv05Panel
    power_cls = Agv05Power
    simulated = False

    def __init__(self):
        super(Agv05, self).__init__()
        self.audio = Agv05Audio()
        self.remote_io = Agv05RemoteIo()
        self.modbus = Agv05Modbus()
        self.rfid = Agv05Rfid()
        self.__mode = 0

        self.__get_running_capabilities = rospy.ServiceProxy('/capability_server/get_running_capabilities', capabilities.srv.GetRunningCapabilities, persistent=False)

    def start(self, mode):
        if self.is_alive():
            rospy.logwarn('Robot is already started.')
            return

        super(Agv05, self).start()
        self.power.start()

        for i in range(5):
            rospy.sleep(1.0)
            if rospy.get_param('/robot_description', None):
                break
        else:
            try:
                self.stop()
            except Exception:
                pass
            raise RuntimeError('Timeout while waiting for robot description parameter.')

        try:
            self.start_agv05_bringup()
        except Exception:
            pass

        self.__mode = mode

        # Wait for components to start
        if not self.base.wait_for_server(rospy.Duration(50.0)):
            try:
                self.stop()
            except Exception:
                pass
            raise RuntimeError('Timeout while waiting for backend components to start.')

    def stop(self):
        try:
            self.stop_agv05_bringup()
        except Exception:
            pass

        super(Agv05, self).stop()
        self.base.clear_dimension_profile()
        self.__mode = 0

        # Wait for components to stop
        for i in range(20):
            caps = self.__get_running_capabilities(include_stopping=True)
            if not caps.running_capabilities:
                break
            time.sleep(1.0)  # use wall-time because the simulation clock would have stopped
        time.sleep(1.0)  # allow some time for capability server to settle down

    def get_mode(self):
        return self.__mode

    def start_agv05_bringup(self):
        self.add_capability('agv05_capabilities/Agv05Bringup', None)

    def stop_agv05_bringup(self):
        self.remove_capability('agv05_capabilities/Agv05Bringup')
