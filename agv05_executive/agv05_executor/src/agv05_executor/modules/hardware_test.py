from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_msgs.msg import BatteryState, LedControl, LineSensor, \
    LineSensorActivation, MotorControl, MotorFeedback, MsbRaw, \
    ObstacleSensor, ObstacleSensorArea, \
    OutputPort, PowerSource, SafetyTriggers
from agv05_webserver.system.models import Variable, db_auto_reconnect
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, String, UInt16
from wraptor.decorators import memoize, throttle
import dynamic_reconfigure.client
import rospy
import tf
import ujson as json

from .module import Module


class Io(object):

    @db_auto_reconnect()
    def start(self):
        self.inputs = [0] * 8
        self.outputs = [0] * 8
        self.io_name = {}

        try:
            self.io_name = json.loads(Variable.objects.get(name=Variable.IO_NAME).value)
        except Exception:
            pass

        # ROS publishers
        self.__pubs = [rospy.Publisher("agv05/io/port%d/output" % (i + 1), OutputPort, queue_size=1) for i in range(8)]

        # ROS subscribers
        self.__subs = [rospy.Subscriber("agv05/io/port%d/input" % (i + 1), UInt16, self.handle_input, i, queue_size=1) for i in range(8)]
        self.__subs.extend([rospy.Subscriber("agv05/io/port%d/output" % (i + 1), OutputPort, self.handle_output, i, queue_size=10) for i in range(8)])

        # clear outputs
        for i in range(8):
            self.__pubs[i].publish(OutputPort(0, 0xffff))

    def reset(self):
        # clear outputs
        for i in range(8):
            self.__pubs[i].publish(OutputPort(0, 0xffff))

    def stop(self):
        for pub in self.__pubs:
            pub.unregister()
        for sub in self.__subs:
            sub.unregister()

    def data(self):
        return {
            'inputs': self.inputs,
            'outputs': self.outputs,
            'io_name': self.io_name,
        }

    def handle_in_pipe(self, data):
        if data['command'] == 'output':
            i = data['port'] - 1
            self.outputs[i] = data['output']
            self.__pubs[i].publish(OutputPort(data['output'], 0xffff))

    def handle_input(self, msg, i):
        self.inputs[i] = msg.data

    def handle_output(self, msg, i):
        self.outputs[i] = self.outputs[i] & ~msg.output_mask | msg.output & msg.output_mask


class Laser(object):

    def start(self):
        self.activation = ObstacleSensor()

        # ROS publisher
        self.__pub = rospy.Publisher('agv05/obstacle_sensor/area', ObstacleSensorArea, queue_size=1)

        # ROS subscriber
        self.__sub = rospy.Subscriber('agv05/obstacle_sensor/activation', ObstacleSensor, self.handle_activation, queue_size=1)

    def reset(self):
        # clear area
        self.__pub.publish(ObstacleSensorArea())

    def stop(self):
        self.__pub.unregister()
        self.__sub.unregister()

    def data(self):
        return {
            'far_blocked': self.activation.far_blocked,
            'middle_blocked': self.activation.middle_blocked,
            'near_blocked': self.activation.near_blocked,
            'malfunction': self.activation.malfunction,
            'hint': self.activation.hint,
        }

    def handle_in_pipe(self, data):
        if data['command'] == 'area':
            self.__pub.publish(ObstacleSensorArea(data['profile'], data['area']))

    def handle_activation(self, msg):
        self.activation = msg


class Led(object):

    def start(self):
        # ROS publisher
        self.__pub = rospy.Publisher('agv05/led/control/user', LedControl, queue_size=1)

    def reset(self):
        # clear led
        self.__pub.publish(LedControl())

    def stop(self):
        self.__pub.unregister()

    def data(self):
        return {}

    def handle_in_pipe(self, data):
        if data['command'] == 'mode':
            self.__pub.publish(LedControl(data['mode']))


class Line(object):

    def start(self):
        self.front_sensor = LineSensor()
        self.rear_sensor = LineSensor()
        self.front_activation = []
        self.rear_activation = []
        self.front_raw = []
        self.rear_raw = []

        # ROS subscribers
        self.__subs = [
            rospy.Subscriber('agv05/line_sensor/front_sensor', LineSensor, self.handle_front_sensor, queue_size=1),
            rospy.Subscriber('agv05/line_sensor/rear_sensor', LineSensor, self.handle_rear_sensor, queue_size=1),
            rospy.Subscriber('agv05/line_sensor/front_activation', LineSensorActivation, self.handle_front_activation, queue_size=1),
            rospy.Subscriber('agv05/line_sensor/rear_activation', LineSensorActivation, self.handle_rear_activation, queue_size=1),
            rospy.Subscriber('agv05/msb/msb_front_raw', MsbRaw, self.handle_front_raw, queue_size=1),
            rospy.Subscriber('agv05/msb/msb_rear_raw', MsbRaw, self.handle_rear_raw, queue_size=1),
        ]

    def reset(self):
        pass

    def stop(self):
        for sub in self.__subs:
            sub.unregister()

    def data(self):
        return {
            'front_enable': self.front_sensor.enable,
            'front_error': self.front_sensor.sensor_error,
            'front_linear_error': self.front_sensor.linear_error,
            'front_angular_error': self.front_sensor.angular_error,
            'front_out_of_line': self.front_sensor.out_of_line,
            'front_on_junction': self.front_sensor.on_junction,
            'front_raw': self.front_raw,
            'front_activation': self.front_activation,

            'rear_enable': self.rear_sensor.enable,
            'rear_error': self.rear_sensor.sensor_error,
            'rear_linear_error': self.rear_sensor.linear_error,
            'rear_angular_error': self.rear_sensor.angular_error,
            'rear_out_of_line': self.rear_sensor.out_of_line,
            'rear_on_junction': self.rear_sensor.on_junction,
            'rear_raw': self.rear_raw,
            'rear_activation': self.rear_activation,
        }

    @throttle(0.1, instance_method=True)
    def handle_front_sensor(self, msg):
        self.front_sensor = msg

    @throttle(0.1, instance_method=True)
    def handle_rear_sensor(self, msg):
        self.rear_sensor = msg

    @throttle(0.1, instance_method=True)
    def handle_front_activation(self, msg):
        self.front_activation = msg.activation

    @throttle(0.1, instance_method=True)
    def handle_rear_activation(self, msg):
        self.rear_activation = msg.activation

    @throttle(0.1, instance_method=True)
    def handle_front_raw(self, msg):
        self.front_raw = msg.raw

    @throttle(0.1, instance_method=True)
    def handle_rear_raw(self, msg):
        self.rear_raw = msg.raw


class Motor(object):

    def start(self):
        self.feedback = MotorFeedback()
        self.straight_distance = 0
        self.rotational_distance = 0
        self.track_width = 0.5

        # ROS publishers
        self.__pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # ROS subscribers
        self.__subs = [
            rospy.Subscriber('agv05/motor/feedback', MotorFeedback, self.handle_feedback, queue_size=1),
            rospy.Subscriber('agv05/motor/straight_distance', Float64, self.handle_straight_distance, queue_size=1),
            rospy.Subscriber('agv05/motor/rotational_distance', Float64, self.handle_rotational_distance, queue_size=1),
        ]

        # Dyncfg
        try:
            client = dynamic_reconfigure.client.Client('/agv05_motor', timeout=0.5)
            config = client.get_configuration(timeout=0.5)
            if config:
                self.track_width = config.track_width
        except Exception:
            pass

    def reset(self):
        # stop motor
        self.__pub.publish(Twist())

    def stop(self):
        self.__pub.unregister()
        for sub in self.__subs:
            sub.unregister()

    def data(self):
        return {
            'left_distance': self.feedback.left_distance,
            'right_distance': self.feedback.right_distance,
            'left_current': self.feedback.left_current,
            'right_current': self.feedback.right_current,
            'straight_distance': self.straight_distance,
            'rotational_distance': self.rotational_distance,
        }

    def handle_in_pipe(self, data):
        if data['command'] == 'speed':
            left_speed = data['left_speed']
            right_speed = data['right_speed']

            msg = Twist()
            msg.linear.x = (left_speed + right_speed) / 2.0
            msg.angular.z = (right_speed - left_speed) / self.track_width
            self.__pub.publish(msg)

    @throttle(0.1, instance_method=True)
    def handle_feedback(self, msg):
        self.feedback = msg

    @throttle(0.1, instance_method=True)
    def handle_straight_distance(self, msg):
        self.straight_distance = msg.data

    @throttle(0.1, instance_method=True)
    def handle_rotational_distance(self, msg):
        self.rotational_distance = msg.data


class Panel(object):

    def start(self):
        self.button_mode = False
        self.button_power = False
        self.button_start = False
        self.button_stop = False
        self.button_unbrake = False
        self.leds = ['led_low_batt', 'led_mode', 'led_power', 'led_start', 'led_stop', 'led_unbrake']
        self.led_states = [Bool() for i in range(len(self.leds))]

        # ROS publishers
        self.__pubs = [rospy.Publisher('agv05/panel/%s' % led, Bool, queue_size=1) for led in self.leds]

        # ROS subscribers
        self.__subs = [
            rospy.Subscriber('agv05/panel/control/button_mode', Bool, self.handle_button_mode, queue_size=1),
            rospy.Subscriber('agv05/panel/control/button_power', Bool, self.handle_button_power, queue_size=1),
            rospy.Subscriber('agv05/panel/control/button_start', Bool, self.handle_button_start, queue_size=1),
            rospy.Subscriber('agv05/panel/control/button_stop', Bool, self.handle_button_stop, queue_size=1),
            rospy.Subscriber('agv05/panel/control/button_unbrake', Bool, self.handle_button_unbrake, queue_size=1),
        ]

    def reset(self):
        # clear panel leds
        msg = Bool()
        for pub in self.__pubs:
            pub.publish(msg)

    def stop(self):
        for pub in self.__pubs:
            pub.unregister()
        for sub in self.__subs:
            sub.unregister()

    def data(self):
        for i, pub in enumerate(self.__pubs):
            pub.publish(self.led_states[i])

        return {
            'button_mode': self.button_mode,
            'button_power': self.button_power,
            'button_start': self.button_start,
            'button_stop': self.button_stop,
            'button_unbrake': self.button_unbrake,
        }

    def handle_in_pipe(self, data):
        if data['command'] in self.leds:
            i = self.leds.index(data['command'])
            self.led_states[i].data = data['state']

    def handle_button_mode(self, msg):
        self.button_mode = msg.data

    def handle_button_power(self, msg):
        self.button_power = msg.data

    def handle_button_start(self, msg):
        self.button_start = msg.data

    def handle_button_stop(self, msg):
        self.button_stop = msg.data

    def handle_button_unbrake(self, msg):
        self.button_unbrake = msg.data


class Power(object):

    def start(self):
        self.battery_state = BatteryState()
        self.source = PowerSource()

        # ROS subscribers
        self.__subs = [
            rospy.Subscriber('agv05/power/battery_state', BatteryState, self.handle_battery_state, queue_size=1),
            rospy.Subscriber('agv05/power/source', PowerSource, self.handle_source, queue_size=1),
        ]

    def reset(self):
        pass

    def stop(self):
        for sub in self.__subs:
            sub.unregister()

    def data(self):
        state = self.battery_state.state
        if state == BatteryState.IDLE:
            charging = 'Idle'
        elif state == BatteryState.AUTO_CHARGER_ENGAGING:
            charging = 'Auto Charger Engaging'
        elif state == BatteryState.AUTO_CHARGING:
            charging = 'Auto Charging'
        elif state == BatteryState.AUTO_CHARGER_CONNECTION_FAILED:
            charging = 'Auto Charger Connection Failed'
        elif state == BatteryState.AUTO_CHARGING_COMPLETED:
            charging = 'Auto Charging Completed'
        elif state == BatteryState.AUTO_CHARGER_DISENGAGING:
            charging = 'Auto Charger Disengaging'
        elif state == BatteryState.MANUAL_CHARGING:
            charging = 'Manual Charging'
        elif state == BatteryState.BATTERY_ERROR:
            charging = 'Battery Error'
        else:
            charging = 'Unknown %d' % state

        return {
            'charging': charging,
            'battery1_voltage': self.source.battery1_voltage,
            'battery2_voltage': self.source.battery2_voltage,
            'battery_current': self.source.battery_current,
            'auto_charger_voltage': self.source.auto_charger_voltage,
            'manual_charger_voltage': self.source.manual_charger_voltage,
            'manual_charger_connected': self.source.manual_charger_connected,
        }

    def handle_battery_state(self, msg):
        self.battery_state = msg

    @throttle(0.1, instance_method=True)
    def handle_source(self, msg):
        self.source = msg


class Rfid(object):

    def start(self):
        self.front_rfid = ''
        self.rear_rfid = ''

        # ROS subscribers
        self.__subs = [
            rospy.Subscriber('agv05/rfid/rfid_front', String, self.handle_front_rfid, queue_size=1),
            rospy.Subscriber('agv05/rfid/rfid_rear', String, self.handle_rear_rfid, queue_size=1),
        ]

    def reset(self):
        pass

    def stop(self):
        for sub in self.__subs:
            sub.unregister()

    def data(self):
        return {
            'front_rfid': self.front_rfid,
            'rear_rfid': self.rear_rfid,
        }

    def handle_front_rfid(self, msg):
        self.front_rfid = msg.data

    def handle_rear_rfid(self, msg):
        self.rear_rfid = msg.data


class Safety(object):

    def start(self):
        self.safety_triggers = SafetyTriggers()

        # ROS subcriber
        self.__sub = rospy.Subscriber("agv05/safety/safety_triggers", SafetyTriggers, self.handle_safety_triggers, queue_size=1)

    def reset(self):
        pass

    def stop(self):
        self.__sub.unregister()

    def data(self):
        return {
            'bumper_front': self.safety_triggers.bumper_front,
            'bumper_rear': self.safety_triggers.bumper_rear,
            'emergency_button': self.safety_triggers.emergency_button,
            'safety_external1': self.safety_triggers.safety_in_1,
            'safety_external2': self.safety_triggers.safety_in_2,
            'motor_fault': self.safety_triggers.motor_fault,
            'wheel_slippage': self.safety_triggers.wheel_slippage,
            'charger_connected': self.safety_triggers.charger_connected,
            'system_error': self.safety_triggers.system_error,
            'nav_trigger': self.safety_triggers.nav_trigger,
        }

    def handle_safety_triggers(self, msg):
        self.safety_triggers = msg


class Yaw(object):

    def start(self):
        self.__flags = {
            'odom': False,
            'wheel': False,
        }
        self.__yaw = {
            'odom': 0,
            'wheel': 0,
            'imu': 0,
        }

        # ROS publishers
        self.__pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # ROS subscribers
        self.__subs = [
            rospy.Subscriber('odom', Odometry, self.handle_odom, 'odom', queue_size=1),
            rospy.Subscriber('odom/wheel', Odometry, self.handle_odom, 'wheel', queue_size=1),
            rospy.Subscriber('imu/data', Imu, self.handle_imu_data, queue_size=1),
        ]

        self.__scan_topics = []
        nav2d_scan_topics = rospy.get_param('nav2d_scan_topics', '')
        if nav2d_scan_topics:
            self.__scan_topics = nav2d_scan_topics.split()
            for topic in self.__scan_topics:
                self.__flags[topic] = False
                self.__yaw[topic] = 0
                self.__subs.append(rospy.Subscriber('odom/' + topic, Odometry, self.handle_odom, topic, queue_size=1))
            self.__flags['amcl'] = False
            self.__yaw['amcl'] = 0
            self.__subs.append(rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.handle_odom, 'amcl', queue_size=1))

    def reset(self):
        # stop motor
        self.__pub.publish(Twist())

    def stop(self):
        self.__pub.unregister()
        for sub in self.__subs:
            sub.unregister()

    def data(self):
        return ([
            ['AMCL', self.__yaw['amcl']]
        ] if 'amcl' in self.__yaw else []) + [
            ['Odom', self.__yaw['odom']],
            ['Wheel', self.__yaw['wheel']],
        ] + [
            [topic.replace('/', ' ').title(), self.__yaw[topic]] for topic in self.__scan_topics
        ] + [
            ['IMU', self.__yaw['imu']]
        ]

    def handle_in_pipe(self, data):
        if data['command'] == 'speed':
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = data['yaw_speed']
            self.__pub.publish(msg)

    def _get_yaw(self, orientation):
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        return rpy[2]

    @memoize(timeout=0.1, instance_method=True)
    def _update_flag(self, topic):
        self.__flags[topic] = True

    def handle_odom(self, msg, topic):
        self._update_flag(topic)
        if self.__flags[topic]:
            self.__flags[topic] = False
            self.__yaw[topic] = self._get_yaw(msg.pose.pose.orientation)

    @throttle(0.1, instance_method=True)
    def handle_imu_data(self, msg):
        self.__yaw['imu'] = self._get_yaw(msg.orientation)


class HardwareTest(Module):
    id = 'hardware-test'

    def __init__(self, *args, **kwargs):
        super(HardwareTest, self).__init__(*args, **kwargs)
        self.io = Io()
        self.laser = Laser()
        self.led = Led()
        self.line = Line()
        self.motor = Motor()
        self.panel = Panel()
        self.power = Power()
        self.rfid = Rfid()
        self.safety = Safety()
        self.yaw = Yaw()
        self.timer = None

    def start(self):
        self.io.start()
        self.laser.start()
        self.led.start()
        self.line.start()
        self.motor.start()
        self.panel.start()
        self.power.start()
        self.rfid.start()
        self.safety.start()
        self.yaw.start()
        self.timer = rospy.Timer(rospy.Duration(0.2), self.handle_timer)

    def stop(self):
        if self.timer:
            self.timer.shutdown()
            self.timer = None
        self.io.reset()
        self.laser.reset()
        self.led.reset()
        self.line.reset()
        self.motor.reset()
        self.panel.reset()
        self.power.reset()
        self.rfid.reset()
        self.safety.reset()
        self.yaw.reset()

        rospy.sleep(1.0)

        self.io.stop()
        self.laser.stop()
        self.led.stop()
        self.line.stop()
        self.motor.stop()
        self.panel.stop()
        self.power.stop()
        self.rfid.stop()
        self.safety.stop()
        self.yaw.stop()

    def handle_in_pipe(self, data):
        try:
            if data['id'] == 'io':
                self.io.handle_in_pipe(data['data'])
            elif data['id'] == 'laser':
                self.laser.handle_in_pipe(data['data'])
            elif data['id'] == 'led':
                self.led.handle_in_pipe(data['data'])
            elif data['id'] == 'motor':
                self.motor.handle_in_pipe(data['data'])
            elif data['id'] == 'panel':
                self.panel.handle_in_pipe(data['data'])
            elif data['id'] == 'yaw':
                self.yaw.handle_in_pipe(data['data'])
        except KeyError as ex:
            rospy.logerr('[HardwareTest] invalid input: %s', ex)
        except Exception as ex:
            rospy.logerr('Exception: %s', ex)

    def handle_timer(self, timer_event):
        data = {
            'io': self.io.data(),
            'laser': self.laser.data(),
            'led': self.led.data(),
            'line': self.line.data(),
            'motor': self.motor.data(),
            'panel': self.panel.data(),
            'power': self.power.data(),
            'rfid': self.rfid.data(),
            'safety': self.safety.data(),
            'yaw': self.yaw.data(),
        }
        self.out(data)
