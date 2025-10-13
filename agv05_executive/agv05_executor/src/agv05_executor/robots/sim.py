from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Direction
import rospy
import six

from .agv05 import Agv05Audio, Agv05Modbus, Agv05Panel, Agv05Power, Agv05RemoteIo
from .robot import Robot


class SimIo(object):
    # Todo: do something controllable by tester.

    def __init__(self, robot):
        pass

    def write_output(self, port, bit, state):
        pass

    def write_output_register(self, port, output, output_mask):
        pass

    def read_input(self, port, bit):
        return False

    def read_input_register(self, port):
        return 0


class SimPower(Agv05Power):

    def __init__(self, robot):
        super(SimPower, self).__init__(robot)
        self.battery_percentage = 88

    def enable_charging(self, state):
        if state:
            rospy.loginfo('SimPower is enabling charging.')
        else:
            rospy.loginfo('SimPower is disabling charging.')
        self.charging_status = state

    def recalibrate_battery(self):
        rospy.loginfo('SimPower is recalibrating battery.')


class SimRfid(object):

    def __init__(self, robot):
        self.robot = robot

    def _get_rfid(self, j, h):
        rfid = ''
        for k, v in six.iteritems(self.robot.models.rfid_assoc):
            if v == (j, Direction.NA):
                rfid = k
                break
        if rfid:
            for k, v in six.iteritems(self.robot.models.rfid_suffix_assoc):
                if v == h:
                    rfid = '%s:%s' % (rfid, k)
                    break
        return rfid

    @property
    def front_rfid(self):
        if self.robot.trackless:
            return ''
        j, h = self.robot.base.get_location()
        return self._get_rfid(j, h)

    @property
    def rear_rfid(self):
        if self.robot.trackless:
            return ''
        j, h = self.robot.base.get_location()
        if h < Direction.NA:
            h = (h + Direction.NA / 2) % Direction.NA
        return self._get_rfid(j, h)

    def _reset_rfid(self, rfid_location):
        pass


class Sim(Robot):
    name = 'sim'
    io_cls = SimIo
    panel_cls = Agv05Panel  # Stealing form agv05
    power_cls = SimPower
    simulated = True

    def __init__(self):
        super(Sim, self).__init__()
        self.audio = Agv05Audio()  # Stealing from agv05
        self.remote_io = Agv05RemoteIo()  # Stealing from agv05
        self.modbus = Agv05Modbus()  # Stealing from agv05
        self.rfid = SimRfid(self)
        self.__started = False
        self.__mode = 0

    def start(self, mode):
        self.__started = True
        self.__mode = mode
        rospy.set_param('/primary_obstacle_scan_topics/scan', 'Sim Laser')

    def stop(self):
        self.base.reset_pose()
        self.base.clear_dimension_profile()
        self.__started = False
        self.__mode = 0

    def is_alive(self):
        return self.__started

    def get_mode(self):
        return self.__mode
