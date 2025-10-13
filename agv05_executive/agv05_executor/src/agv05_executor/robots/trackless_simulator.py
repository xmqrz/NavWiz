from __future__ import absolute_import
from __future__ import unicode_literals

import agv05_msgs.msg
import rospy
import std_msgs.msg
import threading

from .sim import SimIo, SimPower
from .trackless_agv05 import TracklessAgv05


SAFETY_HEARTBEAT_TIMEOUT = 0.5


class TracklessSimulator(TracklessAgv05):
    name = 'trackless_simulator'
    io_cls = SimIo
    power_cls = SimPower
    simulated = True

    capability_interface = 'agv05_capabilities/SimulatorMobileRobot'

    def start(self, mode):
        super(TracklessSimulator, self).start(mode)

        # emulate safety inputs for nav
        self.__nav_enable_pub = rospy.Publisher('/agv05/motor/navigation_enable', std_msgs.msg.Bool, latch=True, queue_size=1)
        self.__steering_align_pub = rospy.Publisher('/agv05/motor/steering_align', std_msgs.msg.Bool, latch=True, queue_size=1)
        self.__safety_triggers_pub = rospy.Publisher('/agv05/safety/safety_triggers', agv05_msgs.msg.SafetyTriggers, latch=True, queue_size=1)

        self.__nav_enable_pub.publish(True)
        self.__steering_align_pub.publish(True)
        self.__safety_triggers_pub.publish()

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run)
        self._thread.start()

    def _run(self):
        while not self._stop_event.wait(SAFETY_HEARTBEAT_TIMEOUT):
            self.__safety_triggers_pub.publish()

    def stop(self):
        self._stop_event.set()
        self._thread.join()

        return super(TracklessSimulator, self).stop()
