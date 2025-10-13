from __future__ import absolute_import
from __future__ import unicode_literals

from capabilities.client import CapabilitiesClient, \
    CapabilityNotRunningException
import rospy
import threading

from .base import Base


class Robot(object):
    """
    Interface that wraps the low level stuff of a robot.
    """

    name = 'robot'
    base_cls = Base
    io_cls = None
    panel_cls = None
    power_cls = None
    simulated = False

    def __init__(self):
        self.lock = threading.Lock()
        self.__rsv = None

        self.base = self.get_base_cls()(self)
        try:
            self.io = self.get_io_cls()(self)
        except Exception:
            pass
        try:
            self.panel = self.get_panel_cls()(self)
        except Exception:
            pass
        try:
            self.power = self.get_power_cls()(self)
        except Exception:
            pass

    # running status
    # --------------
    def start(self):
        raise NotImplementedError()

    def stop(self):
        raise NotImplementedError()

    def is_alive(self):
        raise NotImplementedError()

    # reservation management
    # ----------------------
    def reserve(self, module_id):
        with self.lock:
            if self.__rsv is None:
                self.__rsv = module_id
                return True
            else:
                return False

    def unreserve(self, module_id):
        with self.lock:
            if self.__rsv == module_id:
                self.__rsv = None
                return True
            else:
                return False

    def get_reservation(self):
        return self.__rsv

    def is_reserved(self):
        return self.__rsv is not None

    # helper functions
    # ----------------
    def get_base_cls(self):
        if not self.base_cls:
            raise RuntimeError('Robot\'s base_cls is not defined.')
        return self.base_cls

    def get_io_cls(self):
        if not self.io_cls:
            raise RuntimeError('Robot\'s io_cls is not defined.')
        return self.io_cls

    def get_panel_cls(self):
        if not self.panel_cls:
            raise RuntimeError('Robot\'s panel_cls is not defined.')
        return self.panel_cls

    def get_power_cls(self):
        if not self.power_cls:
            raise RuntimeError('Robot\'s power_cls is not defined.')
        return self.power_cls


class CapabilityLauncherMixin(object):
    capability_interface = 'agv05_capabilities/MobileRobot'
    capability_provider = None  # use default provider set in launch file

    def __init__(self):
        super(CapabilityLauncherMixin, self).__init__()
        self.__cpb_client = None

    def start(self):
        with self.lock:
            if self.__cpb_client:
                rospy.logwarn('Robot is already started.')
                return True

            try:
                self.__cpb_client = CapabilitiesClient('/capability_server')
                self.__cpb_client.use_capability(self.get_capability_interface(), self.get_capability_provider())
                return True
            except Exception as ex:
                self.__cpb_client.shutdown()
                self.__cpb_client = None
                rospy.logerr('Failed to start robot using capability server: %s', ex)
                raise

    def stop(self):
        with self.lock:
            if not self.__cpb_client:
                rospy.logwarn('Robot has not started yet.')
                return True

            try:
                try:
                    self.__cpb_client.free_capability(self.get_capability_interface())
                except CapabilityNotRunningException:
                    pass
                self.__cpb_client.shutdown()
                self.__cpb_client = None
                return True
            except Exception as ex:
                rospy.logerr('Failed to stop robot using capability server: %s', ex)
                raise

    def is_alive(self):
        # Todo: Query the capability server for status in case the capability has terminated abruptly.
        return self.__cpb_client is not None

    def add_capability(self, interface, preferred_provider=None):
        with self.lock:
            try:
                if not self.__cpb_client:
                    raise RuntimeError('Robot has not started yet.')
                self.__cpb_client.use_capability(interface, preferred_provider)
                return True
            except Exception as ex:
                rospy.logerr('Failed to add capability: %s', ex)
                raise

    def remove_capability(self, interface):
        with self.lock:
            try:
                if not self.__cpb_client:
                    raise RuntimeError('Robot has not started yet.')
                self.__cpb_client.free_capability(interface)
                return True
            except Exception as ex:
                rospy.logerr('Failed to remove capability: %s', ex)
                raise

    def get_capability_interface(self):
        if not self.capability_interface:
            raise ValueError('Capability interface is not specified.')
        return self.capability_interface

    def get_capability_provider(self):
        return self.capability_provider


class TrackedMixin(object):
    trackless = False


class TracklessMixin(object):
    trackless = True

    def start_amcl(self):
        raise NotImplementedError()

    def stop_amcl(self):
        raise NotImplementedError()

    def start_slam(self):
        raise NotImplementedError()

    def stop_slam(self):
        raise NotImplementedError()
