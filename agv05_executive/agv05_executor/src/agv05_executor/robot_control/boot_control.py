from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.app.serializers import BootSerializer
import agv05_executive_msgs.srv
import os
import rospy
import std_msgs.msg
import std_srvs.srv
import subprocess

from ..robots.robot import CapabilityLauncherMixin


class BootControl(object):

    def __init__(self, robot):
        self.robot = robot
        self.shutdown_success = False
        self.__shutdown_service = rospy.Service('~shutdown', std_srvs.srv.Trigger, self.handle_shutdown)
        self.__shutdown_success_sub = rospy.Subscriber('/shutdown_success', std_msgs.msg.Bool, self.handle_shutdown_success)

    def start(self):
        pass

    def stop(self):
        pass

    def handle_boot_control(self, data):
        try:
            serializer = BootSerializer(data=data)
            serializer.is_valid(raise_exception=True)
        except Exception:
            return

        operation = serializer.validated_data['operation']

        if operation in [BootSerializer.Operation.START_ROBOT,
                BootSerializer.Operation.START_ROBOT_2,
                BootSerializer.Operation.START_ROBOT_3,
                BootSerializer.Operation.STOP_ROBOT]:
            try:
                set_robot_running = rospy.ServiceProxy('~set_robot_running', agv05_executive_msgs.srv.SetRobotRunning, persistent=False)
                set_robot_running(operation)
            except Exception:
                pass

        elif operation == BootSerializer.Operation.SOFT_REBOOT:
            subprocess.Popen(['sudo', 'service', 'kiosk', 'restart'], preexec_fn=os.setsid)
            subprocess.Popen(['sudo', 'supervisorctl', 'restart', 'agv05:*'], preexec_fn=os.setsid)

        elif operation == BootSerializer.Operation.HARD_REBOOT:
            subprocess.Popen(['sudo', 'reboot'])

        elif operation == BootSerializer.Operation.POWER_OFF:
            # Turn off embedded first, followed by PC itself.
            self.handle_shutdown()
            subprocess.Popen(['sudo', 'poweroff'])

    def handle_shutdown(self, req=None):
        # Turn off embedded
        shutdown_pub = rospy.Publisher('/shutdown', std_msgs.msg.Bool, queue_size=1, latch=True)

        if not self.robot.is_alive() and isinstance(self.robot, CapabilityLauncherMixin):
            try:
                CapabilityLauncherMixin.start(self.robot)
            except Exception:
                return False, 'Failed to start robot'

            timeout_time = rospy.get_time() + 50.0
            while not rospy.is_shutdown() and rospy.get_time() < timeout_time:
                rospy.sleep(1.0)
                if shutdown_pub.get_num_connections() > 0:
                    break

            if shutdown_pub.get_num_connections() == 0:
                return False, 'Timeout while waiting for shutdown topic'

        shutdown_pub.publish(std_msgs.msg.Bool(data=True))

        timeout_time = rospy.get_time() + 10.0
        while not rospy.is_shutdown() and rospy.get_time() < timeout_time:
            rospy.sleep(0.1)
            if self.shutdown_success:
                break

        return True, ''

    def handle_shutdown_success(self, data):
        self.shutdown_success = data.data
