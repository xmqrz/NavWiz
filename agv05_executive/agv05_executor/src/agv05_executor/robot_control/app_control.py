from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.app.serializers import AppSerializer
import agv05_executive_msgs.srv
import rospy


class AppControl(object):

    def __init__(self, robot):
        self.robot = robot

    def start(self):
        pass

    def stop(self):
        pass

    def handle_app_control(self, data):
        try:
            serializer = AppSerializer(data=data)
            serializer.is_valid(raise_exception=True)
        except Exception:
            return

        operation = serializer.validated_data['operation']
        app_id = serializer.validated_data['app_id']

        try:
            set_app_running = rospy.ServiceProxy('~set_module_running', agv05_executive_msgs.srv.SetModuleRunning, persistent=False)
            set_app_running(operation, app_id)
        except Exception:
            pass
