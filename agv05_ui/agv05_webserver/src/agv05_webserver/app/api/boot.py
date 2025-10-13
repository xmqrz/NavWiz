from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Task
from django.conf import settings as django_settings
from django.utils.encoding import force_text
from rest_framework import permissions, viewsets
from rest_framework.response import Response
import agv05_executive_msgs.srv
import os
import rospy
import std_msgs.msg
import std_srvs.srv
import subprocess

from ..serializers import BootSerializer


class BootViewSet(viewsets.ViewSet):
    permission_classes = (permissions.AllowAny,)
    serializer_class = BootSerializer

    agv05_executor = getattr(django_settings, 'AGV05_EXECUTOR')

    def create(self, request, *args, **kwargs):
        serializer = BootSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        operation = serializer.validated_data['operation']
        if operation == BootSerializer.Operation.EXTEND_COUNTDOWN:
            if not request.user.has_perm('app.start_agv05'):
                self.permission_denied(request)

            success = False
            try:
                rospy.set_param('/agv05_sockserver/extend_countdown', True)
                success = True
            except Exception as ex:
                error_msg = force_text(ex)

            if not success:
                message = 'Failed to extend countdown.'
                if error_msg:
                    message += '\n\nError: %s' % error_msg
                return Response({'result': False, 'message': message})

        elif operation in [BootSerializer.Operation.START_ROBOT, BootSerializer.Operation.START_ROBOT_2, BootSerializer.Operation.START_ROBOT_3]:
            if not request.user.has_perm('app.start_agv05'):
                self.permission_denied(request)

            success = False
            try:
                set_robot_running = rospy.ServiceProxy(
                    self.agv05_executor + '/set_robot_running', agv05_executive_msgs.srv.SetRobotRunning, persistent=False)
                res = set_robot_running(operation)
                if res.success:
                    success = True
                error_msg = res.message
            except Exception as ex:
                error_msg = force_text(ex)

            if not success:
                message = 'AGV05 controller is not ready. Please try again.'
                if error_msg:
                    message += '\n\nError: %s' % error_msg
                return Response({'result': False, 'message': message})

        elif operation == BootSerializer.Operation.STOP_ROBOT:
            if not request.user.has_perm('app.stop_agv05'):
                self.permission_denied(request)

            success = False
            try:
                set_robot_running = rospy.ServiceProxy(
                    self.agv05_executor + '/set_robot_running', agv05_executive_msgs.srv.SetRobotRunning, persistent=False)
                res = set_robot_running(0)
                if res.success:
                    success = True
                error_msg = res.message
            except Exception as ex:
                error_msg = force_text(ex)

            if not success:
                message = 'Failed to stop AGV controller.'
                if error_msg:
                    message += '\n\nError: %s' % error_msg
                return Response({'result': False, 'message': message})

        elif operation in [BootSerializer.Operation.SOFT_REBOOT, BootSerializer.Operation.SAFE_SOFT_REBOOT]:
            if not request.user.has_perm('app.soft_reboot'):
                self.permission_denied(request)

            if operation == BootSerializer.Operation.SAFE_SOFT_REBOOT and Task.objects.running().exists():
                message = 'Please stop all running tasks before performing soft reboot.'
                return Response({'result': False, 'message': message})

            subprocess.Popen(['sudo', 'service', 'kiosk',
                              'restart'], preexec_fn=os.setsid)
            subprocess.Popen(
                ['sudo', 'supervisorctl', 'restart', 'agv05:*'], preexec_fn=os.setsid)

        elif operation == BootSerializer.Operation.HOT_RELOAD:
            if not request.user.has_perm('app.hot_reload'):
                self.permission_denied(request)

            if not self._is_robot_running():
                return Response({'result': False, 'message': 'Controller is not running.'})

            success = False
            try:
                srv = rospy.ServiceProxy(self.agv05_executor + '/hot_reload', std_srvs.srv.Trigger, persistent=False)
                res = srv()
            except Exception as ex:
                error_msg = force_text(ex)
                message = 'Failed to hot reload AGV controller.\n\nError: %s' % error_msg
                return Response({'result': False, 'message': message})

        elif operation == BootSerializer.Operation.HARD_REBOOT:
            if not request.user.has_perm('app.hard_reboot'):
                self.permission_denied(request)

            subprocess.Popen(['sudo', 'reboot'])

        elif operation == BootSerializer.Operation.POWER_OFF:
            if not request.user.has_perm('app.poweroff'):
                self.permission_denied(request)

            # Turn off embedded first, followed by PC itself.
            try:
                shutdown = rospy.ServiceProxy(
                    self.agv05_executor + '/shutdown', std_srvs.srv.Trigger, persistent=False)
                shutdown()
            except Exception:
                pass

            subprocess.Popen(['sudo', 'poweroff'])

        return Response({'result': True})

    def _is_robot_running(self):
        try:
            msg = rospy.wait_for_message(self.agv05_executor + '/robot_running', std_msgs.msg.UInt8, timeout=1)
            return msg.data
        except Exception:
            return False
