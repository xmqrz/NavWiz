from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from django.utils.encoding import force_text
from rest_framework import permissions, status, viewsets
from rest_framework.decorators import action
from rest_framework.response import Response
from rest_framework.reverse import reverse
import rospy
import std_srvs.srv
import linux_wifi.srv
import linux_wifi.msg
import ujson as json

from ..serializers import WifiOpSerializer, WifiConnectSerializer, WifiIPConfigSerializer


class WifiViewSet(viewsets.ViewSet):
    permission_classes = (permissions.AllowAny,)
    serializer_class = None
    wifi_manager = getattr(django_settings, 'WIFI_MANAGER')

    def list(self, request, *args, **kwargs):
        return Response({
            'connect': reverse('app:api:wifi-connect', request=request),
            'operation': reverse('app:api:wifi-operation', request=request),
            'setip': reverse('app:api:wifi-setip', request=request),
        })

    @action(methods=['post'], detail=False, serializer_class=WifiOpSerializer)
    def operation(self, request, *args, **kwargs):
        serializer = WifiOpSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        error_msg = ''
        message = ''
        wifiop = serializer.data['wifiop']
        state = serializer.data['state']

        if wifiop == WifiOpSerializer.ADHOC:
            if not request.user.has_perm('app.wifi_adhoc'):
                self.permission_denied(request)

            success = False
            try:
                set_adhoc = rospy.ServiceProxy(self.wifi_manager + '/adhoc', std_srvs.srv.SetBool, persistent=False)
                service = set_adhoc(state)
                success = service.success
                message = service.message
            except Exception as ex:
                error_msg = force_text(ex)

        elif wifiop == WifiOpSerializer.WIFILIST:
            success = False
            try:
                refresh_wifi = rospy.ServiceProxy(self.wifi_manager + '/list', linux_wifi.srv.WifiList, persistent=False)
                service = refresh_wifi()
                ssid_list = json.loads(service.ssid_list)
                ip_cfg = json.loads(service.ip_cfg)
                success = True
                return Response({'ssid_list': ssid_list, 'ip_cfg': ip_cfg}, status=status.HTTP_200_OK)
            except Exception as ex:
                error_msg = force_text(ex)

        elif wifiop == WifiOpSerializer.WNICS:
            if not request.user.has_perm('app.wifi_connect'):
                self.permission_denied(request)

            success = False
            try:
                srv = rospy.ServiceProxy(self.wifi_manager + '/wnics', std_srvs.srv.SetBool, persistent=False)
                service = srv(state)
                success = service.success
                message = service.message
            except Exception as ex:
                error_msg = force_text(ex)

        elif wifiop == WifiOpSerializer.STATUS:
            if not request.user.has_perm('app.wifi_connect'):
                self.permission_denied(request)

            success = False
            try:
                msg = rospy.wait_for_message(self.wifi_manager + '/status', linux_wifi.msg.WifiStatus, timeout=1)
                success = True
                wnics_status = msg.state
                adhoc_status = msg.adhocstate
                message = {'wnics': wnics_status, 'adhoc': adhoc_status}
            except Exception as ex:
                error_msg = force_text(ex)

        return Response({'result': success, 'message': message, 'error': error_msg}, status=status.HTTP_200_OK)

    @action(methods=['post'], detail=False, serializer_class=WifiConnectSerializer)
    def connect(self, request, *args, **kwargs):
        serializer = WifiConnectSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        if not request.user.has_perm('app.wifi_connect'):
            self.permission_denied(request)

        error_msg = ''
        message = ''
        op = serializer.data['op']
        wificonnect_ssid = serializer.data['ssid']
        if op == WifiConnectSerializer.CONNECT:
            wificonnect_pass = ''
            wificonnect_identity = ''
            wificonnect_enc = serializer.data['enc']
            wificonnect_eap = ''
            wificonnect_hidden = serializer.data['hidden']

            if serializer.data['password'] is not None:
                wificonnect_pass = serializer.data['password']

            if serializer.data['identity'] is not None:
                wificonnect_identity = serializer.data['identity']

            if serializer.data['eap'] is not None:
                wificonnect_eap = serializer.data['eap']

            success = False

            try:
                srv = rospy.ServiceProxy(self.wifi_manager + '/connect', linux_wifi.srv.WifiConnect, persistent=False)
                service = srv(wificonnect_ssid, wificonnect_identity, wificonnect_pass, wificonnect_enc, wificonnect_eap, wificonnect_hidden)
                success = service.success
                message = service.message
            except Exception as ex:
                error_msg = force_text(ex)

        elif op == WifiConnectSerializer.FORGET:
            success = False

            try:
                srv = rospy.ServiceProxy(self.wifi_manager + '/forget', linux_wifi.srv.WifiForget, persistent=False)
                service = srv(wificonnect_ssid)
                success = service.success
                message = service.message
            except Exception as ex:
                error_msg = force_text(ex)

        return Response({'result': success, 'message': message, 'error': error_msg}, status=status.HTTP_200_OK)

    @action(methods=['post'], detail=False, serializer_class=WifiIPConfigSerializer)
    def setip(self, request, *args, **kwargs):
        serializer = WifiIPConfigSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        if not request.user.has_perm('app.wifi_connect'):
            self.permission_denied(request)

        error_msg = ''
        result = ''
        static = serializer.data['staticmode']
        success = False
        if static:
            ip = serializer.data['ip']
            netmask = serializer.data['netmask']
            gateway = serializer.data['gateway']
            dns_nameserver = serializer.data['dns_nameserver']
        elif not static:
            ip = ''
            netmask = ''
            gateway = ''
            dns_nameserver = ''

        try:
            srv = rospy.ServiceProxy(self.wifi_manager + '/set_ip', linux_wifi.srv.WifiSetIP, persistent=False)
            service = srv(static, ip, netmask, gateway, dns_nameserver)
            success = service.success
            message = service.message
        except Exception as ex:
            error_msg = force_text(ex)

        return Response({'result': success, 'message': result, 'error': error_msg}, status=status.HTTP_200_OK)
