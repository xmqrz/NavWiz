from __future__ import absolute_import
from __future__ import unicode_literals

from django.utils import timezone
from rest_framework import viewsets
from rest_framework.response import Response
import distutils.util
import os
import subprocess

from ..mixin import Permission
from ...serializers import DateTimeSerializer


class DateTimeConfigViewSet(viewsets.ViewSet):
    permission_classes = Permission('system.change_datetime')
    serializer_class = DateTimeSerializer

    def get_initial(self):
        initial = {}
        initial['date_and_time'] = timezone.localtime()
        initial['time_zone'] = timezone.get_current_timezone_name()

        try:
            lines = subprocess.check_output(['timedatectl'], universal_newlines=True).split('\n')
            for line in lines:
                kv = line.split('synchronized: ')
                if len(kv) == 2:
                    synced = bool(distutils.util.strtobool(kv[1]))
        except Exception:
            synced = False

        try:
            tracking = subprocess.check_output(
                ['chronyc', '-nc', 'tracking'],
                universal_newlines=True
            ).split(',')
            reference = tracking[1]
            offset = float(tracking[4])
        except Exception:
            initial['ntp_sync'] = False
            initial['ntp_status'] = 'Disabled.'
        else:
            initial['ntp_sync'] = True
            if reference:
                if synced:
                    initial['ntp_status'] = 'Synchronized to %s.' % reference
                else:
                    initial['ntp_status'] = 'Synchronizing to %s. System time is %f seconds %s of NTP time.' % (
                        reference, abs(offset), 'fast' if offset < 0 else 'slow')
            else:
                initial['ntp_status'] = 'Failed to connect to any external network time server.'

        try:
            initial['ntp_server'] = subprocess.check_output(
                ['grep', '^server .* iburst prefer trust', '/etc/chrony/chrony.conf'],
                universal_newlines=True).split()[1]
        except Exception:
            pass

        return initial

    def list(self, request, *args, **kwargs):
        init = self.get_initial()
        serializer = DateTimeSerializer(init)
        return Response(serializer.data)

    def create(self, request, *args, **kwargs):
        serializer = DateTimeSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        date_and_time = serializer.validated_data['date_and_time'].strftime('%Y-%m-%d %H:%M:%S')
        time_zone = serializer.validated_data['time_zone']
        ntp_sync = serializer.validated_data['ntp_sync']
        ntp_server = serializer.validated_data.get('ntp_server')

        if ntp_server:
            subcmd = (
                'sudo sed -i \'$a server %s iburst prefer trust\' /etc/chrony/chrony.conf && '
                'sudo sed -ri \'s/^initstepslew .*$/initstepslew 1.0 %s/\' /etc/chrony/chrony.conf && '
            ) % (ntp_server, ntp_server)
        else:
            subcmd = (
                'sudo sed -ri "s/^initstepslew .*\\$/initstepslew 1.0 '
                '`(dig +short +timeout=2 +retries=0 ntp.ubuntu.com || echo \'\') | awk \'END{print}\'` '
                'ntp.ubuntu.com/" /etc/chrony/chrony.conf && '
            )

        # The page redirect has to be returned first before performing the
        # date and time change, because otherwise nginx will give a
        # "504 Gateway Timeout" error when the date is set to the future.
        # Consequently, the result validation is sacrified.
        subprocess.Popen('sleep 1 && ' +
            'sudo timedatectl set-ntp "false" && sudo systemctl disable --now chrony && ' +
            'sudo TZ="%s" timedatectl set-time "%s" && ' % (time_zone, date_and_time) +
            'sudo timedatectl set-timezone "%s" && ' % time_zone +
            '(sudo hwclock -w &) && ' +
            'sudo sed -i \'/server .* iburst prefer trust/d\' /etc/chrony/chrony.conf && ' + subcmd +
            '(sudo systemctl %s --now chrony &) && ' % ('enable' if ntp_sync else 'disable') +
            'sudo supervisorctl restart agv05:*', shell=True, preexec_fn=os.setsid)

        return Response(serializer.validated_data)
