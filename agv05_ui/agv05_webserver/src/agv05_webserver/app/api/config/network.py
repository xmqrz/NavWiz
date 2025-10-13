from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.core.files.base import File
from django.http import HttpResponse
from django.utils import timezone
from rest_framework import serializers, status, viewsets
from rest_framework.decorators import action
from rest_framework.response import Response
from rest_framework.reverse import reverse
import datetime
import debinterface
import iptools
import os
import pynetlinux
import pythonwifi.iwlibs
import socket
import subprocess
import tempfile
import time
import ujson as json

from ...serializers import NetworkEAPSerializer
from ..mixin import Permission


class NetworkViewSetMixin(viewsets.ViewSet):
    permission_classes = Permission('system.view_system_panel')
    serializer_class = serializers.Serializer

    def get_interfaces(self):
        return sorted((d for d in pynetlinux.ifconfig.iterifs() if not (
            d.name.decode().startswith('can') or d.name.decode().startswith('rename'))),
            key=lambda d: d.index)

    def get_wifi_interface_names(self):
        SYSFS_NET_PATH = pynetlinux.ifconfig.SYSFS_NET_PATH.decode()
        net_files = os.listdir(SYSFS_NET_PATH)
        interfaces = set()
        for d in net_files:
            if os.path.exists(os.path.join(SYSFS_NET_PATH, d, 'wireless')):
                interfaces.add(d)
        return interfaces


class NetworkViewSet(NetworkViewSetMixin):
    CRT_FOLDER = '/var/lib/navwiz/certs'

    WIFI_CRT = 'wifi_crt.p12'
    WIFI_CRT_PASS = 'wifi_crt.txt'
    WIFI_CA = 'wifi_ca.crt'

    def list(self, request, *args, **kwargs):
        return Response(OrderedDict([
            ('status', reverse('app:config-api:network-status', request=request)),
            ('configuration', reverse('app:config-api:network-configuration', request=request)),
            ('identify', reverse('app:config-api:network-identify', request=request)),
            ('hostname', reverse('app:config-api:network-hostname', request=request)),
            ('https', reverse('app:config-api:network-https', request=request)),
            ('eap', reverse('app:config-api:network-eap', request=request)),
        ]))

    @action(methods=['get'], detail=False)
    def status(self, request, *args, **kwargs):
        context = {}

        context['hostname'] = socket.gethostname()
        context['interfaces'] = []
        wifi_interface_names = self.get_wifi_interface_names()

        try:
            site_enabled = subprocess.Popen(['readlink', '-f', '/etc/nginx/sites-enabled/agv05'],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True).communicate()[0]
            context['scheme'] = 'HTTPS' if 'agv05_secured' in site_enabled else 'HTTP'
        except Exception:
            context['scheme'] = 'HTTP'

        try:
            with open(self.CRT_FOLDER + '/public.crt', 'r') as crt_file:
                public_crt = crt_file.read()
            p = subprocess.Popen(['openssl', 'x509', '-enddate', '-noout'],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            resp = p.communicate(input=public_crt)[0]
            if p.returncode == 0:
                exp_time = datetime.datetime.strptime(resp, 'notAfter=%b %d %H:%M:%S %Y GMT\n')
                context['crt_exp'] = timezone.make_aware(exp_time, timezone.utc)
        except Exception:
            pass

        try:
            ca_path = os.path.join(self.CRT_FOLDER, self.WIFI_CA)
            p = subprocess.Popen(['openssl', 'x509', '-enddate', '-fingerprint', '-noout', '-in', ca_path],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
            resp = p.communicate()[0]
            if p.returncode == 0:
                resp = resp.split('\n')
                exp_time = datetime.datetime.strptime(resp[0], 'notAfter=%b %d %H:%M:%S %Y GMT')
                context['wifi_ca_exp'] = timezone.make_aware(exp_time, timezone.utc)
                context['wifi_ca_fingerprint'] = resp[1].split('=')[1]
        except Exception:
            pass

        try:
            wifi_crt_path = os.path.join(self.CRT_FOLDER, self.WIFI_CRT)
            wifi_crt_pass_path = os.path.join(self.CRT_FOLDER, self.WIFI_CRT_PASS)
            with open(wifi_crt_pass_path, 'r') as f:
                wifi_crt_pass = f.read()
                wifi_crt_pass.strip()

            p1 = subprocess.Popen(["openssl", "pkcs12", "-passin",
                    "pass:%s" % wifi_crt_pass, "-in", wifi_crt_path, "-nokeys", "-chain"],
                    stdout=subprocess.PIPE, universal_newlines=True)
            p = subprocess.Popen(["openssl", "x509", "-noout", "-enddate", "-fingerprint"],
                                 stdin=p1.stdout, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 universal_newlines=True)
            p1.stdout.close()
            resp = p.communicate()[0]
            if p.returncode == 0:
                resp = resp.split('\n')
                exp_time = datetime.datetime.strptime(resp[0], 'notAfter=%b %d %H:%M:%S %Y GMT')
                context['wifi_crt_exp'] = timezone.make_aware(exp_time, timezone.utc)
                context['wifi_crt_fingerprint'] = resp[1].split('=')[1]
        except Exception:
            pass

        for interface in self.get_interfaces():
            iface = {
                'name': interface.name.decode(),
                'mac': interface.mac,
                'link_info': (0, None, True, False),  # (speed, duplex, auto, connected)
            }
            if interface.name.decode() in wifi_interface_names:
                wifi = pythonwifi.iwlibs.Wireless(interface.name.decode())
                try:
                    iface['link_info'] = (wifi.getBitrate().replace('/', 'p'), True, True, True)
                    q = wifi.getStatistics()[1]
                    q_max = wifi.getQualityMax()
                    iface['wifi'] = {
                        'specs': wifi.getWirelessName(),
                        'ssid': wifi.getEssid(),
                        'mode': wifi.getMode(),
                        'frequency': wifi.getFrequency(),
                        'access_point': wifi.getAPaddr(),
                        'quality': '%s/%s' % (q.quality, q_max.quality),
                        'signal_level': ('%s/%s' % (q.siglevel, q_max.siglevel)
                            if q.siglevel < 128 else '%s dBm' % (q.siglevel - 256)),
                    }
                except IOError:
                    iface['wifi'] = None
                    pass
            else:
                try:
                    iface['link_info'] = interface.get_link_info()
                except IOError:
                    pass
            if interface.ip:
                iface['ip'] = interface.ip
            if interface.netmask:
                iface['dotted_quad_netmask'] = iptools.ipv4.cidr2block('255.255.255.255/%s' % interface.netmask)[0]
            try:
                iface['get_stats'] = interface.get_stats()
            except Exception:
                pass
            context['interfaces'].append(iface)

        try:
            context['default_interface'] = pynetlinux.route.get_default_if()
            context['gateway'] = pynetlinux.route.get_default_gw()
            context['dns_nameservers'] = []
            with open('/etc/resolv.conf') as f:
                for line in f:
                    if line.startswith('nameserver'):
                        context['dns_nameservers'].append(line.split()[1])
        except Exception:
            pass

        return Response(context)

    @action(methods=['get', 'post'], detail=False, permission_classes=Permission('system.view_system_panel', 'system.change_network'))
    def configuration(self, request, *args, **kwargs):
        network_config = []
        prefix = '# robotcfg_network: '
        # robotcfg_network: [{"name":"Lidar Pole","address":"192.168.10.1","netmask":"255.255.255.0","extraOptions":{}}]
        try:
            with open('/etc/network/interfaces') as f:
                for line in f:
                    if line.startswith(prefix):
                        network_config = json.loads(line[len(prefix):])
                        assert isinstance(network_config, list)

                        # Handle legacy format
                        if network_config and \
                                isinstance(network_config[0], list):
                            network_config = [{
                                'name': 'Preset %s' % i,
                                'address': n[0],
                                'netmask': n[1],
                                'extraOptions': n[2],
                            } for i, n in enumerate(network_config, start=1)]

                        break
        except Exception:
            network_config = []

        network_preset_assoc = {n['name']: n for n in network_config}
        wifi_interface_names = self.get_wifi_interface_names()

        # use list to ensure consistent order
        interface_names = [
            interface.name.decode()
            for interface in self.get_interfaces()
            if interface.name.decode() not in wifi_interface_names
        ]

        if 'interfaces' in request.data:
            if not request.user.has_perm('system.change_network'):
                self.permission_denied(request)

            # write configuration
            fd, path = tempfile.mkstemp()
            os.close(fd)
            inf = debinterface.Interfaces(update_adapters=False)
            try:
                inf.updateAdapters()
            except Exception:
                # auto lo
                inf.addAdapter({
                    'name': 'lo',
                    'auto': True,
                    'addrFam': 'inet',
                    'source': 'loopback',
                })
            inf._set_paths(path, None)

            interfaces = []
            for n in request.data['interfaces']:
                if n['name'] in wifi_interface_names:
                    continue

                inf.removeAdapterByName(n['name'])

                source = n['source']
                if n['source'] == '':
                    continue

                interfaces.append(n['name'])
                d = {
                    'name': n['name'],
                    'addrFam': 'inet',
                }

                if source in network_preset_assoc:
                    d['source'] = 'static'
                    d['metric'] = 10
                    d['address'] = network_preset_assoc[source]['address']
                    d['netmask'] = network_preset_assoc[source]['netmask']
                    d['preset'] = source
                    # Apply extra options
                    extra_options = network_preset_assoc[source]['extraOptions']
                    if extra_options.get('autoneg_off'):
                        d['auto'] = True
                        d['ethernet-autoneg'] = 'off'
                        d['link-speed'] = 100
                        d['link-duplex'] = 'full'
                else:
                    d['source'] = source
                    d['metric'] = int(n['metric'])
                    if source == 'static':
                        d['address'] = n['address']
                        d['netmask'] = n['netmask']

                if d['source'] == 'static':
                    d['post-down'] = 'ip addr flush dev $IFACE'

                inf.addAdapter(d)

            inf.writeInterfaces()

            # ifplugd configuration
            with tempfile.NamedTemporaryFile(mode='w', delete=False) as f:
                ifplugd_path = f.name
                f.write('INTERFACES="%s"\n' % ' '.join(interfaces))
                f.write('HOTPLUG_INTERFACES=""\n')
                f.write('ARGS="-q -f -u0 -d5 -w -I"\n')
                f.write('SUSPEND_ACTION="none"\n')

            # Redirect page first before updating network
            subprocess.Popen('sleep 1 && ' +
                'sudo service ifplugd stop && ' +
                'sudo ifdown -a && ' +
                '(sudo killall wpa_supplicant || true) && ' +
                '(sudo sed -i -r "s/^#?timeout .*$/timeout 10;/" /etc/dhcp/dhclient.conf || true) && ' +
                '(sudo sed -i -r "s/^#?retry .*$/retry 30;/" /etc/dhcp/dhclient.conf || true) && ' +
                'sudo mv %s /etc/network/interfaces && ' % inf.interfaces_path +
                'sudo chown root:root /etc/network/interfaces && ' +
                'sudo chmod 0644 /etc/network/interfaces && ' +
                'sudo rm %s && ' % inf.backup_path +
                'sudo mv %s /etc/default/ifplugd && ' % ifplugd_path +
                'sudo chown root:root /etc/default/ifplugd && ' +
                'sudo chmod 0644 /etc/default/ifplugd && ' +
                'sudo service ifplugd start && ' +
                'sudo ifup lo && ' +
                'sudo ifup -a --allow=hotplug', shell=True)

        initial = {
            'interfaces': [{'name': n} for n in interface_names],
            'network_preset': {
                'names': {n['name'] for n in network_config},
                'field': network_preset_assoc,
            },
        }

        # read configuration
        try:
            adapters = debinterface.Interfaces().adapters
        except Exception:
            adapters = []

        for a in adapters:
            attrs = a.attributes
            n = attrs.get('name')
            unknown = attrs.get('unknown', {})
            if n in interface_names:
                interface_names.remove(n)
                for interface in initial['interfaces']:
                    if interface['name'] == n:
                        interface.update({
                            'source': unknown.get('preset', attrs.get('source', '')),
                            'address': attrs.get('address', ''),
                            'netmask': attrs.get('netmask', '255.255.255.0'),
                            'metric': unknown.get('metric', 10),
                        })
                        break

        for interface in initial['interfaces']:
            if interface['name'] in interface_names:
                interface.update({
                    'source': '',
                    'address': '',
                    'netmask': '255.255.255.0',
                    'metric': 10,
                })

        return Response(initial)

    @action(methods=['post'], detail=False)
    def identify(self, request, *args, **kwargs):
        if 'name' not in request.data or request.data['name'] in self.get_wifi_interface_names():
            return Response({}, status=status.HTTP_400_BAD_REQUEST)

        interface = request.data['name']
        try:
            p = subprocess.Popen(['sudo', 'ethtool', '-p', interface, '10'])
            time.sleep(1.0)
            assert not p.poll()  # error if device busy
            return Response(request.data, status=status.HTTP_200_OK)
        except Exception:
            return Response({}, status=status.HTTP_200_OK)

    @action(methods=['get', 'post'], detail=False, permission_classes=Permission('system.view_system_panel', 'system.change_network'))
    def hostname(self, request, *args, **kwargs):
        hostname = socket.gethostname()

        if 'hostname' in request.data:
            if not request.user.has_perm('system.change_network'):
                self.permission_denied(request)

            old_hostname = hostname
            hostname = request.data['hostname']

            # Redirect page first before updating network
            subprocess.Popen(
                'sleep 1 && ' +
                'sudo sed -ri \'s/^([0-9\\.]+)\\s+%s/\\1 %s/\' /etc/hosts &&' % (old_hostname, hostname) +
                'sudo hostnamectl set-hostname %s && ' % hostname +
                'sudo supervisorctl restart agv05:*', shell=True, preexec_fn=os.setsid)

        return Response({'hostname': hostname})

    @action(methods=['get', 'post'], detail=False, permission_classes=Permission('system.view_system_panel', 'system.change_network'))
    def https(self, request, *args, **kwargs):
        initial = {'gen_crt': False, 'error': {}}

        if 'network_scheme' in request.data:
            if not request.user.has_perm('system.change_network'):
                self.permission_denied(request)

            scheme = request.data['network_scheme']
            private_key = request.data['private_key']
            public_crt = request.data['public_crt']
            gen_crt = request.data['gen_crt']

            if private_key:
                try:
                    p = subprocess.Popen(['openssl', 'rsa', '-check', '-noout', '-passin', 'pass:'],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            universal_newlines=True)
                    p.communicate(input=private_key)
                    if p.returncode != 0:
                        initial['error']['private_key'] = 'Invalid private key.'
                        return Response(initial)
                except Exception:
                    initial['error']['private_key'] = 'Error while checking private key.'
                    return Response(initial)

            if public_crt and not gen_crt:
                try:
                    p = subprocess.Popen(['openssl', 'x509', '-text', '-noout'],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            universal_newlines=True)
                    p.communicate(input=public_crt)
                    if p.returncode != 0:
                        initial['error']['public_crt'] = 'Invalid public certificate.'
                        return Response(initial)
                except Exception:
                    initial['error']['public_crt'] = 'Error while checking public certificate.'
                    return Response(initial)

                if not private_key:
                    initial['error']['private_key'] = 'Fail to verify public certificate, private key is required.'
                    return Response(initial)
                try:
                    crt_modulus = subprocess.Popen(['openssl', 'x509', '-modulus', '-noout'],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, universal_newlines=True).communicate(input=public_crt)[0]
                    key_modulus = subprocess.Popen(['openssl', 'rsa', '-modulus', '-noout'],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, universal_newlines=True).communicate(input=private_key)[0]
                    if crt_modulus != key_modulus:
                        initial['error']['private_key'] = 'Public certificate is not from the provided private key.'
                        initial['error']['public_crt'] = 'Public certificate is not from the provided private key.'
                        return Response(initial)
                except Exception:
                    initial['error']['private_key'] = 'Fail to validate public certificate and private key pair.'
                    initial['error']['public_crt'] = 'Fail to validate public certificate and private key pair.'
                    return Response(initial)

            if gen_crt:
                if not private_key:
                    try:
                        private_key = subprocess.check_output(['openssl', 'genrsa', '2048'], universal_newlines=True)
                    except Exception:
                        initial['error']['gen_crt'] = 'Fail to generate private key.'
                        return Response(initial)
                try:
                    p = subprocess.Popen(['openssl', 'req', '-key', '/dev/stdin',
                        '-new', '-x509', '-days', '3650', '-subj', '/CN=localhost'],
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True)
                    cert = p.communicate(input=private_key)[0]
                    if p.returncode == 0:
                        public_crt = cert
                    else:
                        initial['error']['gen_crt'] = 'Fail to generate public certificate.'
                        return Response(initial)
                except Exception:
                    initial['error']['gen_crt'] = 'Fail to generate public certificate.'
                    return Response(initial)

            if scheme == 'https':
                if not public_crt:
                    initial['error']['public_crt'] = 'Public certificate required for HTTPS mode.'
                    return Response(initial)
                if not private_key:
                    initial['error']['private_key'] = 'Private key required for HTTPS mode.'
                    return Response(initial)

            try:
                os.makedirs(self.CRT_FOLDER)
            except OSError:
                pass

            if private_key:
                try:
                    with open(self.CRT_FOLDER + '/private.key', 'w+') as key_file:
                        key_file.write(private_key)
                    os.chmod(self.CRT_FOLDER + '/private.key', 0o600)
                except Exception:
                    initial['error']['private_key'] = 'Fail to save private key.'
                    return Response(initial)
            else:
                try:
                    os.remove(self.CRT_FOLDER + '/private.key')
                except OSError:
                    pass

            if public_crt:
                try:
                    with open(self.CRT_FOLDER + '/public.crt', 'w+') as crt_file:
                        crt_file.write(public_crt)
                    os.chmod(self.CRT_FOLDER + '/public.crt', 0o600)
                except Exception:
                    initial['error']['public_crt'] = 'Fail to save public certificate.'
                    return Response(initial)
            else:
                try:
                    os.remove(self.CRT_FOLDER + '/public.crt')
                except OSError:
                    pass

            # Redirect page first before updating network
            subprocess.Popen('sleep 1 && ' +
                'sudo ln -sfT /etc/nginx/sites-available/%s /etc/nginx/sites-enabled/agv05 &&' % ('agv05_secured' if scheme == 'https' else 'agv05') +
                'sudo nginx -t &&' +
                'sudo service nginx restart ||' +
                'sudo ln -sfT /etc/nginx/sites-available/agv05 /etc/nginx/sites-enabled/agv05', shell=True)

        try:
            site_enabled = subprocess.Popen(['readlink', '-f', '/etc/nginx/sites-enabled/agv05'],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True).communicate()[0]
            initial['network_scheme'] = 'https' if 'agv05_secured' in site_enabled else 'http'
        except Exception:
            pass

        try:
            with open(self.CRT_FOLDER + '/public.crt', 'r') as crt_file:
                initial['public_crt'] = crt_file.read()
        except Exception:
            initial['public_crt'] = ''

        try:
            with open(self.CRT_FOLDER + '/private.key', 'r') as key_file:
                initial['private_key'] = key_file.read()
        except Exception:
            initial['private_key'] = ''

        return Response(initial)

    @action(methods=['get', 'post'], detail=False, serializer_class=NetworkEAPSerializer, permission_classes=Permission('system.view_system_panel', 'system.change_network'))
    def eap(self, request, *args, **kwargs):
        ca_path = os.path.join(self.CRT_FOLDER, self.WIFI_CA)
        wifi_crt_path = os.path.join(self.CRT_FOLDER, self.WIFI_CRT)
        wifi_crt_pass_path = os.path.join(self.CRT_FOLDER, self.WIFI_CRT_PASS)

        download = request.data.get('download') or self.request.GET.get('download', None)
        if download == self.WIFI_CA:
            if os.path.isfile(ca_path):
                with open(ca_path, 'r') as f:
                    response = HttpResponse(f.read(), content_type='application/x-x509-ca-cert')
                    response['Content-Disposition'] = 'attachment; filename="%s"' % self.WIFI_CA
                    return response
        elif download == self.WIFI_CRT:
            if os.path.isfile(wifi_crt_path):
                with open(wifi_crt_path, 'r') as f:
                    response = HttpResponse(f.read(), content_type='application/x-pkcs12')
                    response['Content-Disposition'] = 'attachment; filename="%s"' % self.WIFI_CRT
                    return response

        initial = {'error': {}}

        if 'wifi_crt_pass' in request.data:
            if not request.user.has_perm('system.change_network'):
                self.permission_denied(request)

            ca_crt = request.FILES.get('ca_crt') or request.data.get('ca_crt') == 'on'
            wifi_crt = request.FILES.get('wifi_crt') or request.data.get('wifi_crt') == 'on'
            wifi_crt_pass = request.data.get('wifi_crt_pass') or ''
            wifi_crt_pass.strip()
            ca_in_wifi_crt = None

            # Validate

            try:
                if wifi_crt and isinstance(wifi_crt, File):
                    p = subprocess.Popen(['openssl', 'pkcs12', '-noout', '-passin', 'pass:%s' % wifi_crt_pass],
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                    wifi_crt.seek(0)
                    p.communicate(input=wifi_crt.read())

                    # retry with existing password
                    if p.returncode != 0 and os.path.isfile(wifi_crt_pass_path):
                        with open(wifi_crt_pass_path, 'r') as f:
                            wifi_crt_pass = f.read()
                            wifi_crt_pass.strip()
                        p = subprocess.Popen(['openssl', 'pkcs12', '-noout', '-passin', 'pass:%s' % wifi_crt_pass],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                        wifi_crt.seek(0)
                        p.communicate(input=wifi_crt.read())

                    if p.returncode != 0:
                        initial['error']['wifi_crt'] = 'Invalid Wi-Fi certificate or password.'
                        initial['error']['wifi_crt_pass'] = 'Invalid Wi-Fi certificate or password.'
                        return Response(initial)

                    # check if ca exist in p12
                    p = subprocess.Popen(['openssl', 'pkcs12', '-cacerts', '-nokeys', '-passin', 'pass:%s' % wifi_crt_pass],
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE)
                    wifi_crt.seek(0)
                    o, e = p.communicate(input=wifi_crt.read())
                    if p.returncode == 0 and o:
                        o = o.decode('utf-8')
                        p = subprocess.Popen(['openssl', 'x509', '-noout'],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                        p.communicate(input=o)
                        if p.returncode == 0:
                            ca_in_wifi_crt = o

                elif not wifi_crt and os.path.isfile(wifi_crt_path):
                    p = subprocess.Popen(['openssl', 'pkcs12', '-noout', '-passin', 'pass:%s' % wifi_crt_pass, '-in', wifi_crt_path])
                    p.wait()

                    # retry with existing password
                    if p.returncode != 0 and os.path.isfile(wifi_crt_pass_path):
                        with open(wifi_crt_pass_path, 'r') as f:
                            wifi_crt_pass = f.read()
                            wifi_crt_pass.strip()
                        p = subprocess.Popen(['openssl', 'pkcs12', '-noout', '-passin', 'pass:%s' % wifi_crt_pass, '-in', wifi_crt_path])
                        p.wait()

                    if p.returncode != 0:
                        initial['error']['wifi_crt'] = 'Invalid Wi-Fi certificate or password.'
                        initial['error']['wifi_crt_pass'] = 'Invalid Wi-Fi certificate or password.'
                        return Response(initial)

            except Exception:
                initial['error']['wifi_crt'] = 'Error while validating Wi-Fi certificate.'
                return Response(initial)

            try:
                if ca_crt and isinstance(ca_crt, File):
                    p = subprocess.Popen(['openssl', 'x509', '-noout'],
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                    ca_crt.seek(0)
                    p.communicate(input=ca_crt.read())
                    if p.returncode != 0:
                        initial['error']['ca_crt'] = 'Invalid CA certificate.'
                        return Response(initial)
            except Exception:
                initial['error']['ca_crt'] = 'Error while validating CA certificate.'
                return Response(initial)

            try:
                os.makedirs(self.CRT_FOLDER)
            except OSError:
                pass

            # Save

            if ca_crt and isinstance(ca_crt, File):
                try:
                    with open(ca_path, 'wb') as key_file:
                        ca_crt.seek(0)
                        key_file.write(ca_crt.read())
                    os.chmod(ca_path, 0o600)
                except Exception:
                    initial['error']['ca_crt'] = 'Fail to save CA certificate.'
                    return Response(initial)
            elif ca_in_wifi_crt:
                try:
                    with open(ca_path, 'wb') as key_file:
                        key_file.write(ca_in_wifi_crt)
                    os.chmod(ca_path, 0o600)
                except Exception:
                    initial['error']['wifi_crt'] = 'Fail to save CA certificate from Wi-Fi certificate.'
                    return Response(initial)
            elif ca_crt is True:
                if os.path.isfile(ca_path):
                    os.remove(ca_path)

            if wifi_crt and isinstance(wifi_crt, File):
                try:
                    with open(wifi_crt_path, 'wb') as key_file:
                        wifi_crt.seek(0)
                        key_file.write(wifi_crt.read())
                    os.chmod(wifi_crt_path, 0o600)
                except Exception:
                    initial['error']['wifi_crt'] = 'Fail to save Wi-Fi certificate.'
                    return Response(initial)
            elif wifi_crt is True:
                if os.path.isfile(wifi_crt_path):
                    os.remove(wifi_crt_path)

            if os.path.isfile(wifi_crt_path):
                try:
                    with open(wifi_crt_pass_path, 'w') as f:
                        f.write(wifi_crt_pass)
                    os.chmod(wifi_crt_pass_path, 0o600)
                except Exception:
                    initial['error']['wifi_crt_pass'] = 'Error saving Wi-Fi certificate.'
                    return Response(initial)
            elif os.path.isfile(wifi_crt_pass_path):
                os.remove(wifi_crt_pass_path)

        if os.path.isfile(ca_path):
            initial['ca_crt'] = self.WIFI_CA
        if os.path.isfile(wifi_crt_path):
            initial['wifi_crt'] = self.WIFI_CRT
            if os.path.isfile(wifi_crt_pass_path):
                with open(wifi_crt_pass_path, 'r') as f:
                    content = f.read()
                    if content.strip():
                        initial['wifi_crt_pass'] = '********'

        return Response(initial)
