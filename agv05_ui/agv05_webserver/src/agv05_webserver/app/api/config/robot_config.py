from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from rest_framework.response import Response
import capabilities.srv
import debinterface
import distutils.util
import iptools
import os
import rospy
import six
import subprocess
import tempfile
import ujson as json

from ..mixin import Permission
from .network import NetworkViewSetMixin


class RobotConfigViewSet(NetworkViewSetMixin):
    permission_classes = Permission('system.view_system_panel', 'system.change_robot')
    RESERVED_ENVS = [
        'DEBUG', 'FORCE_SCRIPT_NAME', 'NAVWIZ_PLUGIN',
        'TRACKLESS', 'AUTO_START', 'ROBOT_NAME',
        'AGV05_BRINGUP_PROVIDER', 'AGV05_MOBILE_ROBOT_PROVIDER',
        'AGV05_SIMULATOR_MOBILE_ROBOT_PROVIDER',
    ]

    def list(self, request, *args, **kwargs):
        if not request.user.has_perm('system.view_system_panel'):
            self.permission_denied(request)

        initial = {}
        choices = {}

        try:
            initial['auto_start'] = rospy.get_param('/agv05_executor/auto_start')
        except KeyError:
            try:
                initial['auto_start'] = bool(distutils.util.strtobool(os.environ.get('AUTO_START')))
            except Exception:
                initial['auto_start'] = False

        try:
            initial['robot_name'] = rospy.get_param('/robot_name')
        except KeyError:
            initial['robot_name'] = os.environ.get('ROBOT_NAME')

        initial['network'] = self._collect_network()

        initial['env'] = self._collect_env()

        providers, default_provider = self._collect_mobile_robot_providers()
        if providers:
            choices['mobile_robot_provider'] = [(p, p) for p in ([''] + providers)]
        initial['mobile_robot_provider'] = default_provider if default_provider != '' else None

        providers, default_provider = self._collect_simulator_mobile_robot_providers()
        if providers:
            choices['simulator_mobile_robot_provider'] = [(p, p) for p in ([''] + providers)]
        initial['simulator_mobile_robot_provider'] = default_provider if default_provider != '' else None

        return Response(OrderedDict([
            ('initial', initial),
            ('choices', choices),
        ]))

    def create(self, request, *args, **kwargs):
        if not request.user.has_perm('system.change_robot'):
            self.permission_denied(request)

        auto_start = request.data['auto_start']
        robot_name = request.data['robot_name']
        mobile_robot_provider = request.data['mobile_robot_provider']
        simulator_mobile_robot_provider = request.data['simulator_mobile_robot_provider']
        network = request.data['network']
        env = request.data['env']

        # custom validation
        try:
            if robot_name in ['tracked_agv05', 'trackless_agv05']:
                assert mobile_robot_provider, 'Mobile robot provider must not be empty.'
            elif robot_name == 'trackless_simulator':
                assert simulator_mobile_robot_provider, 'Simulator mobile robot provider must not be empty.'

            assert isinstance(network, list), 'Network configuration is invalid.'

            network_name = []
            for n in network:
                assert isinstance(n, dict) and isinstance(n.get('name'), six.string_types) and \
                    isinstance(n.get('address'), six.string_types) and isinstance(n.get('netmask'), six.string_types) and \
                    isinstance(n.get('extraOptions'), dict), 'Network configuration is invalid.'

                assert n['name'].strip() and n['address'] and n['netmask'], 'Network configuration must not be empty.'

                assert n['name'] not in ['static', 'dhcp'], 'Network configuration name is a reserved key word.'
                assert n['name'].replace(' ', '0').isalnum(), 'Network configuration name must be alpha numeric or space.'

                assert n['name'] not in network_name, 'Network configuration name is not unique.'
                network_name.append(n['name'])

                assert iptools.ipv4.validate_ip(n['address']), 'Network configuration has an invalid IP address.'
                assert iptools.ipv4.netmask2prefix(n['netmask']), 'Network configuration has an invalid subnet mask.'

                assert isinstance(n['extraOptions'].get('autoneg_off', False), bool), 'Network configuration is invalid.'

            assert isinstance(env, list), 'Environment variable is invalid.'
            for e in env:
                assert len(e) == 2 and isinstance(e[0], six.string_types) and isinstance(e[1], six.string_types), \
                    'Environment variable is invalid.'
                assert e[0] and e[1], 'Environment variable must not be empty.'

                s = e[0].replace('_', '0')
                assert s.isalnum() and s.isupper() and s[0].isalpha(), \
                    'Environment variable identifier must begin with a capital letter, ' + \
                    'followed by one or more capital letters, numbers or underscores.'

                assert e[0] not in self.RESERVED_ENVS, 'Environment variable "%s" is reserved and cannot be set.' % e[0]

                s = e[1].replace('_', '0')
                # allow values like DISPLAY=:0.0
                s = s.replace('.', '0')
                s = s.replace(':', '0')
                # allow values like ROS_MASTER_URI=http://192.168.2.1:11311
                s = s.replace('/', '0')
                assert s.isalnum(), \
                    'Environment variable value must consist of letters, numbers or \'_.:/\'.'
        except Exception as ex:
            return Response({'error': {'error': '%s' % ex}})

        # environment variable default file
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as f:
            default_file_path = f.name
            f.write('# Set production(0) or debug(1) mode.\n')
            f.write('DEBUG=0\n\n')
            force_script_name = os.getenv('FORCE_SCRIPT_NAME')
            if force_script_name:
                f.write('# Force script name.\n')
                f.write('FORCE_SCRIPT_NAME=\'%s\'\n\n' % force_script_name)
            navwiz_plugin = os.getenv('NAVWIZ_PLUGIN')
            if navwiz_plugin:
                f.write('# NavWiz plugin.\n')
                f.write('NAVWIZ_PLUGIN=\'%s\'\n\n' % navwiz_plugin)
            f.write('# Set tracked(0) or trackless(1) mode.\n')
            f.write('TRACKLESS=%d\n\n' % robot_name.startswith('trackless'))
            f.write('# Robot config variables.\n')
            f.write('AUTO_START=%s\n' % auto_start)
            f.write('ROBOT_NAME=%s\n' % robot_name)
            f.write('AGV05_MOBILE_ROBOT_PROVIDER=%s\n\n' % mobile_robot_provider)
            f.write('AGV05_SIMULATOR_MOBILE_ROBOT_PROVIDER=%s\n\n' % simulator_mobile_robot_provider)
            f.write('# User-defined variables.\n')
            for e in env:
                f.write('%s=%s\n' % tuple(e))

        # network configuration
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

        wifi_interface_names = self.get_wifi_interface_names()
        interface_names = [i.name.decode() for i in self.get_interfaces() if i.name.decode() not in wifi_interface_names]

        # the loop modify adapters. avoid infinite loop.
        adapters = inf.adapters[:]
        for a in adapters:
            attrs = a.attributes
            name = attrs.get('name')
            unknown = attrs.get('unknown', {})
            preset = unknown.get('preset')
            if not preset:
                continue

            for n in network:
                if n['name'] != preset:
                    continue

                inf.removeAdapterByName(name)
                d = {
                    'name': name,
                    'preset': preset,
                    'addrFam': 'inet',
                    'source': 'static',
                    'metric': '10',
                    'address': n['address'],
                    'netmask': n['netmask'],
                    'post-down': 'ip addr flush dev $IFACE',
                }

                # Apply extra options
                if n['extraOptions'].get('autoneg_off'):
                    d['auto'] = True
                    d['ethernet-autoneg'] = 'off'
                    d['link-speed'] = 100
                    d['link-duplex'] = 'full'

                inf.addAdapter(d)
                break

        inf._header_comment = '# DO NOT REMOVE THIS COMMENT!\n' + \
            '# robotcfg_network: %s' % json.dumps(request.data['network'])

        inf.writeInterfaces()

        # ifplugd configuration
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as f:
            ifplugd_path = f.name
            f.write('INTERFACES="%s"\n' % ' '.join(interface_names))
            f.write('HOTPLUG_INTERFACES=""\n')
            f.write('ARGS="-q -f -u0 -d5 -w -I"\n')
            f.write('SUSPEND_ACTION="none"\n')

        # Redirect page first before updating default file and network
        subprocess.Popen('sleep 1 && ' +
            'sudo mv %s /etc/default/navwiz-core && ' % default_file_path +
            'sudo chown root:root /etc/default/navwiz-core && ' +
            'sudo chmod 0644 /etc/default/navwiz-core && ' +
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
            'sudo ifup -a --allow=hotplug && ' +
            '(sudo service kiosk restart & sudo supervisorctl restart agv05:*)',
            shell=True, preexec_fn=os.setsid)
        return Response({})

    def _collect_mobile_robot_providers(self):
        try:
            _get_providers = rospy.ServiceProxy('/capability_server/get_providers', capabilities.srv.GetProviders, persistent=False)
            req = capabilities.srv.GetProvidersRequest(interface='agv05_capabilities/MobileRobot')
            res = _get_providers(req)
            return sorted(res.providers), res.default_provider
        except Exception:
            return [], os.environ.get('AGV05_MOBILE_ROBOT_PROVIDER')

    def _collect_simulator_mobile_robot_providers(self):
        try:
            _get_providers = rospy.ServiceProxy('/capability_server/get_providers', capabilities.srv.GetProviders, persistent=False)
            req = capabilities.srv.GetProvidersRequest(interface='agv05_capabilities/SimulatorMobileRobot')
            res = _get_providers(req)
            return sorted(res.providers), res.default_provider
        except Exception:
            return [], os.environ.get('AGV05_SIMULATOR_MOBILE_ROBOT_PROVIDER')

    def _collect_network(self):
        prefix = '# robotcfg_network: '
        try:
            with open('/etc/network/interfaces') as f:
                for line in f:
                    if line.startswith(prefix):
                        network_cfg = json.loads(line[len(prefix):])
                        assert isinstance(network_cfg, list)

                        # Handle legacy format
                        if network_cfg and \
                                isinstance(network_cfg[0], list):
                            network_cfg = [{
                                'name': '',
                                'address': n[0],
                                'netmask': n[1],
                                'extraOptions': n[2],
                            } for n in network_cfg]

                        return network_cfg
        except Exception:
            pass
        return []

    def _collect_env(self):
        env = []
        try:
            with open('/etc/default/navwiz-core') as f:
                for line in f:
                    line = line.strip()
                    if not line or line[0] == '#' or line == 'DEBUG=0':
                        continue

                    try:
                        k, v = line.split('=')
                    except Exception:
                        pass
                    else:
                        if k not in self.RESERVED_ENVS:
                            env.append([k, v])
        except Exception:
            pass
        return env
