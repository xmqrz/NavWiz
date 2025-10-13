from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.core.files.storage import default_storage
from django.http import Http404
from rest_framework import exceptions, viewsets
from rest_framework.decorators import action
from rest_framework.response import Response
from sendfile import sendfile
import os
import subprocess

from ...serializers import HardwarePluginSerializer
from ..mixin import Permission
from ..version import _get_plugin_api_version


class HardwarePluginViewSet(viewsets.ViewSet):
    permission_classes = Permission('system.update_hardware_plugin')
    serializer_class = HardwarePluginSerializer
    lookup_field = 'filename'
    lookup_value_regex = '[^/.:][^/:]+'

    def list(self, request, *args, **kwargs):
        plugins = []
        for path in os.environ.get('NAVWIZ_PLUGIN', '').split(':'):
            if path:
                plugins.append(os.path.basename(path))
        return Response({'plugins': plugins})

    def create(self, request, *args, **kwargs):
        serializer = HardwarePluginSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        current = {}
        for path in os.environ.get('NAVWIZ_PLUGIN', '').split(':'):
            if path:
                basepath = os.path.basename(path)
                current[basepath] = path

        plugins = []
        errors = OrderedDict()
        updates = serializer.validated_data['plugins']
        for (idx, u) in enumerate(updates):
            if u.get('upload'):
                err = []
                upload = u['upload']
                name = upload.name
                if not name.endswith('.plugin'):
                    err.append('Only .plugin file type is allowed.')
                if ':' in name:
                    err.append('Plugin file name must not contain colon ":".')
                if not err:
                    plugin_path = self._save_plugin(upload, err)
                if err:
                    errors[idx] = {'upload': err}
                    continue
                plugins.append(plugin_path)

            elif u.get('name'):
                name = u['name']
                if name in current:
                    plugins.append(current[name])

        if errors:
            raise exceptions.ValidationError({'plugins': errors})

        plugins = ':'.join(plugins)
        self._save_config(plugins)
        return self.list(request)

    @action(detail=True)
    def download(self, request, *args, **kwargs):
        filename = kwargs['filename']
        for path in os.environ.get('NAVWIZ_PLUGIN', '').split(':'):
            if path and filename == os.path.basename(path):
                return sendfile(request, path, attachment=True)
        raise Http404()

    def _save_config(self, plugin_path):
        # return first before hard reboot
        subprocess.Popen('sleep 1 && '
            '(grep -q \'NAVWIZ_PLUGIN=\' /etc/default/navwiz-core && ' +
            'sudo sed -i \'s#^.*NAVWIZ_PLUGIN=.*$#NAVWIZ_PLUGIN=\'"\'"\'%s\'"\'"\'#\' /etc/default/navwiz-core || ' % plugin_path +
            'sudo sed -i \'$ a\\NAVWIZ_PLUGIN=\'"\'"\'%s\'"\'"\'\' /etc/default/navwiz-core) && ' % plugin_path +
            'sudo reboot',
            shell=True, preexec_fn=os.setsid)

    def _save_plugin(self, plugin_file, errors):
        plugin_new = os.path.join('plugin', plugin_file.name)
        plugin_new = default_storage.save(plugin_new, plugin_file)
        plugin_path = default_storage.path(plugin_new)
        mountpoint = plugin_path + '.mount'

        try:
            os.mkdir(mountpoint)
        except OSError:
            pass

        try:
            ros_distro = os.environ['ROS_DISTRO']
            version = _get_plugin_api_version()
            ros_path = mountpoint + '/opt/ros'
            version_path = mountpoint + '/version'
            plugin_data = subprocess.check_output('sudo mount \'%s\' \'%s\' 2>&1 1>/dev/null && ' % (plugin_path, mountpoint) +
                'echo "$( cat \'%s\' 2>/dev/null || echo "1")" && ' % version_path +
                'ls \'%s\' && ' % ros_path +
                '(sudo rsync -avhq \'%s/etc/udev/rules.d/\' /etc/udev/rules.d/ || true) && ' % mountpoint +
                '(sudo rsync -avhq \'%s/lib/udev/rules.d/\' /lib/udev/rules.d/ || true) && ' % mountpoint +
                'sudo umount \'%s\' 2>&1 1>/dev/null' % mountpoint, shell=True,
                universal_newlines=True)
            plugin_version, plugin_ros_distro = plugin_data.split('\n', 1)
            plugin_version = plugin_version.split(' ')[0]
            plugin_api_version = int(plugin_version.split('.')[0])
            plugin_ros_distro = plugin_ros_distro.strip()

            if ros_distro != plugin_ros_distro:
                errors.append('Unsupported %s release. Only %s release is supported.' % (plugin_ros_distro, ros_distro))
            if version != plugin_api_version:
                errors.append('Unsupported plugin version %s. Only version %s.x.x is supported.' % (plugin_version, version))
        except subprocess.CalledProcessError:
            errors.append('Fail to obtain plugin version.')
        finally:
            try:
                os.rmdir(mountpoint)
            except OSError:
                pass

        if errors:
            default_storage.delete(plugin_new)
            return False
        return plugin_path
