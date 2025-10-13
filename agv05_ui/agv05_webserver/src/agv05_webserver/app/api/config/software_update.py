from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.core.files.storage import default_storage
from django.utils import timezone
from rest_framework import status, viewsets
from rest_framework.decorators import action
from rest_framework.generics import GenericAPIView
from rest_framework.response import Response
from rest_framework.reverse import reverse
import os
import subprocess
import tempfile

from agv05_webserver.system.models import Cache
from ..mixin import CustomErrorMixin, Permission
from ...serializers import SoftwareUpdateUsbInfoSerializer, SoftwareUpdateUsbSerializer


OS_RELEASE_CODENAME = 'focal'
try:
    import lsb_release
    OS_RELEASE_CODENAME = lsb_release.get_distro_information()['CODENAME']
except Exception:
    pass


def get_current_version():
    try:
        binary = all(s.startswith('/opt/ros/') for s in os.environ.get('ROS_PACKAGE_PATH').split(':'))
    except Exception:
        binary = True

    version = None
    if binary:
        version = get_debian_pkg_version('navwiz-core')
    return version or '%s-src' % Cache.get_models_version()


def is_kiosk_installed():
    return get_debian_pkg_version('navwiz') is not None


def get_debian_pkg_version(debian_pkg_name):
    """Returns the installed version of a debian package."""
    return next(get_debian_pkg_versions(debian_pkg_name, '/dev/null'), None)


def get_debian_pkg_versions(debian_pkg_name, sources_list):
    """Returns all available versions of a debian package."""
    try:
        output = subprocess.check_output(['apt-cache', 'show',
            '-o', 'Dir::Etc::sourcelist=%s' % sources_list,
            '-o', 'Dir::Etc::sourceparts=-', debian_pkg_name],
            universal_newlines=True)

        t = 'Version: '
        for line in output.splitlines():
            if line.startswith(t):
                yield line[len(t):]
    except subprocess.CalledProcessError:
        pass


class SoftwareUpdateConfigViewSet(CustomErrorMixin, GenericAPIView, viewsets.ViewSet):
    permission_classes = Permission('system.update_software')
    USBMOUNT_PATH = '/var/run/usbmount'

    def list(self, request, *args, **kwargs):
        return Response(OrderedDict([
            ('usb-info', reverse('app:config-api:software-update-usb-info', request=request)),
            ('usb', reverse('app:config-api:software-update-usb', request=request)),
        ]))

    @action(detail=False, methods=['POST'], serializer_class=SoftwareUpdateUsbInfoSerializer, url_path='usb-info')
    def usb_info(self, request, *args, **kwargs):
        serializer = self.get_serializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        usb_device = serializer.validated_data['usb']
        try:
            versions = self._list_versions_from_usb(usb_device)
            current_version = get_current_version()
            kiosk_installed = is_kiosk_installed()
            return Response(OrderedDict([
                ('current_version', current_version),
                ('kiosk_installed', kiosk_installed),
                ('versions', [v[0] for v in versions]),
            ]))
        except RuntimeError as ex:
            self._custom_error('Error while obtaining versions in usb: %s' % ex, status_code=status.HTTP_400_BAD_REQUEST)

    @action(detail=False, methods=['POST'], serializer_class=SoftwareUpdateUsbSerializer)
    def usb(self, request, *args, **kwargs):
        serializer = self.get_serializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        usb_device = serializer.validated_data['usb']
        version = serializer.validated_data['version']
        install_kiosk = serializer.validated_data.get('install_kiosk', False)
        versions = self._list_versions_from_usb(usb_device)
        sources_dict = dict(versions)

        if version not in sources_dict:
            return Response(OrderedDict([
                ('version', ['Invalid version number.']),
            ]))

        line = sources_dict[version]
        cdrom_mount_point = '-o Acquire::cdrom::mount="%s" ' % os.path.join(self.USBMOUNT_PATH, usb_device)

        if is_kiosk_installed():
            install_kiosk = True

        with tempfile.NamedTemporaryFile(mode='w', delete=False) as f:
            sources_list = f.name
            f.write(line + '\n')

        # Populate packages
        packages_list = '"navwiz-core=%s"' % version
        if install_kiosk:
            packages_list += ' "navwiz=%s"' % version

        # Purge old log files
        try:
            for filename in default_storage.listdir('software-update')[1]:
                path = os.path.join('software-update', filename)
                modified = default_storage.get_modified_time(path)
                if timezone.now() - modified > timezone.timedelta(days=1):
                    default_storage.delete(path)
        except Exception:
            pass

        # Create an empty log file
        log_folder = default_storage.path('software-update')
        if not os.path.isdir(log_folder):
            os.mkdir(log_folder)
        fd, log_file = tempfile.mkstemp('', '', log_folder)
        os.close(fd)

        # Redirect page first before installing updates
        subprocess.Popen('sleep 1 && ' +
            'sudo DEBIAN_FRONTEND=noninteractive apt-get install -y ' +
            cdrom_mount_point +
            '--fix-policy -o Debug::pkgProblemResolver=yes ' +
            '-o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" ' +
            '-o Dir::Etc::sourcelist="%s" ' % sources_list +
            '-o Dir::Etc::sourceparts=- ' +
            '%s >> %s 2>&1 && ' % (packages_list, log_file) +
            'echo "UPDATE COMPLETED!" >> %s || ' % log_file +
            'echo "UPDATE FAILED!" >> %s; ' % log_file +
            'rm "%s"' % sources_list, shell=True, preexec_fn=os.setsid)

        return Response({'log_file': os.path.basename(log_file)})

    def get_serializer(self, *args, **kwargs):
        serializer = super(SoftwareUpdateConfigViewSet, self).get_serializer(*args, **kwargs)
        if isinstance(serializer, (SoftwareUpdateUsbInfoSerializer, SoftwareUpdateUsbSerializer)):
            serializer.fields['usb'].choices = [(v, v) for v in self._list_usb()]
        return serializer

    def _list_usb(self):
        if os.path.isdir(self.USBMOUNT_PATH):
            return [usb for usb in os.listdir(self.USBMOUNT_PATH) if
                os.path.isdir(os.path.join(self.USBMOUNT_PATH, usb))]
        return []

    def _list_versions_from_usb(self, usb_device):
        usb_path = os.path.join(self.USBMOUNT_PATH, usb_device)
        try:
            with open(os.path.join(usb_path, '.disk', 'info'), 'r') as f:
                if not f.readline().strip():
                    raise RuntimeError
        except Exception:
            raise RuntimeError('Invalid USB installer')

        fd, sources_list = tempfile.mkstemp()
        os.close(fd)
        try:
            output = subprocess.check_output(['sudo', 'apt-cdrom', 'add',
                '-o', 'Dir::Etc::sourcelist=%s' % sources_list,
                '-m', '-d', usb_path],
                stderr=subprocess.STDOUT, universal_newlines=True)
        except subprocess.CalledProcessError as ex:
            if 'Signature verification failed' in ex.output:
                raise RuntimeError('Failed to validate authenticity of the USB installer')
            else:
                raise RuntimeError('Invalid USB installer')
        finally:
            subprocess.check_call(['sudo', 'rm', '-f', sources_list])

        for line in output.splitlines():
            if OS_RELEASE_CODENAME == 'bionic':
                if line.startswith('deb cdrom:[NavWiz (Bionic) '):
                    break
            elif OS_RELEASE_CODENAME == 'trusty':
                if line.startswith('deb cdrom:[NavWiz (Trusty) '):
                    break
            elif OS_RELEASE_CODENAME == 'focal':
                if line.startswith('deb cdrom:[NavWiz (Focal) '):
                    break
        else:
            raise RuntimeError('Invalid NavWiz USB installer')

        with tempfile.NamedTemporaryFile(mode='w') as f:
            sources_list = f.name
            f.write(line + '\n')
            f.flush()

            versions = [(v, line) for v in get_debian_pkg_versions('navwiz', sources_list)]
            if not versions:
                raise RuntimeError('No "navwiz" package found in USB installer')
            return versions
