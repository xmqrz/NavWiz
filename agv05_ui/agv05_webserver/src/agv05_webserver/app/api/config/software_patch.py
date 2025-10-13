from __future__ import absolute_import
from __future__ import unicode_literals

from django.core.files.storage import default_storage
from django.utils import timezone
from rest_framework import exceptions, viewsets
from rest_framework.response import Response
import os
import re
import subprocess
import tempfile

from ...serializers import SoftwarePatchSerializer
from ..mixin import Permission


class SoftwarePatchConfigViewSet(viewsets.ViewSet):
    permission_classes = Permission('system.update_software')
    serializer_class = SoftwarePatchSerializer

    def list(self, request, *args, **kwargs):
        return Response({
            'changelog': self._get_changelog(),
        })

    def create(self, request, *args, **kwargs):
        serializer = SoftwarePatchSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        patch_file = serializer.validated_data['patch_file']
        if not patch_file.name.endswith('.tar'):
            raise exceptions.ValidationError({'patch_file': ['Only .tar file type is allowed.']})

        TMPDIR = tempfile.mkdtemp(dir='/tmp')
        patch_path = os.path.join(TMPDIR, patch_file.name)
        file_path = default_storage.save(patch_file.name, patch_file)
        file_path = default_storage.path(file_path)

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

        # Redirect page first
        subprocess \
            .Popen('sleep 1 && ' +
                   'mv \'%s\' \'%s\' &&' % (file_path, patch_path) +
                   'tar -xvf \'%s\' -C %s && ' % (patch_path, TMPDIR) +
                   'sudo DEBIAN_FRONTEND=noninteractive dpkg --force-depends ' +
                   '--force-confdef --force-confold ' +
                   '-i %s/*.deb >> %s 2>&1 && ' % (TMPDIR, log_file) +
                   'echo "PATCH COMPLETED!" >> %s || ' % log_file +
                   'echo "PATCH FAILED!" >> %s; ' % log_file +
                   'rm -rf %s' % TMPDIR,
                   shell=True, preexec_fn=os.setsid)

        return Response({'log_file': os.path.basename(log_file)})

    def _get_changelog(self):
        try:
            ros_distro = os.environ['ROS_DISTRO']
        except Exception:
            ros_distro = 'noetic'

        dependencies = subprocess.check_output('dpkg -s navwiz-core | grep ^Depends',
                                               shell=True, executable='/bin/bash',
                                               universal_newlines=True)
        dependencies_split = re.split(r'\s', dependencies)
        listOfRosPackages = [x.replace(',', '') for x in dependencies_split if re.match('ros-%s-.*' % ros_distro, x)]
        listOfPatchChangeLog = []
        for pkg in (['navwiz', 'navwiz-core'] + listOfRosPackages):
            try:
                changelog_content = subprocess.check_output(
                    'zgrep "PATCH:" /usr/share/doc/%s/changelog.Debian.gz' % pkg, shell=True, executable='/bin/bash', universal_newlines=True)
                if (len(changelog_content) > 0):
                    listOfPatchChangeLog.append(pkg)
                    listOfPatchChangeLog.append(changelog_content)
            except subprocess.CalledProcessError:
                pass
        return '\n'.join(listOfPatchChangeLog)
