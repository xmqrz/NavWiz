from __future__ import absolute_import
from __future__ import unicode_literals

from django.http import Http404
from django.views import View
import os
import rospkg
import rospy

from ..models import Cache
from .hw_app import unsafe_sendfile


def dashboard_base_directory():
    navwiz_dashboard = rospy.get_param('navwiz_dashboard', None)
    if not navwiz_dashboard:
        return

    base_directory = _resolve_full_path(navwiz_dashboard)
    if not base_directory or not os.path.exists(base_directory):
        return

    return base_directory


def dashboard_params():
    base_directory = dashboard_base_directory()
    if not base_directory:
        return

    js_files = [f for f in os.listdir(base_directory) if f.endswith('.js') and os.path.isfile(os.path.join(base_directory, f))]
    if not js_files:
        return

    return {
        'base_directory': base_directory,
        'entry': js_files[0]
    }


def _resolve_full_path(input_path):
    rootfs = Cache.get_models_rootfs()
    ros_package_path = os.environ.get('ROS_PACKAGE_PATH')
    if ros_package_path and rootfs and os.path.exists(rootfs):
        os.environ['ROS_PACKAGE_PATH'] = ':'.join([os.path.join(rootfs, p.lstrip('/')) for p in ros_package_path.split(':')])

    try:
        rospack = rospkg.RosPack()
        path_components = input_path.split('/')

        package_name = path_components[0]
        base_path = rospack.get_path(package_name)

        return os.path.join(base_path, *path_components[1:])
    finally:
        os.environ['ROS_PACKAGE_PATH'] = ros_package_path


class DashboardView(View):

    def get(self, request, *args, **kwargs):
        asset = kwargs.get('asset')

        base_directory = dashboard_base_directory()

        if not base_directory:
            raise Http404('Dashboard does not exist.')

        file_path = os.path.join(base_directory, asset)

        if os.path.exists(file_path):
            return unsafe_sendfile(request, file_path)
        else:
            raise Http404('Asset file unavailable')
