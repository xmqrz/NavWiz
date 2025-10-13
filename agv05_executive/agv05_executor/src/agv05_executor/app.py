from __future__ import absolute_import
from __future__ import unicode_literals

import distutils.util
import importlib
import os
import re
import rospkg
import rospy
import time


class App(object):
    name = 'NONAME'

    def __init__(self, out):
        self.__out = out

    def start(self):
        pass

    def stop(self):
        pass

    def handle_in_pipe(self, data):
        raise NotImplementedError()

    def out(self, data):
        self.__out(data)


class AppManager(object):

    def __init__(self, robot):
        self.robot = robot
        self.apps = {}
        self.app_descriptions = []

        rootfs = os.environ.get('ROOTFS')
        for app_id, cls, entry, assets in self.load_plugins():
            if rootfs:
                assets = os.path.join(rootfs, assets.lstrip('/'))
            desc = {
                'id': app_id,
                'name': cls.name,
                'assets': assets,
                'entry': entry,
            }
            self.app_descriptions.append(desc)
            self.apps[app_id] = cls

    def get_app_descriptions(self):
        return self.app_descriptions

    def load_plugins(self):
        for plugin, assets in self._iter_plugins():
            cls, entry = self._load_plugin0(plugin, assets)
            if cls:
                yield (plugin, cls, entry, assets)

    def _load_plugin0(self, plugin, assets):
        try:
            if not re.fullmatch('[\\w._-]+', plugin):
                rospy.logerr('Cannot load hw app plugin [%s]: Plugin ID must only contain alphanumerics, underscores, hyphens and dots.', plugin)
                return (None, None)

            start_import = time.time()
            module_name, cls_name = plugin.rsplit('.', 1)
            module = importlib.import_module(module_name)
            cls = getattr(module, cls_name)

            import_duration = time.time() - start_import
            if import_duration > 2:
                rospy.logwarn('Warning : hw app plugin [%s] module import take longer time then expected. (%.2f seconds)', plugin, import_duration)

            if not issubclass(cls, App):
                rospy.logerr('Cannot load hw app plugin [%s]: Plugin must inherit `agv05_executor.app.App` class.', plugin)
                return (None, None)

            if cls.name == App.name:
                rospy.logerr('Cannot load hw app plugin [%s]: Plugin class must define a `name` attribute.', plugin)
                return (None, None)

            if not os.path.exists(assets):
                rospy.logerr('Cannot load hw app plugin [%s]: assets path does not exist.', plugin)
                return (None, None)

            js_files = [f for f in os.listdir(assets) if f.endswith('.js') and os.path.isfile(os.path.join(assets, f))]

            if len(js_files) != 1:
                rospy.logerr('Cannot load hw app plugin [%s]: invalid assets entry.', plugin)
                return (None, None)

            return (cls, js_files[0])

        except Exception as ex:
            rospy.logerr('Cannot load hw app plugin [%s]: %s', plugin, ex)
            return (None, None)

    def _iter_plugins(self):
        rospack = rospkg.RosPack()
        pkg_list = sorted(rospack.get_depends_on('agv05_executor', implicit=False))

        for pkg in pkg_list:
            m = rospack.get_manifest(pkg)
            for e in m.exports:
                if e.tag != 'hw_app':
                    continue

                plugin = e.get('app_plugin')
                assets = e.get('assets')

                tracked_only = False
                try:
                    tracked_only = distutils.util.strtobool(e.get('tracked_only'))
                except Exception:
                    pass

                trackless_only = False
                try:
                    trackless_only = distutils.util.strtobool(e.get('trackless_only'))
                except Exception:
                    pass

                if plugin is None or assets is None:
                    continue

                try:
                    assets = self._resolve_full_path(rospack, assets)
                except Exception as ex:
                    rospy.logerr('Cannot load hw app plugin [%s] fail to resolve plugin assets: %s', plugin, ex)
                    continue

                if tracked_only and self.robot.trackless:
                    continue
                if trackless_only and not self.robot.trackless:
                    continue

                yield (plugin, assets)

    def _resolve_full_path(self, rospack, input_path):
        path_components = input_path.split('/')

        package_name = path_components[0]
        base_path = rospack.get_path(package_name)

        return os.path.join(base_path, *path_components[1:])
