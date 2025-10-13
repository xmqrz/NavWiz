#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from functools import partial
from urllib.parse import urljoin
import importlib
import os
import re
import rospkg
import rospy
import sockjs.tornado
import threading
import time
import tornado.ioloop
import tornado.web
import ujson as json

from agv05_executor.app import App

DIRECTORY = ''
rospack = rospkg.RosPack()


class StaticFileHandler(tornado.web.StaticFileHandler):
    def get(self, path, include_body=True):
        if path == '':
            path = 'index.html'
        super().get(path, include_body)


class Connection(sockjs.tornado.SockJSConnection):
    app = None
    clients = set()

    def on_open(self, request):
        self.clients.add(self)
        rospy.loginfo('Client connected. Total clients: %d' % len(self.clients))

    def on_message(self, msg):
        if not self.app:
            return

        try:
            msg = json.loads(msg)
        except (KeyError, ValueError):
            rospy.logerr('Client message format error.')
            return

        if msg.get('action') == 'publish' and msg.get('channel') == 'module':
            try:
                self.app.handle_in_pipe(msg.get('data'))
            except Exception as ex:
                rospy.logerr('HwApp handle_in_pipe callback error: %s' % ex)

    def on_close(self):
        self.clients.remove(self)
        rospy.loginfo('Client disconnected. Total clients: %d' % len(self.clients))

    # thread-safe broadcast and send methods
    def broadcast(self, clients, msg):
        tornado.ioloop.IOLoop.instance().add_callback(partial(super(Connection, self).broadcast, clients, msg))

    def send(self, msg):
        tornado.ioloop.IOLoop.instance().add_callback(partial(super(Connection, self).send, msg))

    @classmethod
    def out(cls, data):
        try:
            data = json.dumps({
                'channel': 'module',
                'data': data
            })
        except Exception as ex:
            rospy.logerr('HwApp message format error: %s' % ex)
            return

        if cls.clients:
            conn = next(iter(cls.clients))
            conn.broadcast(cls.clients, data)


class HwAppDev(object):

    def __init__(self, app_plugin, assets, port, is_trackless, is_agv_panel):
        try:
            assets = self._resolve_full_path(assets)
        except Exception as ex:
            rospy.logerr('Cannot load hw app plugin [%s] fail to resolve plugin assets: %s', app_plugin, ex)
            raise RuntimeError('Fail loading plugin.')

        app_class = self._load_plugin0(app_plugin, assets)
        if app_class is None:
            raise RuntimeError('Fail loading plugin.')
        self.port = port
        self.app_id = app_plugin
        self.app_class = app_class
        self.entry = ''
        self.assets = assets

        self.server_address = ('', self.port)

        class EntryHandler(tornado.web.RequestHandler):
            def get(s):
                s.write(self.get_entry())

        class ContextHandler(tornado.web.RequestHandler):
            def get(s):
                s.set_header('Content-Type', 'application/json')
                s.write(json.dumps({
                    'name': self.app_class.name,
                    'is_trackless': is_trackless,
                    'is_agv_panel': is_agv_panel
                }))

        start_import = time.time()
        self.app = self.app_class(Connection.out)
        Connection.app = self.app

        duration = time.time() - start_import
        if duration > 2:
            rospy.logwarn('Warning : hw app plugin [%s] instance construction take longer time then expected. (%.2f seconds)', self.app_id, duration)

        router = sockjs.tornado.SockJSRouter(Connection, r'/ws/app')
        self.web_app = tornado.web.Application(router.urls + [
            (r'/api/entry', EntryHandler),
            (r'/api/context', ContextHandler),
            (r'/api/plugin/(.*)', StaticFileHandler, {'path': self.assets}),
            (r'/(.*)', StaticFileHandler, {'path': DIRECTORY}),
        ], debug=False, autoreload=False)

    def get_entry(self):
        self.reload_entry()
        return urljoin('/api/plugin/', self.entry)

    def reload_entry(self):
        if not os.path.exists(self.assets):
            return
        js_files = [f for f in os.listdir(self.assets) if f.endswith('.js') and os.path.isfile(os.path.join(self.assets, f))]
        if len(js_files) != 1:
            return
        self.entry = js_files[0]

    def _server_thread(self):
        rospy.loginfo('Starting server...')
        rospy.loginfo('Listening to %s:%s' % (self.server_address[0] or '0.0.0.0', self.server_address[1]))
        self.web_app.listen(
            self.server_address[1],
            address=self.server_address[0]
        )

        tornado.ioloop.IOLoop.current().start()

    def spin(self):
        # Start the server
        server_thread = threading.Thread(target=self._server_thread)
        server_thread.start()

        # TODO: accountability, show warning if slow start and stop.
        rospy.loginfo('Starting hw app...')
        self.app.start()

        try:
            r = rospy.Rate(1)
            while not rospy.is_shutdown():
                r.sleep()
        finally:
            tornado.ioloop.IOLoop.current().stop()
            server_thread.join()

            rospy.loginfo('Stoping hw app...')
            self.app.stop()

    def _load_plugin0(self, plugin, assets):
        try:
            if not re.fullmatch('[\\w._-]+', plugin):
                rospy.logerr('Cannot load hw app plugin [%s]: Plugin ID must only contain alphanumerics, underscores, hyphens and dots.', plugin)
                return None

            start_import = time.time()
            module_name, cls_name = plugin.rsplit('.', 1)
            module = importlib.import_module(module_name)
            cls = getattr(module, cls_name)

            import_duration = time.time() - start_import
            if import_duration > 2:
                rospy.logwarn('Warning : hw app plugin [%s] module import take longer time then expected. (%.2f seconds)', plugin, import_duration)

            if not issubclass(cls, App):
                rospy.logerr('Cannot load hw app plugin [%s]: Plugin must inherit `agv05_executor.app.App` class.', plugin)
                return None

            if cls.name == App.name:
                rospy.logerr('Cannot load hw app plugin [%s]: Plugin class must define a `name` attribute.', plugin)
                return None

            return cls

        except Exception as ex:
            rospy.logerr('Cannot load hw app plugin [%s]: %s', plugin, ex)

    def _resolve_full_path(self, input_path):
        path_components = input_path.split('/')

        package_name = path_components[0]
        base_path = rospack.get_path(package_name)

        return os.path.join(base_path, *path_components[1:])


def main():
    global DIRECTORY

    # get base path
    package_path = rospack.get_path('agv05_webapp')
    DIRECTORY = os.path.join(package_path, 'hwapp_build')
    if not DIRECTORY:
        rospy.logerr('Fail to obtain base assets path.')
        return

    rospy.init_node('hw_app_dev')

    try:
        # Get required param value
        app_plugin = rospy.get_param('~app_plugin')
        assets = rospy.get_param('~assets')
        port = rospy.get_param('~port', 9080)
        is_trackless = rospy.get_param('~is_trackless', True)
        is_agv_panel = rospy.get_param('~is_agv_panel', False)
    except KeyError:
        rospy.logerr('Missing required params.')
        return

    try:
        hw_app_dev = HwAppDev(
            app_plugin,
            assets,
            port,
            is_trackless,
            is_agv_panel
        )
    except Exception as ex:
        rospy.logerr('Error: %s' % ex)
        return

    hw_app_dev.spin()


if __name__ == '__main__':
    main()
