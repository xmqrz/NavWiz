#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from urllib.parse import urljoin
import os
import rospkg
import rospy
import threading
import tornado.ioloop
import tornado.web
import ujson as json

DIRECTORY = ''
rospack = rospkg.RosPack()


class StaticFileHandler(tornado.web.StaticFileHandler):
    def get(self, path, include_body=True):
        if path == '':
            path = 'index.html'
        super().get(path, include_body)


class DashboardDev(object):

    def __init__(self, navwiz_dashboard, port, is_trackless, is_agv_panel):
        try:
            assets = self._resolve_full_path(navwiz_dashboard)
        except Exception as ex:
            rospy.logerr('fail to resolve dashboard assets:', ex)
            raise RuntimeError('Fail loading plugin.')

        self.port = port
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
                    'is_trackless': is_trackless,
                    'is_agv_panel': is_agv_panel
                }))

        self.web_app = tornado.web.Application([
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
        if len(js_files) <= 0:
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
        rospy.loginfo('Starting dashboard dev server...')

        try:
            r = rospy.Rate(1)
            while not rospy.is_shutdown():
                r.sleep()
        finally:
            tornado.ioloop.IOLoop.current().stop()
            server_thread.join()

            rospy.loginfo('Stoping dashboard dev server...')

    def _resolve_full_path(self, input_path):
        path_components = input_path.split('/')

        package_name = path_components[0]
        base_path = rospack.get_path(package_name)

        return os.path.join(base_path, *path_components[1:])


def main():
    global DIRECTORY

    # get base path
    package_path = rospack.get_path('agv05_webapp')
    DIRECTORY = os.path.join(package_path, 'dashboard_build')
    if not DIRECTORY:
        rospy.logerr('Fail to obtain base assets path.')
        return

    rospy.init_node('dashboard_dev')

    try:
        # Get required param value
        navwiz_dashboard = rospy.get_param('~navwiz_dashboard')
        port = rospy.get_param('~port', 9080)
        is_trackless = rospy.get_param('~is_trackless', True)
        is_agv_panel = rospy.get_param('~is_agv_panel', False)
    except KeyError:
        rospy.logerr('Missing required params.')
        return

    try:
        dashboard_dev = DashboardDev(
            navwiz_dashboard,
            port,
            is_trackless,
            is_agv_panel
        )
    except Exception as ex:
        rospy.logerr('Error: %s' % ex)
        return

    dashboard_dev.spin()


if __name__ == '__main__':
    main()
