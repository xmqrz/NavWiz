from __future__ import absolute_import
from __future__ import unicode_literals

import rospy
import time

from .module import Module


class HardwareApp(Module):
    id = 'hardware-app'

    def __init__(self, HWAppClass, app_desc, *args, **kwargs):
        super(HardwareApp, self).__init__(*args, **kwargs)
        self.id = app_desc['id']
        self.app_desc = app_desc

        start_import = time.time()
        self.hw_app = HWAppClass(self.out)
        import_duration = time.time() - start_import
        if import_duration > 2:
            rospy.logwarn('Warning : hw app plugin [%s] instance construction take longer time then expected. (%.2f seconds)', app_desc['id'], import_duration)

    def start(self):
        # allow an exception raised when failed to start.
        self.hw_app.start()

    def stop(self):
        try:
            self.hw_app.stop()
        except Exception as ex:
            rospy.logerr('Stopping module "%s" raised an exception: %s', self.id, ex)

    def handle_in_pipe(self, data):
        try:
            self.hw_app.handle_in_pipe(data)
        except Exception as ex:
            rospy.logerr('Calling handle_in_pipe for module "%s" raised an exception: %s', self.id, ex)
