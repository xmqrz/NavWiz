from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
import logging
import os
import rospy


class MessageLogger(object):

    def __init__(self, robot):
        self.logger = logging.getLogger('custom_messages')
        formatter = logging.Formatter(fmt='%(created)f %(levelname)s %(message)s')

        folder = os.path.join(django_settings.MEDIA_ROOT, 'log/')
        if not os.path.exists(folder):
            try:
                os.makedirs(folder)
            except OSError:
                pass
        filename = os.path.join(folder, 'custom.log')

        try:
            handler = logging.handlers.RotatingFileHandler(
                filename, maxBytes=500 * 1024, backupCount=1, encoding='utf-8')
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
        except IOError:
            rospy.logerr('Failed to open custom log file for writing.')

        handler2 = FmsCustomLogHandler(robot)
        self.logger.addHandler(handler2)

    def log(self, level, message):
        self.logger.log(level, message)


class FmsCustomLogHandler(logging.Handler):

    def __init__(self, robot, *args, **kwargs):
        super(FmsCustomLogHandler, self).__init__(*args, **kwargs)
        self.robot = robot

    def emit(self, record):
        if self.robot.fms_manager:
            self.robot.fms_manager.send_custom_log_async({
                'created': record.created,
                'level_name': record.levelname,
                'agv_name': self.robot.models.agv_name,
                'message': record.message,
            })
