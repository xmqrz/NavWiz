#!/usr/bin/env python
import django
import os
import rospy
import signal
import sys
import distutils.util


def shutdown_ros(sig, stack):
    rospy.signal_shutdown('SIGINT')
    signal.default_int_handler()


if __name__ == '__main__':
    # debugpy
    if distutils.util.strtobool(os.getenv('DEBUGPY', '0')):
        import debugpy
        debugpy.listen(('localhost', 56781))
        debugpy.wait_for_client()

    # ROS init_node
    rospy.init_node('agv05_executor', disable_signals=True)
    signal.signal(signal.SIGINT, shutdown_ros)

    # setup django
    os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'agv05_webserver.settings')
    django.setup()

    from agv05_executor.executor import main
    sys.exit(main())
