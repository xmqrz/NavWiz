#!/usr/bin/env python
import os
import rospy
import sys

if __name__ == "__main__":
    # Attempt to fix log file directory. When this script is run directly (not by roslaunch),
    # the log file path is not provided in the arguments ('__log:=').
    # Hence, we modify the environment to move the log into the '~/.ros/log/{run_id}' directory.
    try:
        os.environ['ROS_LOG_DIR'] = os.path.join(os.environ['ROS_HOME'], 'log/latest')
    except Exception:
        os.environ['ROS_LOG_DIR'] = os.path.join(os.environ['HOME'], '.ros/log/latest')

    if len(sys.argv) > 1 and sys.argv[1] in ['runserver', 'sockserver']:
        while not os.path.exists(os.environ['ROS_LOG_DIR']):
            rospy.sleep(1.0)

        node_names = {
            'runserver': 'agv05_webserver',
            'sockserver': 'agv05_sockserver',
        }
        rospy.init_node(node_names[sys.argv[1]], anonymous=True, disable_signals=True)

    # Remove ROS remapping arguments from sys.argv
    sys.argv = rospy.myargv()

    os.environ.setdefault("DJANGO_SETTINGS_MODULE", "agv05_webserver.settings")

    # Set dynamic_reconfigure_storage_backend based on Django's database settings
    try:
        if not rospy.has_param('/dynamic_reconfigure_storage_backend'):
            from django.conf import settings as django_settings
            db = django_settings.DATABASES['alt' if django_settings.TRACKLESS else 'default']
            engine = db['ENGINE']
            if 'sqlite3' in engine:
                rospy.set_param('/dynamic_reconfigure_storage_backend', 'dynamic_reconfigure_sqlite_storage/SqliteStorage')
                rospy.set_param('/dynamic_reconfigure_storage_backend_py', 'dynamic_reconfigure_sqlite_storage.SqliteStorage')
                rospy.set_param('/dynamic_reconfigure_storage_url', 'sqlite://%s#system_parameter' % db['NAME'])

            elif 'mysql' in engine:
                rospy.set_param('/dynamic_reconfigure_storage_backend', 'dynamic_reconfigure_mysql_storage/MysqlStorage')
                rospy.set_param('/dynamic_reconfigure_storage_backend_py', 'dynamic_reconfigure_mysql_storage.MysqlStorage')
                host = db['HOST'] or '127.0.0.1'
                port = ':%s' % db['PORT'] if db['PORT'] else ''
                rospy.set_param('/dynamic_reconfigure_storage_url', 'mysql://%s:%s@%s%s/%s#system_parameter' %
                    (db['USER'], db['PASSWORD'], host, port, db['NAME']))

        if not rospy.has_param('/variable_storage_url'):
            from django.conf import settings as django_settings
            db = django_settings.DATABASES['alt' if django_settings.TRACKLESS else 'default']
            engine = db['ENGINE']
            if 'mysql' in engine:
                host = db['HOST'] or '127.0.0.1'
                port = ':%s' % db['PORT'] if db['PORT'] else ''
                rospy.set_param('/variable_storage_url', 'mysql://%s:%s@%s%s/%s#system_variable' %
                    (db['USER'], db['PASSWORD'], host, port, db['NAME']))
    except Exception:
        pass

    try:
        from django.core.management import execute_from_command_line
    except ImportError:
        # The above import may fail for some other reason. Ensure that the
        # issue is really that Django is missing to avoid masking other
        # exceptions on Python 2.
        try:
            import django
        except ImportError:
            raise ImportError(
                "Couldn't import Django. Are you sure it's installed and "
                "available on your PYTHONPATH environment variable? Did you "
                "forget to activate a virtual environment?"
            )
        raise
    execute_from_command_line(sys.argv)
