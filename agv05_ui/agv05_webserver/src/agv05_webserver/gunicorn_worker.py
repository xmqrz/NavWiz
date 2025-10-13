import gunicorn.workers.sync


class SyncWorker(gunicorn.workers.sync.SyncWorker):

    def run(self):
        # Attempt to fix log file directory. When this script is run directly (not by roslaunch),
        # the log file path is not provided in the arguments ('__log:=').
        # Hence, we modify the environment to move the log into the '~/.ros/log/{run_id}' directory.
        import os
        try:
            os.environ['ROS_LOG_DIR'] = os.path.join(os.environ['ROS_HOME'], 'log/latest')
        except Exception:
            os.environ['ROS_LOG_DIR'] = os.path.join(os.environ['HOME'], '.ros/log/latest')

        import rospy
        while not os.path.exists(os.environ['ROS_LOG_DIR']):
            rospy.sleep(1.0)
        rospy.init_node('agv05_webserver', anonymous=True, disable_signals=True)

        super(SyncWorker, self).run()
