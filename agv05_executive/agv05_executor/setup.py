from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'agv05_executor',
        'agv05_executor.models',
        'agv05_executor.models.validators',
        'agv05_executor.modules',
        'agv05_executor.modules.live_app',
        'agv05_executor.robot_control',
        'agv05_executor.robots',
    ],
    package_dir={'': 'src'},
    requires=[
        'actionlib',
        'agv05_executive_msgs',
        'agv05_msgs',
        'agv05_webserver',
        'capabilities',
        'dynamic_reconfigure',
        'geometry_msgs',
        'networkx',
        'rospkg',
        'rospy',
        'smach',
        'smach_ros',
        'tf',
    ],
)

setup(**d)
