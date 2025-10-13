from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['agv05_skills'],
    package_dir={'': 'src'},
    requires=['agv05_executor', 'smach', 'rospy'],
)

setup(**d)
