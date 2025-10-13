from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'agv05_variable_storage',
    ],
    package_dir={'': 'src'},
    requires=['rospy', 'SQLAlchemy'],
    install_requires=['MySQL-python'],
)

setup(**d)
