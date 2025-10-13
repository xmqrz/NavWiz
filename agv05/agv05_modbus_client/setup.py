from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'agv05_modbus_client',
    ],
    package_dir={'': 'src'},
    requires=[
        'agv05_msgs',
        'dynamic_reconfigure',
        'pymodbus',
        'rospy',
    ],
)

setup(**d)
