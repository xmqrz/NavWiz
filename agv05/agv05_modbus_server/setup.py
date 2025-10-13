from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'agv05_modbus_server',
        'agv05_modbus_server.data_block',
    ],
    package_dir={'': 'src'},
    requires=[
        'agv05_executive_msgs',
        'agv05_msgs',
        'pymodbus',
        'rospy',
    ],
)

setup(**d)
