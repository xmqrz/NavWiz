from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


###################################################################

KEYWORDS = ['navwiz', 'agv', 'dfautomation']
CLASSIFIERS = [
    'Intended Audience :: Developers',
    'Natural Language :: English',
    'License :: Other/Proprietary License',
    'Operating System :: POSIX :: Linux',
    'Programming Language :: Python',
    'Programming Language :: Python :: 2',
    'Programming Language :: Python :: 2.7',
    'Programming Language :: Python :: 3',
    'Programming Language :: Python :: 3.8',
]

###################################################################


d = generate_distutils_setup(
    keywords=KEYWORDS,
    packages=[
        'agv05_webserver',
        'agv05_webserver.anon',
        'agv05_webserver.anon.migrations',
        'agv05_webserver.app',
        'agv05_webserver.app.api',
        'agv05_webserver.app.api.config',
        'agv05_webserver.app.channels',
        'agv05_webserver.app.management',
        'agv05_webserver.app.management.commands',
        'agv05_webserver.app.migrations',
        'agv05_webserver.appx',
        'agv05_webserver.appx.api',
        'agv05_webserver.appx.api.config',
        'agv05_webserver.appx.management',
        'agv05_webserver.appx.migrations',
        'agv05_webserver.hashicon',
        'agv05_webserver.system',
        'agv05_webserver.system.channels',
        'agv05_webserver.system.management',
        'agv05_webserver.system.management.commands',
        'agv05_webserver.system.migrations',
        'agv05_webserver.system.tasks',
        'agv05_webserver.system.views',
        'agv05_webserver.systemx',
        'agv05_webserver.systemx.channels',
        'agv05_webserver.systemx.migrations',
        'agv05_webserver.systemx.tasks',
        'agv05_webserver.systemx.views',
    ],
    package_dir={'': 'src'},
    package_data={
        'agv05_webserver.system': [
            'fixtures/*.json',
        ],
    },
    requires=[
        'agv05_executive_msgs',
        'agv05_msgs',
        'capabilities',
        'diagnostic_msgs',
        'dynamic_reconfigure',
        'image_geometry',
        'linux_wifi',
        'rospkg',
        'rospy',
        'tf',
    ],
    classifiers=CLASSIFIERS,
)

setup(**d)
