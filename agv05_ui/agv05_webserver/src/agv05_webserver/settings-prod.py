from __future__ import absolute_import

import subprocess

from .settings import *


# Read secret key from envvar (min length: 45)
SECRET_KEY = '+46x0_))^7rj!=hhfgg6c_qqi2m@gu%n6ff84vq*0p^d#wgqb='
k = os.getenv('SECRET_KEY', '')
if len(k) >= 45:
    SECRET_KEY = k

# Read DEBUG flag from envvar
DEBUG = False
try:
    DEBUG = distutils.util.strtobool(os.environ['DEBUG'])
except Exception:
    pass

# No auto poweroff
NO_AUTO_POWER_OFF = False
try:
    NO_AUTO_POWER_OFF = distutils.util.strtobool(os.environ['NO_AUTO_POWER_OFF'])
except Exception:
    pass

ALLOWED_HOSTS = ['*']


# Cookies
try:
    with open('/etc/machine-id') as f:
        d = f.read().strip()[-4:]
except Exception:
    d = ''
p = '/sys/class/dmi/id/product_uuid'
if os.stat(p).st_mode & 0x04 == 0:
    subprocess.check_call(['sudo', '-n', 'chmod', '0444', p])
with open(p) as f:
    cookie_prefix = (f.read().strip() + d)[-8:]

CSRF_COOKIE_NAME = ('%s_agv05_csrftoken' if not TRACKLESS else '%s_agv05x_csrftoken') % cookie_prefix
LANGUAGE_COOKIE_NAME = ('%s_agv05_django_language' if not TRACKLESS else '%s_agv05x_django_language') % cookie_prefix
SESSION_COOKIE_NAME = ('%s_agv05_sessionid' if not TRACKLESS else '%s_agv05x_sessionid') % cookie_prefix


# Read FORCE_SCRIPT_NAME from envvar
force_script_name = os.getenv('FORCE_SCRIPT_NAME')
if force_script_name and force_script_name != '/':
    FORCE_SCRIPT_NAME = force_script_name
    STATIC_URL = FORCE_SCRIPT_NAME + '/static/'


# Django Crispy Forms
CRISPY_FAIL_SILENTLY = not DEBUG


# Django Sendfile
SENDFILE_BACKEND = 'sendfile.backends.nginx'
