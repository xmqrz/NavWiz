from __future__ import absolute_import

from .settings import *


INSTALLED_APPS += ['debug_toolbar']

MIDDLEWARE += ['debug_toolbar.middleware.DebugToolbarMiddleware']


# Logging
LOGGING['handlers']['console']['level'] = 'DEBUG'
LOGGING['loggers']['agv05_webserver']['level'] = 'DEBUG'


# Django Debug Toolbar
DEBUG_TOOLBAR_CONFIG = {
    # 'JQUERY_URL': '',
}
