from __future__ import absolute_import

from .settings import *


TESTING = True

DATABASES['default']['ENGINE'] = 'django.db.backends.sqlite3'
DATABASES['alt']['ENGINE'] = 'django.db.backends.sqlite3'
DATABASES['alt']['TEST'] = {'MIRROR': 'default'}

CACHES['default']['BACKEND'] = 'django.core.cache.backends.locmem.LocMemCache'
CACHES['redis']['BACKEND'] = 'django.core.cache.backends.locmem.LocMemCache'
