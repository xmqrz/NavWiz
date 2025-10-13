from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf.urls import url

from ..system.urls import sub_urlpatterns

urlpatterns = [vv for v in sub_urlpatterns.values() for vv in v]
