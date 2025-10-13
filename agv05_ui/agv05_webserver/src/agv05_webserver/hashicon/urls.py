from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf.urls import url

from .views import IndexView

urlpatterns = [
    url(r'^(?P<string>.*)$', IndexView.as_view(), name='index'),
]
