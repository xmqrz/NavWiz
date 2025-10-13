from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.conf.urls import url

from .views import LoginView, LogoutView, AuthView, AgvPanelLogin, HwAppView, DashboardView

sub_urlpatterns = OrderedDict()

sub_urlpatterns['auth'] = [
    url(r'^login$', LoginView.as_view(), name='login'),
    url(r'^logout$', LogoutView.as_view(), name='logout'),
    url(r'^auth$', AuthView.as_view(), name='auth'),
    url(r'^agv-panel-login$', AgvPanelLogin.as_view(), name='agv-panel-login'),
]

sub_urlpatterns['hw_app'] = [
    url(r'^hw-app/(?P<app_id>[\w._-]+)/(?P<asset>.+)$', HwAppView.as_view(), name='hw-app'),
]

sub_urlpatterns['dashboard'] = [
    url(r'^dashboard/(?P<asset>.+)$', DashboardView.as_view(), name='dashboard'),
]

urlpatterns = [vv for v in sub_urlpatterns.values() for vv in v]
