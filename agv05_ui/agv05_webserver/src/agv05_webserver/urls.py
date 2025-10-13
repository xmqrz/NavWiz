"""agv05_webserver URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/1.9/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  url(r'^$', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  url(r'^$', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.conf.urls import url, include
    2. Add a URL to urlpatterns:  url(r'^blog/', include('blog.urls'))
"""
from __future__ import unicode_literals

from django.apps import apps
from django.conf import settings as django_settings
from django.conf.urls import include, url
from django.contrib import admin
from django.views.generic import TemplateView

urlpatterns = [
    # TODO: make this share with sveltekit base path. and use base path in svelte.
    # placeholder for pointing to config url
    url(r'^config$', TemplateView.as_view(template_name='404.html'), name='config-panel'),

    # nginx urls
    url(r'^logs/diagnostics/(?P<file_name>.+)$', TemplateView.as_view(template_name='404.html'), name='diagnostic-download'),
    url(r'^logs/main/(?P<file_name>.*)$', TemplateView.as_view(template_name='404.html'), name='main-log-download'),
    url(r'^logs/peripherals/(?P<file_name>.*)$', TemplateView.as_view(template_name='404.html'), name='peripheral-log-download'),

    url(r'^admin/', admin.site.urls),
    url(r'^hashicon/', include('agv05_webserver.hashicon.urls', namespace='hashicon')),
    url(r'^system/', include(
        'agv05_webserver.system.urls' if not django_settings.TRACKLESS else
        'agv05_webserver.systemx.urls',
        namespace='system')),
    url(r'', include(
        'agv05_webserver.app.urls' if not django_settings.TRACKLESS else
        'agv05_webserver.appx.urls',
        namespace='app'
    )),
]

if 'debug_toolbar' in [n.name for n in apps.get_app_configs()]:
    import debug_toolbar
    urlpatterns += [url(r'^__debug__/', include(debug_toolbar.urls))]
