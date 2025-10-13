from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Parameter
from collections import OrderedDict
from django.conf import settings as django_settings
from django.contrib.auth import get_user_model
from django.http import Http404
from rest_framework import viewsets, mixins, status, serializers
from rest_framework.decorators import action
from rest_framework.response import Response
import dynamic_reconfigure
import dynamic_reconfigure.client
import logging
import six
import ujson as json
import yaml
try:
    from yaml import full_load
except Exception:
    from yaml import load as full_load

from ...serializers import (
    ParameterListSerializer,
    ParameterSerializer,
    parameterserializer_factory,
)
from ..mixin import CustomErrorMixin, NoPaginationMixin, Permission, AltModelPermissions
from .license_void import LicenseVoidMixin

User = get_user_model()
logger = logging.getLogger(__name__)


class MuteSerializer(serializers.Serializer):
    pass


class ParameterMixin(LicenseVoidMixin):
    permission_classes = (
        Permission('system.view_system_panel'),
        Permission('system.change_protected_parameter', methods='DELETE'),
        AltModelPermissions,
    )

    def dispatch(self, request, *args, **kwargs):
        response = super(ParameterMixin, self).dispatch(request, *args, **kwargs)
        if getattr(self, '_dyncfg_client', None):
            self._dyncfg_client.close()
        return response

    def get_online_components(self):
        if not hasattr(self, '_online_comps'):
            try:
                self._online_comps = dynamic_reconfigure.find_reconfigure_services()
            except Exception as ex:
                logger.warn('Error querying dynamic reconfigure services: %s', ex)
                self._online_comps = []
        return self._online_comps

    def is_component_online(self, component):
        return component in self.get_online_components()

    def get_dyncfg_client(self, component, reload=False):
        if reload or not hasattr(self, '_dyncfg_client'):
            if getattr(self, '_dyncfg_client', None):
                self._dyncfg_client.close()

            if self.is_component_online(component):
                try:
                    self._dyncfg_client = dynamic_reconfigure.client.Client(component, timeout=0.5)
                except Exception as ex:
                    logger.warn('Error creating dynamic reconfigure client: %s', ex)
                    self._dyncfg_client = None
            else:
                self._dyncfg_client = None
        return self._dyncfg_client


class ParameterConfigViewSet(
    NoPaginationMixin,
    CustomErrorMixin,
    ParameterMixin,
    mixins.RetrieveModelMixin,
    mixins.UpdateModelMixin,
    mixins.DestroyModelMixin,
    mixins.ListModelMixin,
    viewsets.GenericViewSet
):
    queryset = Parameter.objects.all()
    serializer_class = ParameterSerializer
    lookup_value_regex = r'[\w\/]+'

    exclude_params = [django_settings.AGV05_AUDIO_PLAYER]
    control_params = ['/amcl', '/dev_mode_server', '/human_follower', '/marker_localization', '/wifi_manager']

    def get_object(self):
        lookup_url_kwarg = self.lookup_url_kwarg or self.lookup_field
        key = self.kwargs[lookup_url_kwarg]
        if not key.startswith('/'):
            self.kwargs[lookup_url_kwarg] = '/%s' % key

        # prevent request excluded params
        if self.kwargs[lookup_url_kwarg] in self.exclude_params:
            raise Http404()

        return super(ParameterConfigViewSet, self).get_object()

    def list(self, request, *args, **kwargs):
        response = super(ParameterConfigViewSet, self).list(request, *args, **kwargs)
        results = OrderedDict()

        results['active_components'] = OrderedDict([
            ('controllers', []),
            ('hardware', []),
        ])
        results['offline_components'] = OrderedDict([
            ('controllers', []),
            ('hardware', []),
        ])
        for obj in response.data:
            if obj['key'] in self.exclude_params:
                continue
            if obj['key'].startswith('/agv05_') or obj['key'].startswith('/agv05x_') or obj['key'] in self.control_params:
                if self.is_component_online(obj['key']):
                    results['active_components']['controllers'].append(obj)
                else:
                    results['offline_components']['controllers'].append(obj)
            else:
                if self.is_component_online(obj['key']):
                    results['active_components']['hardware'].append(obj)
                else:
                    results['offline_components']['hardware'].append(obj)

        return Response(OrderedDict([
            # ('meta', reverse('app:config-api:parameters-meta', request=request)),
            ('results', results)
        ]))

    def get_serializer(self, instance=None, *args, **kwargs):
        if kwargs.get('many'):
            kwargs['instance'] = instance
            kwargs.setdefault('context', self.get_serializer_context())
            return ParameterListSerializer(*args, **kwargs)

        if self.get_serializer_class() == MuteSerializer:
            return MuteSerializer()

        if not instance:
            instance = self.get_object()

        serializer_class = self.generate_serialzier_class(instance)
        kwargs.setdefault('context', self.get_serializer_context())
        initial = self.get_initial(instance, serializer_class)
        kwargs.setdefault('instance_data', initial)
        serializer = serializer_class(instance=instance, *args, **kwargs)
        return serializer

    def update(self, request, *args, **kwargs):
        partial = kwargs.pop('partial', False)
        instance = self.get_object()
        serializer = self.get_serializer(instance, data=request.data, partial=partial)
        serializer.is_valid(raise_exception=True)
        warn = self.perform_update(serializer)
        if warn:
            return Response({'detail': warn}, status=status.HTTP_202_ACCEPTED)

        # NOTE: use data from dyncfg_client, saved data might be modified.
        return self.retrieve(request, *args, **kwargs)

    def get_initial(self, obj, serializer_class):
        initial = {}
        client = self.get_dyncfg_client(obj.key)
        if client:
            config = client.get_configuration(timeout=0.5)
            if config:
                initial = dict(config)
                # Todo: handle initial group states
                if 'groups' in initial:
                    del initial['groups']

        if not initial:
            try:
                config_msg = full_load(obj.value)
                initial = {c['name']: c['value'] for c in config_msg['bools'] + config_msg['ints'] + config_msg['strs'] + config_msg['doubles']}
            except Exception:
                pass

        # patch initial data
        initial['key'] = obj.key
        initial['layout'] = json.dumps(serializer_class.LAYOUT)
        return initial

    def generate_serialzier_class(self, obj):
        serializer_class = None
        client = self.get_dyncfg_client(obj.key)
        protected = self.request.user.has_perm('system.change_protected_parameter')
        protected = protected and 'protected' in self.request.GET
        if client:
            group_description = client.get_group_descriptions(timeout=0.5)
            if group_description:
                if not django_settings.TRACKLESS and obj.key == django_settings.AGV05_EXECUTOR:
                    # Hack: Hide Mapping and Navigate To configurations if it is not trackless mode
                    try:
                        groups = group_description['groups']
                        groups.pop('Mapping')
                        groups.pop('Navigate_To')
                    except Exception:
                        pass
                serializer_class = parameterserializer_factory(desc=group_description, allow_protected=protected)

        if not serializer_class:
            config_msg = None
            try:
                config_msg = full_load(obj.value)
            except Exception:
                pass
            serializer_class = parameterserializer_factory(config_msg=config_msg, allow_protected=protected)

        return serializer_class

    def perform_update(self, serializer):
        # NOTE return string if got warning
        client = self.get_dyncfg_client(serializer.instance.key)
        if client:
            # update to dynamic reconfigure server
            config = {}

            for k in serializer.validated_data:
                # exclude group state due to a bug in python-based dynamic reconfigure server.
                # Todo: update group states too
                if k == 'groups':
                    continue

                config[k] = serializer.validated_data[k]

            try:
                client.update_configuration(config)
            except Exception:
                pass
            else:
                return

        # update to database since dynamic reconfigure server is away
        serializer.instance = self.get_object()
        config_msg = {}
        try:
            config_msg = full_load(serializer.instance.value)
        except Exception:
            pass

        if config_msg:
            # Todo: update group states too
            try:
                bools = config_msg['bools']
                for c in bools:
                    if c['name'] in serializer.validated_data:
                        c['value'] = serializer.validated_data[c['name']]
            except Exception:
                pass

            try:
                ints = config_msg['ints']
                for c in ints:
                    if c['name'] in serializer.validated_data:
                        c['value'] = serializer.validated_data[c['name']]
            except Exception:
                pass

            try:
                strs = config_msg['strs']
                for c in strs:
                    if c['name'] in serializer.validated_data:
                        c['value'] = six.ensure_str(serializer.validated_data[c['name']])
            except Exception:
                pass

            try:
                doubles = config_msg['doubles']
                for c in doubles:
                    if c['name'] in serializer.validated_data:
                        c['value'] = serializer.validated_data[c['name']]
            except Exception:
                pass

            serializer.instance.value = yaml.dump(config_msg)
            serializer.instance.save()

        if client:
            return 'The component "%s" appears to be active but the system is unable notify it about the parameter update. Saving to database only.' % serializer.instance

    def perform_destroy(self, instance):
        if self.is_component_online(instance.key):
            self._custom_error('Cannot delete active component "%s".' % instance, status_code=status.HTTP_405_METHOD_NOT_ALLOWED)
        instance.delete()

    @action(detail=True, methods=['POST'], serializer_class=MuteSerializer, permission_classes=Permission('system.view_system_panel', 'system.change_parameter'))
    def reset(self, request, *args, **kwargs):
        instance = self.get_object()
        protected = request.user.has_perm('system.change_protected_parameter')
        client = self.get_dyncfg_client(instance.key)
        if client:
            # update to dynamic reconfigure server
            param_desc = client.get_parameter_descriptions(timeout=0.5)

            if param_desc:
                config = {}

                for d in param_desc:
                    # exclude group state due to a bug in python-based dynamic reconfigure server.
                    # Todo: reset group states too
                    if d['name'] == 'groups':
                        continue

                    if not protected and not d['name'].endswith('_'):
                        continue

                    config[d['name']] = d['default']

                try:
                    client.update_configuration(config)
                except Exception:
                    pass
                else:
                    return Response(OrderedDict([
                        ('status', 'ok'),
                        ('message', 'Parameter component "%s" reset successfully' % instance.key),
                    ]))

        # update doesn't happen, display error message
        self._custom_error('Failed to reset parameter component "%s" because the component has gone away.' % instance, status_code=status.HTTP_405_METHOD_NOT_ALLOWED)
