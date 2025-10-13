from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.contrib.auth import get_user_model
from django.contrib.auth.models import Group
from django.db import transaction
from itertools import chain
from rest_framework import permissions, viewsets, status
from rest_framework.decorators import action
from rest_framework.response import Response
from rest_framework.reverse import reverse
import ujson as json

from agv05_webserver.system.models import Cache, MapChangeset, TaskTemplate, Variable
from agv05_webserver.system.signals import mark_models_cache_as_dirty, suppress_signals
from ...filters import TaskTemplateFilter
from ...serializers import (
    TaskTemplateEditSerializer,
    TaskTemplateListSerializer,
    TaskTemplateUserSerializer,
    TaskTemplateMassUpdateSerializer,
    TaskTemplateMassDeleteSerializer,
    TaskTemplateMassUserSerializer,
    TaskTemplateMassCategoryUpdateSerializer,
    VariableSerializer,
)
from ..mixin import CustomErrorMixin, Permission
from .fms_void import FmsVoidMixin
from .license_void import LicenseVoidMixin
from .mixin import VariableEndpointMixin

User = get_user_model()


class TaskTemplateConfigViewSet(CustomErrorMixin, VariableEndpointMixin, FmsVoidMixin, LicenseVoidMixin, viewsets.ModelViewSet):
    permission_classes = (Permission('system.view_system_panel'), permissions.DjangoModelPermissions)
    queryset = TaskTemplate.objects.all()
    serializer_class = TaskTemplateEditSerializer
    filter_class = TaskTemplateFilter

    def list(self, request, *args, **kwargs):
        # queryset = backend().filter_queryset(self.request, queryset, self)
        filter_options = self.get_filter_fields()
        response = super(TaskTemplateConfigViewSet, self).list(request, *args, **kwargs)
        return Response(OrderedDict(chain([
            ('meta', reverse('app:config-api:task-templates-meta', request=request)),
            ('global-param', reverse('app:config-api:task-templates-global-param', request=request)),
            ('variable', reverse('app:config-api:task-templates-variable', request=request)),
            ('filter_options', filter_options)
        ], response.data.items())))

    def get_filter_fields(self):
        filter = self.filter_class(self.request.query_params, queryset=self.get_queryset(), request=self.request)
        # TODO: refactor this magic
        category_chocies = filter.declared_filters['category'].extra['choices']()
        return OrderedDict([
            ('category', category_chocies),
            ('is_active', [
                ('true', 'Yes'),
                ('false', 'No'),
            ]),
            ('is_top_level', [
                ('true', 'Yes'),
                ('false', 'No'),
            ]),
        ])

    def get_serializer(self, *args, **kwargs):
        if kwargs.get('many'):
            kwargs.setdefault('context', self.get_serializer_context())
            return TaskTemplateListSerializer(*args, **kwargs)

        return super(TaskTemplateConfigViewSet, self).get_serializer(*args, **kwargs)

    @action(detail=True, methods=['GET'], url_path='last-cached', permission_classes=Permission('system.view_system_panel', 'system.change_tasktemplate'))
    def last_cached(self, request, *args, **kwargs):
        pk = int(kwargs.get('pk'))
        tt = Cache.get_task_template(template_id=pk)
        if not tt:
            self._custom_error(
                'Cached task template is unavailable.',
                status_code=status.HTTP_404_NOT_FOUND
            )

        return Response(self.get_serializer(
            TaskTemplate(
                id=tt['id'],
                name=tt['name'],
                metadata=json.dumps({
                    'params': tt['params'],
                    'outcomes': tt['outcomes'],
                    'create_suspended': tt['create_suspended'],
                }),
                structure=json.dumps({
                    'actions': tt['actions'],
                    'action_count': len(tt['actions']),
                }),
                is_top_level=tt['is_top_level'],
                is_active=True)
        ).data)

    @action(detail=False, methods=['GET'], permission_classes=Permission('system.view_system_panel', 'system.change_tasktemplate'))
    def meta(self, request, *args, **kwargs):
        initial = {}

        try:
            skillset = json.loads(Cache.get_models_skillset())
            initial['skill_descriptions'] = json.dumps(skillset['skill_descriptions'])
            initial['register_list'] = json.dumps(skillset['register_list'])
        except Exception:
            pass

        initial['reserved_global_params'] = json.dumps([{
            'name': 'agv_home',
            'type': 'Station',
        }])
        initial['global_params'] = '[]'
        try:
            global_params = Variable.objects.get(name=Variable.GLOBAL_PARAM).value
            if global_params:
                initial['global_params'] = global_params
        except Exception:
            pass

        initial['variables'] = '[]'
        try:
            variables = Variable.objects.get(name=Variable.VARIABLE).value
            if variables:
                initial['variables'] = variables
        except Exception:
            pass

        tasktemplate_metas = TaskTemplate.objects.values('id', 'name', 'metadata', 'is_active', 'is_top_level').order_by('is_top_level', '-is_active', 'name')
        initial['tasktemplate_metas'] = json.dumps(list(tasktemplate_metas))

        initial['station_list'] = '[]'
        try:
            stations = []
            active_map = [int(v) for v in Variable.objects.get(pk=Variable.ACTIVE_MAP).value.split(',')]
            for m in active_map:
                mc = MapChangeset.objects.filter(map_id=m).first()
                if mc:
                    stations += json.loads(mc.stations)
            stations.sort()
            initial['station_list'] = json.dumps(stations)
        except Exception:
            pass

        return Response(initial)

    @action(detail=True, methods=['POST'], serializer_class=TaskTemplateUserSerializer, permission_classes=Permission('system.view_system_panel', 'system.change_tasktemplate', 'system.view_users'))
    def users(self, request, *args, **kwargs):
        instance = self.get_object()
        serializer = self.get_serializer(instance, data=request.data, partial=True)
        serializer.is_valid(raise_exception=True)
        serializer.save()
        return Response(serializer.data)

    @users.mapping.get
    def get_users(self, request, *args, **kwargs):
        instance = self.get_object()
        return Response(self.get_serializer(instance).data)

    @action(detail=False, methods=['POST'], serializer_class=VariableSerializer, url_path='global-param', permission_classes=Permission('system.view_system_panel', 'system.change_tasktemplate'))
    def global_param(self, *args, **kwargs):
        return self._variable_endpoint(Variable.GLOBAL_PARAM, *args, **kwargs)

    @global_param.mapping.get
    def get_global_param(self, *args, **kwargs):
        return self._variable_get_endpoint(Variable.GLOBAL_PARAM, *args, **kwargs)

    @action(detail=False, methods=['POST'], serializer_class=VariableSerializer, permission_classes=Permission('system.view_system_panel', 'system.change_tasktemplate'))
    def variable(self, *args, **kwargs):
        return self._variable_endpoint(Variable.VARIABLE, *args, **kwargs)

    @variable.mapping.get
    def get_variable(self, *args, **kwargs):
        return self._variable_get_endpoint(Variable.VARIABLE, *args, **kwargs)

    def perform_create(self, serializer):
        # TODO: fix validation, cur can save empty name.
        instance = serializer.save(
            allowed_users=User.objects.filter(username__in=['agv_panel', 'agv_panel_pin_protected'])
        )

    def perform_update(self, serializer):
        overwrite = serializer.validated_data.get('overwrite')
        if not overwrite:
            instance = self.get_object()
            modified = serializer.validated_data.get('modified')
            if str(modified) != str(instance.modified):
                self._custom_error('Your changes are not saved. The content has been changed while you are editing it.', status_code=status.HTTP_409_CONFLICT)

        instance = serializer.save()

    @transaction.atomic
    @action(
        detail=False,
        methods=['POST'],
        serializer_class=TaskTemplateMassUpdateSerializer,
        url_path='mass-update',
        permission_classes=Permission('system.view_system_panel', 'system.change_tasktemplate')
    )
    def mass_update(self, *args, **kwargs):
        tt_list = self.request.query_params.get('tt')
        try:
            task_templates = self._get_tt_list(tt_list)
        except Exception as e:
            return Response({
                'message': str(e),
            }, status=status.HTTP_400_BAD_REQUEST)

        serializer = self.get_serializer(data=self.request.data)
        serializer.is_valid(raise_exception=True)
        with suppress_signals():
            for tt in task_templates.select_for_update():
                if 'is_active' in serializer.validated_data:
                    tt.is_active = serializer.validated_data['is_active']
                if 'is_top_level' in serializer.validated_data:
                    tt.is_top_level = serializer.validated_data['is_top_level']
                if 'metadata' in serializer.validated_data:
                    try:
                        metadata = json.loads(tt.metadata)
                        metadata['create_suspended'] = serializer.validated_data['metadata']
                        tt.metadata = json.dumps(metadata)
                    except Exception:
                        pass
                tt.save()

        mark_models_cache_as_dirty()
        return self.get_mass_update(*args, **kwargs)

    @mass_update.mapping.get
    def get_mass_update(self, *args, **kwargs):
        tt_list = self.request.query_params.get('tt')
        try:
            task_templates = self._get_tt_list(tt_list)
        except Exception as e:
            return Response({
                'message': str(e),
            }, status=status.HTTP_400_BAD_REQUEST)

        # NOTE: cannot use get_serializer due to overwrite behaviour
        serializer = TaskTemplateMassUpdateSerializer(task_templates, many=True)
        return Response(serializer.data)

    def perform_destroy(self, instance):
        instance.delete()

    @transaction.atomic
    @action(
        detail=False,
        methods=['POST'],
        serializer_class=TaskTemplateMassDeleteSerializer,
        url_path='mass-delete',
        permission_classes=Permission('system.view_system_panel', 'system.delete_tasktemplate')
    )
    def mass_delete(self, *args, **kwargs):
        tt_list = self.request.query_params.get('tt')
        try:
            task_templates = self._get_tt_list(tt_list)
        except Exception as e:
            return Response({
                'message': str(e),
            }, status=status.HTTP_400_BAD_REQUEST)

        with suppress_signals():
            for tt in task_templates.select_for_update():
                tt.delete()

        mark_models_cache_as_dirty()
        return Response({'result': True})

    @mass_delete.mapping.get
    def get_mass_delete(self, *args, **kwargs):
        tt_list = self.request.query_params.get('tt')
        try:
            task_templates = self._get_tt_list(tt_list)
        except Exception as e:
            return Response({
                'message': str(e),
            }, status=status.HTTP_400_BAD_REQUEST)

        # NOTE: cannot use get_serializer due to overwrite behaviour
        serializer = TaskTemplateMassDeleteSerializer(task_templates, many=True)
        return Response(serializer.data)

    @transaction.atomic
    @action(
        detail=False,
        methods=['POST'],
        serializer_class=TaskTemplateMassUserSerializer,
        url_path='mass-users',
        permission_classes=Permission(
            'system.view_system_panel',
            'system.change_tasktemplate',
            'system.view_users'
        )
    )
    def mass_users(self, *args, **kwargs):
        tt_list = self.request.query_params.get('tt')
        try:
            task_templates = self._get_tt_list(tt_list)
        except Exception as e:
            return Response({
                'message': str(e),
            }, status=status.HTTP_400_BAD_REQUEST)

        serializer = self.get_serializer(data=self.request.data)
        serializer.is_valid(raise_exception=True)
        allowed_users = serializer.validated_data['allowed_users']
        allowed_groups = serializer.validated_data['allowed_groups']
        with suppress_signals():
            for tt in task_templates.select_for_update():
                tt.allowed_users.set(allowed_users)
                tt.allowed_groups.set(allowed_groups)

        mark_models_cache_as_dirty()
        return self.get_mass_users(*args, **kwargs)

    @mass_users.mapping.get
    def get_mass_users(self, *args, **kwargs):
        tt_list = self.request.query_params.get('tt')
        try:
            task_templates = self._get_tt_list(tt_list)
        except Exception as e:
            return Response({
                'message': str(e),
            }, status=status.HTTP_400_BAD_REQUEST)

        # NOTE: cannot use get_serializer due to overwrite behaviour
        serializer = TaskTemplateMassUserSerializer(task_templates, many=True)
        return Response(OrderedDict([
            (
                'available_users',
                User.objects
                    .filter(is_staff=False)
                    .order_by('username')
                    .values_list('id', 'username'),
            ),
            (
                'available_groups',
                Group.objects.all().values_list('id', 'name'),
            ),
            (
                'task_templates',
                serializer.data,
            ),
        ]))

    @transaction.atomic
    @action(
        detail=False,
        methods=['POST'],
        serializer_class=TaskTemplateMassCategoryUpdateSerializer,
        url_path='mass-category-update',
        permission_classes=Permission('system.view_system_panel', 'system.change_tasktemplate')
    )
    def mass_category_update(self, *args, **kwargs):
        tt_list = self.request.query_params.get('tt')
        try:
            task_templates = self._get_tt_list(tt_list)
        except Exception as e:
            return Response({
                'message': str(e),
            }, status=status.HTTP_400_BAD_REQUEST)

        serializer = self.get_serializer(data=self.request.data)
        serializer.is_valid(raise_exception=True)
        task_templates.update(category=serializer.validated_data['category'])
        mark_models_cache_as_dirty()
        return self.get_mass_category_update(*args, **kwargs)

    @mass_category_update.mapping.get
    def get_mass_category_update(self, *args, **kwargs):
        tt_list = self.request.query_params.get('tt')
        try:
            task_templates = self._get_tt_list(tt_list)
        except Exception as e:
            return Response({
                'message': str(e),
            }, status=status.HTTP_400_BAD_REQUEST)

        # NOTE: cannot use get_serializer due to overwrite behaviour
        serializer = TaskTemplateMassCategoryUpdateSerializer(task_templates, many=True)
        return Response(serializer.data)

    def _get_tt_list(self, tt_list):
        if not tt_list:
            raise RuntimeError('Request must include tt query params.')
        try:
            tt_list = set(int(tid) for tid in tt_list.split(','))
        except Exception:
            raise RuntimeError('Provided tt query params is invalid.')

        task_templates = TaskTemplate.objects.filter(pk__in=tt_list)
        if len(task_templates) != len(tt_list):
            raise RuntimeError('Unable to find some of the task templates.')
        return task_templates
