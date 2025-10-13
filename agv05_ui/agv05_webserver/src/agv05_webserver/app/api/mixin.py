from __future__ import absolute_import
from __future__ import unicode_literals

from rest_framework import exceptions, permissions
from rest_framework.pagination import LimitOffsetPagination


class CustomErrorMixin(object):

    def _custom_error(self, detail=None, code=None, status_code=500):
        exc = exceptions.APIException(detail, code)
        exc.status_code = status_code
        raise exc


class Permission(permissions.BasePermission):

    def __init__(self, *perms, **kwargs):
        self.permission_required = perms
        methods = kwargs.get('methods')
        if methods and not isinstance(methods, (list, tuple)):
            methods = (methods, )
        self.methods = methods

    def __call__(self):
        return self

    def __iter__(self):
        yield self

    def has_permission(self, request, view):
        if self.methods and request.method not in self.methods:
            return True
        return request.user.has_perms(self.permission_required)


class AltModelPermissions(permissions.DjangoModelPermissions):
    # change post permission to change model.
    perms_map = {
        'GET': [],
        'OPTIONS': [],
        'HEAD': [],
        'POST': ['%(app_label)s.change_%(model_name)s'],
        'PUT': ['%(app_label)s.change_%(model_name)s'],
        'PATCH': ['%(app_label)s.change_%(model_name)s'],
        'DELETE': ['%(app_label)s.delete_%(model_name)s'],
    }


class NoPagination(LimitOffsetPagination):
    page_size = None


class NoPaginationMixin(object):
    pagination_class = NoPagination
