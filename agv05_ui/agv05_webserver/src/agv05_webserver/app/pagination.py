from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.http import Http404
from django.utils import timezone
from rest_framework import pagination
from rest_framework.compat import coreapi, coreschema
from rest_framework.response import Response
from rest_framework.utils.urls import replace_query_param


class LimitOffsetPagination(pagination.LimitOffsetPagination):
    default_limit = 50
    max_limit = 100


class MonthPagination(pagination.BasePagination):
    month_query_param = 'month'
    month_query_format = '%Y-%m'
    month_query_description = 'The month from which to return the results.'

    date_field = 'date'

    def paginate_queryset(self, queryset, request, view=None):
        self.queryset = queryset
        self.request = request
        self.view = view
        self.month = self.get_month(request)
        lookup = {
            '%s__gte' % self.date_field: self.month,
            '%s__lt' % self.date_field: self._get_next_month(self.month),
        }
        return queryset.filter(**lookup)

    def get_paginated_response(self, data):
        r = OrderedDict()
        r['current'] = self.get_current_link()
        r['next'] = self.get_next_link()
        r['previous'] = self.get_previous_link()
        r['results'] = data
        try:
            r.update(self.view._get_extra())
        except Exception:
            pass
        return Response(r)

    def get_month(self, request):
        month = request.query_params.get(self.month_query_param)
        if not month:
            return self._get_current_month(timezone.localdate())
        try:
            return timezone.datetime.strptime(month, self.month_query_format).date()
        except ValueError:
            raise Http404()

    def get_current_link(self):
        url = self.request.build_absolute_uri()
        return replace_query_param(url, self.month_query_param, self.month.strftime(self.month_query_format))

    def get_next_link(self):
        next_month = self._get_next_month(self.month)
        today = timezone.localdate()
        if next_month > today:
            return None
        url = self.request.build_absolute_uri()
        return replace_query_param(url, self.month_query_param, next_month.strftime(self.month_query_format))

    def get_previous_link(self):
        lookup = {
            '%s__lt' % self.date_field: self.month,
        }
        if not self.queryset.filter(**lookup).exists():
            return None
        prev_month = self._get_previous_month(self.month)
        url = self.request.build_absolute_uri()
        return replace_query_param(url, self.month_query_param, prev_month.strftime(self.month_query_format))

    def _get_next_month(self, date):
        if date.month == 12:
            return date.replace(year=date.year + 1, month=1, day=1)
        else:
            return date.replace(month=date.month + 1, day=1)

    def _get_current_month(self, date):
        return date.replace(day=1)

    def _get_previous_month(self, date):
        return self._get_current_month(date - timezone.timedelta(days=1))

    def get_schema_fields(self, view):
        assert coreapi is not None, 'coreapi must be installed to use `get_schema_fields()`'
        assert coreschema is not None, 'coreschema must be installed to use `get_schema_fields()`'
        fields = [
            coreapi.Field(
                name=self.month_query_param,
                required=False,
                location='query',
                schema=coreschema.String(
                    title='Month',
                    description=self.month_query_description
                )
            )
        ]
        return fields


class YearPagination(pagination.BasePagination):
    year_query_param = 'year'
    year_query_format = '%Y'
    year_query_description = 'The year from which to return the results.'

    date_field = 'date'

    def paginate_queryset(self, queryset, request, view=None):
        self.queryset = queryset
        self.request = request
        self.view = view
        self.year = self.get_year(request)
        lookup = {
            '%s__gte' % self.date_field: self.year,
            '%s__lt' % self.date_field: self._get_next_year(self.year),
        }
        return queryset.filter(**lookup)

    def get_paginated_response(self, data):
        r = OrderedDict()
        r['current'] = self.get_current_link()
        r['next'] = self.get_next_link()
        r['previous'] = self.get_previous_link()
        r['results'] = data
        try:
            r.update(self.view._get_extra())
        except Exception:
            pass
        return Response(r)

    def get_year(self, request):
        year = request.query_params.get(self.year_query_param)
        if not year:
            return self._get_current_year(timezone.localdate())
        try:
            return timezone.datetime.strptime(year, self.year_query_format).date()
        except ValueError:
            raise Http404()

    def get_current_link(self):
        url = self.request.build_absolute_uri()
        return replace_query_param(url, self.year_query_param, self.year.strftime(self.year_query_format))

    def get_next_link(self):
        next_year = self._get_next_year(self.year)
        today = timezone.localdate()
        if next_year > today:
            return None
        url = self.request.build_absolute_uri()
        return replace_query_param(url, self.year_query_param, next_year.strftime(self.year_query_format))

    def get_previous_link(self):
        lookup = {
            '%s__lt' % self.date_field: self.year,
        }
        if not self.queryset.filter(**lookup).exists():
            return None
        prev_year = self._get_previous_year(self.year)
        url = self.request.build_absolute_uri()
        return replace_query_param(url, self.year_query_param, prev_year.strftime(self.year_query_format))

    def _get_next_year(self, date):
        return date.replace(year=date.year + 1)

    def _get_current_year(self, date):
        return date.replace(month=1, day=1)

    def _get_previous_year(self, date):
        return self._get_current_year(date - timezone.timedelta(days=365))

    def get_schema_fields(self, view):
        assert coreapi is not None, 'coreapi must be installed to use `get_schema_fields()`'
        assert coreschema is not None, 'coreschema must be installed to use `get_schema_fields()`'
        fields = [
            coreapi.Field(
                name=self.year_query_param,
                required=False,
                location='query',
                schema=coreschema.String(
                    title='Year',
                    description=self.year_query_description
                )
            )
        ]
        return fields
