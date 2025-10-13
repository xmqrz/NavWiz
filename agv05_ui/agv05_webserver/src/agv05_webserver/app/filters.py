from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
try:
    from urllib.parse import urlencode
except Exception:
    from urllib import urlencode  # noqa
from django_filters import rest_framework as filters
from django_filters import widgets

from agv05_webserver.system.models import TaskTemplate


class TaskTemplateFilter(filters.FilterSet):

    def _get_category_choices():
        categories = TaskTemplate.objects.filter(category__isnull=False) \
            .values_list('category', flat=True).order_by().distinct()
        return [(c, c) for c in categories]

    category = filters.ChoiceFilter(choices=_get_category_choices)
    is_active = filters.BooleanFilter(widget=widgets.BooleanWidget)
    is_top_level = filters.BooleanFilter(widget=widgets.BooleanWidget)

    class Meta:
        model = TaskTemplate
        fields = ('category', 'is_active', 'is_top_level')

    # TODO: identify whcih one not use and remove. needed to get the query string options available
    @property
    def query_string(self):
        if not hasattr(self, '_query_string'):
            self._query_string = self.get_query_string()
        return self._query_string

    @property
    def query_string_updates(self):
        if not hasattr(self, '_query_string_updates'):
            updates = OrderedDict()
            for name, field in self.form.fields.items():
                updates[name] = OrderedDict()
                for k, v in field.widget.choices:
                    if k is not None and k != '':
                        updates[name][v] = self.get_query_string(**{name: k})
            self._query_string_updates = updates
        return self._query_string_updates

    @property
    def query_string_removes(self):
        if not hasattr(self, '_query_string_removes'):
            removes = OrderedDict()
            for name in self.filters:
                removes[name] = self.get_query_string(**{name: None})
            self._query_string_removes = removes
        return self._query_string_removes

    def get_query_string(self, **kwargs):
        queries = OrderedDict()
        for name in self.filters:
            if name in kwargs:
                value = kwargs.get(name)
            elif self.is_bound:
                value = self.form.cleaned_data.get(name)
            else:
                continue

            if value is not None and value != '':
                queries[name] = value
        return urlencode(queries)
