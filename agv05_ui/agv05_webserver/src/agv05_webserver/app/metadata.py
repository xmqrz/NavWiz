from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.utils.encoding import force_text
from itertools import chain
from rest_framework import metadata, serializers


class ReadOnlyShowChoicesMixin(object):

    def get_field_info(self, field):
        """
        Given an instance of a serializer field, return a dictionary
        of metadata about it.
        """
        field_info = OrderedDict()
        field_info['type'] = self.label_lookup[field]
        field_info['required'] = getattr(field, 'required', False)

        attrs = [
            'read_only', 'label', 'help_text',
            'min_length', 'max_length',
            'min_value', 'max_value'
        ]

        for attr in attrs:
            value = getattr(field, attr, None)
            if value is not None and value != '':
                field_info[attr] = force_text(value, strings_only=True)

        if getattr(field, 'child', None):
            field_info['child'] = self.get_field_info(field.child)
        elif getattr(field, 'fields', None):
            field_info['children'] = self.get_serializer_info(field)

        # NOTE: skip checking for read_only. (include choices in read_only field)
        if (not isinstance(field, (serializers.RelatedField, serializers.ManyRelatedField)) and
                hasattr(field, 'choices')):
            field_info['choices'] = [
                {
                    'value': choice_value,
                    'display_name': force_text(choice_name, strings_only=True)
                }
                for choice_value, choice_name in field.choices.items()
            ]

        return field_info


class SimpleMetadata(ReadOnlyShowChoicesMixin, metadata.SimpleMetadata):

    def get_field_info(self, field):
        field_info = super(SimpleMetadata, self).get_field_info(field)

        # add extra field info
        if hasattr(field, 'get_field_info') and callable(field.get_field_info):
            field_info = OrderedDict(chain(field_info.items(), field.get_field_info().items()))

        return field_info
