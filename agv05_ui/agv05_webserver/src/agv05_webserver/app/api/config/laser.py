from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Parameter
from collections import OrderedDict
from django.http import Http404
from itertools import chain
from rest_framework.response import Response
import logging
import rospy
import six
import ujson as json

from .parameter import ParameterConfigViewSet

logger = logging.getLogger(__name__)


def get_topics_list(parameter_name):
    param = rospy.get_param(parameter_name, '')
    if isinstance(param, str):
        return [t.strip() for t in param.split(' ') if t and t.strip()]
    if not isinstance(param, dict):
        return []

    topics = []
    _flatten_dict_topics(param, topics)
    return topics


def _flatten_dict_topics(tree, topics, prefix=''):
    for key, value in six.iteritems(tree):
        if isinstance(value, str):
            topics.append(prefix + key)
        elif isinstance(value, dict):
            _flatten_dict_topics(value, topics, prefix + key + '/')


class LaserConfigViewSet(ParameterConfigViewSet):
    laser_component = '/agv05_lidar'

    def list(self, request, *args, **kwargs):
        # Note: show detail (similar to retrieve)
        instance = self.get_object()
        serializer = self.get_serializer(instance)

        laser_topics = {
            'lidar_1_topics': get_topics_list('/primary_obstacle_scan_topics'),
            'lidar_2_topics': get_topics_list('/secondary_obstacle_scan_topics'),
            'lidar_3_topics': get_topics_list('/tertiary_obstacle_scan_topics'),
            'lidar_4_topics': get_topics_list('/quaternary_obstacle_scan_topics'),
            'lidar_5_topics': get_topics_list('/quinary_obstacle_scan_topics'),
        }

        return Response(OrderedDict(chain([
            ('laser_topics', json.dumps(laser_topics)),
        ], serializer.data.items())))

    def get_serializer(self, instance=None, *args, **kwargs):
        if not instance:
            instance = self.get_object()
        kwargs['many'] = False
        return super(LaserConfigViewSet, self).get_serializer(instance, *args, **kwargs)

    def create(self, request):
        # Note: perform_update (similar to update method)
        instance = self.get_object()
        serializer = self.get_serializer(instance, data=request.data)
        serializer.is_valid(raise_exception=True)
        self.perform_update(serializer)

        # NOTE: use data from dyncfg_client, saved data might be modified.
        return self.list(request)

    def get_object(self):
        try:
            return Parameter.objects.get(key=self.laser_component)
        except Parameter.DoesNotExist:
            return Parameter(key=self.laser_component)

    def retrieve(self, request, *args, **kwargs):
        raise Http404()

    def update(self, request, pk=None):
        raise Http404()

    def partial_update(self, request, pk=None):
        raise Http404()

    def destroy(self, request, pk=None):
        raise Http404()
