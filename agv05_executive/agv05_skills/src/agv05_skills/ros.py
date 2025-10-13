from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
from dynamic_reconfigure.client import Client
import rospy
import six


class SetParameters(Skill):

    class Meta:
        name = 'Set Parameters'
        params = [
            {
                'name': 'component',
                'type': 'str',
                'description': 'The component name as shown in the system->parameters page.',
            },
            {
                'name': 'param_1',
                'type': 'str',
                'description': 'The 1st parameter to set. Put a dash ("-") to omit this parameter.',
                'default': '-',
            },
            {
                'name': 'value_1',
                'type': 'str',
                'description': 'The value to set to the 1st parameter.',
                'default': '-',
            },
            {
                'name': 'param_2',
                'type': 'str',
                'description': 'The 2nd parameter to set. Put a dash ("-") to omit this parameter.',
                'default': '-',
            },
            {
                'name': 'value_2',
                'type': 'str',
                'description': 'The value to set to the 2nd parameter.',
                'default': '-',
            },
            {
                'name': 'param_3',
                'type': 'str',
                'description': 'The 3rd parameter to set. Put a dash ("-") to omit this parameter.',
                'default': '-',
            },
            {
                'name': 'value_3',
                'type': 'str',
                'description': 'The value to set to the 3rd parameter.',
                'default': '-',
            },
            {
                'name': 'param_4',
                'type': 'str',
                'description': 'The 4th parameter to set. Put a dash ("-") to omit this parameter.',
                'default': '-',
            },
            {
                'name': 'value_4',
                'type': 'str',
                'description': 'The value to set to the 4th parameter.',
                'default': '-',
            },
        ]
        outcomes = ['Done', 'Failed']
        mutexes = []

    def __str__(self):
        return 'Setting parameter for "%s" node.' % self.component

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        config = {}

        self._add_param(config, self.param_1, self.value_1)
        self._add_param(config, self.param_2, self.value_2)
        self._add_param(config, self.param_3, self.value_3)
        self._add_param(config, self.param_4, self.value_4)

        client = None
        try:
            client = Client(self.component, timeout=0.5)
            client.update_configuration(config)
        except Exception as ex:
            rospy.logerr('Error setting "%s" node parameter: %s', self.component, ex)
            return 'Failed'
        else:
            return 'Done'
        finally:
            if client:
                client.close()

    def _add_param(self, config, param, value):
        if param != '-':
            # Note: we made a bold guess that all params will follow
            # ROS convention and use lower case only.
            param = param.replace(' ', '_').lower()
            config[param] = six.ensure_str(value)
