from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
import requests
import rospy
import six
import ujson as json

VALID_STATUS_CODES = [409]


def verify_response(method, expected, actual):
    if method == 0:
        return expected == actual
    elif method <= 2:
        try:
            expected = json.loads(expected)
            actual = json.loads(actual)
            if expected == actual:
                return True
            elif method == 2:
                if isinstance(expected, dict) and isinstance(actual, dict):
                    return six.viewitems(expected) <= six.viewitems(actual)
                elif isinstance(expected, list) and isinstance(actual, list):
                    return all(x in actual for x in expected)
        except Exception as ex:
            pass
    return False


class HttpGet(Skill):

    class Meta:
        name = 'HTTP GET'
        params = [
            {
                'name': 'url',
                'type': 'str',
                'description': 'URL',
            },
            {
                'name': 'headers',
                'type': 'str',
                'description': 'Headers (in JSON format)',
                'default': '{}',
            },
            {
                'name': 'params',
                'type': 'str',
                'description': 'Parameters (in JSON format)',
                'default': '{}',
            },
            {
                'name': 'verify_ssl',
                'type': 'bool',
                'description': 'Enable SSL verification',
                'default': True,
            },
            {
                'name': 'response_verification',
                'type': 'int',
                'description': 'Response verification method (0-Exact, 1-JSON, 2-JSON partial match)',
                'default': 0,
                'min': 0,
                'max': 2,
            },
            {
                'name': 'expected_response',
                'type': 'str',
                'description': 'Expected response (in raw or JSON format)',
                'default': '{}',
            },
        ]
        outcomes = ['Success', 'Response Mismatch', 'Error']
        mutexes = []

    def __str__(self):
        return 'Performing HTTP GET request at %s' % self.url

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        try:
            headers = json.loads(self.headers)
            params = json.loads(self.params)
        except Exception as ex:
            self.fail('Malformed input in HttpGet action.')

        try:
            r = requests.get(self.url, headers=headers, params=params, verify=self.verify_ssl, timeout=(3.05, 7))
            if r.status_code not in VALID_STATUS_CODES:
                r.raise_for_status()
            if verify_response(self.response_verification, self.expected_response, r.content):
                return 'Success'
            else:
                return 'Response Mismatch'
        except Exception as ex:
            rospy.logerr('HTTP GET request error: %s', ex)
        return 'Error'


class HttpPost(Skill):

    class Meta:
        name = 'HTTP POST'
        params = [
            {
                'name': 'url',
                'type': 'str',
                'description': 'URL',
            },
            {
                'name': 'headers',
                'type': 'str',
                'description': 'Headers (in JSON format)',
                'default': '{}',
            },
            {
                'name': 'params',
                'type': 'str',
                'description': 'Parameters (in JSON format)',
                'default': '{}',
            },
            {
                'name': 'data',
                'type': 'str',
                'description': 'Data (in raw format)',
                'default': '{}',
            },
            {
                'name': 'verify_ssl',
                'type': 'bool',
                'description': 'Enable SSL verification',
                'default': True,
            },
            {
                'name': 'response_verification',
                'type': 'int',
                'description': 'Response verification method (0-Exact, 1-JSON, 2-JSON partial match)',
                'default': 0,
                'min': 0,
                'max': 2,
            },
            {
                'name': 'expected_response',
                'type': 'str',
                'description': 'Expected response (in raw or JSON format)',
                'default': '{}',
            },
        ]
        outcomes = ['Success', 'Response Mismatch', 'Error']
        mutexes = []

    def __str__(self):
        return 'Performing HTTP POST request at %s' % self.url

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        try:
            headers = json.loads(self.headers)
            params = json.loads(self.params)
        except Exception as ex:
            self.fail('Malformed input in HttpPost action.')

        try:
            r = requests.post(self.url, headers=headers, params=params, data=self.data, verify=self.verify_ssl, timeout=(3.05, 7))
            if r.status_code not in VALID_STATUS_CODES:
                r.raise_for_status()
            if verify_response(self.response_verification, self.expected_response, r.content):
                return 'Success'
            else:
                return 'Response Mismatch'
        except Exception as ex:
            rospy.logerr('HTTP POST request error: %s', ex)
        return 'Error'


class HttpPut(Skill):

    class Meta(HttpPost.Meta):
        name = 'HTTP PUT'

    def __str__(self):
        return 'Perform HTTP PUT request at %s' % self.url

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        try:
            headers = json.loads(self.headers)
            params = json.loads(self.params)
        except Exception as ex:
            self.fail('Malformed input in HttpPut action.')

        try:
            r = requests.put(self.url, headers=headers, params=params, data=self.data, verify=self.verify_ssl, timeout=(3.05, 7))
            if r.status_code not in VALID_STATUS_CODES:
                r.raise_for_status()
            if verify_response(self.response_verification, self.expected_response, r.content):
                return 'Success'
            else:
                return 'Response Mismatch'
        except Exception as ex:
            rospy.logerr('HTTP PUT request error: %s', ex)
        return 'Error'


class HttpDelete(Skill):

    class Meta(HttpGet.Meta):
        name = 'HTTP DELETE'

    def __str__(self):
        return 'Perform HTTP DELETE request at %s' % self.url

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        try:
            headers = json.loads(self.headers)
            params = json.loads(self.params)
        except Exception as ex:
            self.fail('Malformed input in HttpDelete action.')

        try:
            r = requests.delete(self.url, headers=headers, params=params, verify=self.verify_ssl, timeout=(3.05, 7))
            if r.status_code not in VALID_STATUS_CODES:
                r.raise_for_status()
            if verify_response(self.response_verification, self.expected_response, r.content):
                return 'Success'
            else:
                return 'Response Mismatch'
        except Exception as ex:
            rospy.logerr('HTTP DELETE request error: %s', ex)
        return 'Error'
