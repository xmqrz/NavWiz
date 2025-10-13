from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
from diagnostic_msgs.msg import DiagnosticArray
import rospy


class CheckDiagnosticStatus(Skill):

    class Meta:
        name = 'Check Diagnostic Status'
        params = [
            {
                'name': 'warn_list',
                'type': 'vstr',
                'description': 'A string variable to receive a comma-separated list of nodes with warning status',
            },
            {
                'name': 'error_list',
                'type': 'vstr',
                'description': 'A string variable to receive a comma-separated list of nodes with error status',
            },
            {
                'name': 'stale_list',
                'type': 'vstr',
                'description': 'A string variable to receive a comma-separated list of stale nodes',
            },
        ]
        outcomes = ['Ok', 'Warn', 'Error', 'Stale', 'Timeout']
        mutexes = []

    def __str__(self):
        return 'Checking diagnostic status'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        try:
            msg = rospy.wait_for_message('/diagnostics_agg', DiagnosticArray, 2.0)
        except Exception:
            return 'Timeout'

        warn_list = []
        error_list = []
        stale_list = []
        for diag in msg.status:
            if diag.level == diag.STALE:
                name = self._strip_name(diag.name)
                if name:
                    stale_list.append(name)
            elif diag.level == diag.ERROR:
                name = self._strip_name(diag.name)
                if name:
                    error_list.append(name)
            elif diag.level == diag.WARN:
                name = self._strip_name(diag.name)
                if name:
                    warn_list.append(name)

        warn_list = sorted(set(warn_list))
        error_list = sorted(set(error_list))
        stale_list = sorted(set(stale_list))

        self.warn_list.value = ', '.join(warn_list)
        self.error_list.value = ', '.join(error_list)
        self.stale_list.value = ', '.join(stale_list)

        if stale_list:
            return 'Stale'
        elif error_list:
            return 'Error'
        elif warn_list:
            return 'Warn'
        return 'Ok'

    def _strip_name(self, name):
        try:
            return name.split('/')[2]
        except Exception:
            return ''
