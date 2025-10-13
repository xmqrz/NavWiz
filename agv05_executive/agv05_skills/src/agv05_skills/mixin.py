from __future__ import absolute_import
from __future__ import unicode_literals


class CancelForwardFlagOnPreempt(object):

    def request_preempt(self):
        super(CancelForwardFlagOnPreempt, self).request_preempt()
        self.robot.base.cancel_forward_flag()

    def wait_for_result(self, **kwargs):
        if self.preempt_requested():
            self.robot.base.cancel_forward_flag()
        return self.robot.base.wait_for_result(**kwargs)
