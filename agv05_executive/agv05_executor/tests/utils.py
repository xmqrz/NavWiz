from __future__ import absolute_import
from __future__ import unicode_literals

import sys
import unittest


__all__ = ['TestCase']


# References: https://stackoverflow.com/a/14107323/2312564

class _TracebackType(object):
    def __init__(self, tb_next, tb_frame, tb_lasti, tb_lineno):
        self.tb_next = tb_next
        self.tb_frame = tb_frame
        self.tb_lasti = tb_lasti
        self.tb_lineno = tb_lineno


def _expect(method):
    def expect(self, *args, **kwargs):
        try:
            method(self, *args, **kwargs)
        except self.failureException:
            exc = sys.exc_info()
            frame = exc[2].tb_frame.f_back
            tb = None
            while frame:
                tb_new = _TracebackType(tb, frame, frame.f_lasti, frame.f_lineno)
                if self._result._is_relevant_tb_level(tb_new):
                    break
                frame = frame.f_back
                tb = tb_new

            self._result.addFailure(self, (exc[0], exc[1], tb))
            failures = self._result.failures
            if len(failures) >= 2 and failures[-2][0] is self:
                _, msg = failures.pop()
                failures[-1] = (self, failures[-1][1] + '\n' + msg)
    return expect


class TestCase(unittest.TestCase):

    def run(self, result=None):
        self._result = result
        self._num_expections = 0
        super(TestCase, self).run(result)

    expectEqual = _expect(unittest.TestCase.assertEqual)
    expectIs = _expect(unittest.TestCase.assertIs)
    expectRaises = _expect(unittest.TestCase.assertRaises)
    expectRaisesRegexp = _expect(unittest.TestCase.assertRaisesRegexp)
