from __future__ import absolute_import
from __future__ import unicode_literals

from .module import Module


class SelfTest(Module):
    id = 'self-test'

    def __init__(self, *args, **kwargs):
        super(SelfTest, self).__init__(*args, **kwargs)

    def handle_in_pipe(self, data):
        pass
