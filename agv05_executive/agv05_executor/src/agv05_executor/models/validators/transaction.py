from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, ExecutorMode

from .validator import Validator


class TransactionValidator(Validator):
    cache = Cache
    provides = ['transaction_enabled']
    extra_provides = []

    def is_enabled(self, pre_validate=False):
        executor_mode = self.models.tmp['executor_mode'] if pre_validate else self.models.executor_mode
        return executor_mode == ExecutorMode.Standalone.value

    def validate(self):
        self.transaction_enabled = False  # fake it when we are in standalone mode

    def assemble(self):
        pass
