from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import Cache, ExecutorMode

from .validator import Validator


class FmsSkillsetValidator(Validator):
    cache = Cache
    provides = ['fms_skillset']
    extra_provides = ['fms_skill_descriptions', 'fms_skill_descriptions_assoc', 'fms_register_list']

    def is_enabled(self, pre_validate=False):
        executor_mode = self.models.tmp['executor_mode'] if pre_validate else self.models.executor_mode
        return executor_mode == ExecutorMode.Standalone.value

    def validate(self):
        self.fms_skillset = {}  # fake it when we are in standalone mode
        self._extract_skillset()

    def assemble(self):
        self._extract_skillset()

    def _extract_skillset(self):
        self.fms_skill_descriptions = self.fms_skillset.get('skill_descriptions', [])
        self.fms_skill_descriptions_assoc = {d['id']: d for d in self.fms_skill_descriptions}
        self.fms_register_list = self.fms_skillset.get('register_list', [])
