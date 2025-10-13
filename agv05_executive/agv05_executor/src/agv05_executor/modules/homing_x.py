from __future__ import absolute_import
from __future__ import unicode_literals

from .homing import Homing


class HomingX(Homing):
    homing_skill_id = 'agv05_skills.plan_x.NavigateToX'
    homing_skill_param_defaults = {
        'align_station_type': -1,
        'next_motion': 0,
        'next_speed': 0.0,
        'sense_line': 0,
    }
