from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.models import Models
from agv05_executor.models.validators.map import ConstrainedMapValidator
from agv05_executor.register import RegisterManager
from agv05_executor.robots.tracked_sim import TrackedSim
from agv05_executor.skill import SkillManager
from agv05_executor.variable import VariableManager
from agv05_webserver.system.models import Cache, ExecutorMode, Variable
from django.contrib.auth import get_user_model
import agv05_webserver.system.views
import os
import ujson as json


__all__ = ['setup_robots', 'reconstruct_line_graph']

__dir__ = os.path.dirname(os.path.abspath(__file__))

User = get_user_model()


def load_agv(allowed_motions=None):
    Variable.objects.update_or_create(name='agv_name', defaults={
        'value': 'Test AGV',
    })
    Variable.objects.update_or_create(name='agv_home', defaults={
        'value': 'Home',
    })
    Variable.objects.update_or_create(name='executor_mode', defaults={
        'value': ExecutorMode.Standalone.value,
    })

    executor_cfg = dict(
        dimension={
            'body': {
                'template': 'zalpha',
                'variation': 'extended',
                'length': 1.03,
                'width': 0.55,
                'vcenter': 0.51,
                'hcenter': 0.275,
                'svgPath': ('M0.275 -0.47 V0.46 l-0.12 0.05 H-0.155 l-0.12 -0.05 V-0.47 l0.12 -0.05 H0.155 l0.12 0.05 m0 0.14 H-0.275 ' +
                            'M0.235 -0.07 v0.14 h-0.04 v-0.14 h0.04 M-0.195 -0.07 v0.14 h-0.04 v-0.14 h0.04'),
                'safetyMargin': [0, 0, 0, 0],
            },
            'payloads': ([{
                'template': 'trolley',
                'length': 1.0,
                'width': 0.8,
                'vcenter': 0.3,
                'hcenter': 0,
                'svgPath': ('M-0.4 -0.2 H0.4 V0.8 H-0.4 V-0.2 v0.05 h0.05 v-0.05 v0.025 H0.35 v-0.025 v0.05 h0.05 h-0.025 V0.75 ' +
                            'h0.025 h-0.05 v0.05 v-0.025 H-0.35 v0.025 v-0.05 h-0.05 h0.025 V-0.15'),
                'safetyMargin': [0, 0, 0, 0],
            }, {
                'template': 'trolley',
                'length': 0.8,
                'width': 0.6,
                'vcenter': 0.1,
                'hcenter': 0,
                'svgPath': ('M-0.3 -0.3 H0.3 V0.5 H-0.3 V-0.3 v0.05 h0.05 v-0.05 v0.025 H0.25 v-0.025 v0.05 h0.05 h-0.025 V0.45 ' +
                            'h0.025 h-0.05 v0.05 v-0.025 H-0.25 v0.025 v-0.05 h-0.05 h0.025 V-0.25'),
                'safetyMargin': [0.2, 0.1, 0.12, 0.15],
            }] + [{
                'template': 'trolley',
                'length': 0.8,
                'width': 0.6,
                'vcenter': 0.1,
                'hcenter': 0,
                'svgPath': ('M-0.3 -0.3 H0.3 V0.5 H-0.3 V-0.3 v0.05 h0.05 v-0.05 v0.025 H0.25 v-0.025 v0.05 h0.05 h-0.025 V0.45 ' +
                            'h0.025 h-0.05 v0.05 v-0.025 H-0.25 v0.025 v-0.05 h-0.05 h0.025 V-0.25'),
                'safetyMargin': [0, 0, 0, 0],
            }] * 3),
        },
        allowed_motions=[
            'forward', 'reverse', 'rotate_left', 'rotate_right',
            'uturn_left', 'uturn_right', 'dynamic',
        ]
        if allowed_motions is None else allowed_motions,
        task_triggers={
            'agv_idle': {
                'timeout': 3,
                'abortable': True,
                'ignore_charging': True,
                'ignore_home': True,
                'ignore_stations': [],
                'ignore_low_battery': False,
                'ignore_trigger_active': False,
                'action': {
                    'skillId': None,
                    'params': {},
                    'outcomes': {},
                },
            },
            'battery_low': {
                'threshold': 0,
                'abortable': False,
                'ignore_charging': True,
                'ignore_home': True,
                'ignore_stations': [],
                'ignore_low_battery': False,
                'ignore_trigger_active': False,
                'action': {
                    'skillId': None,
                    'params': {},
                    'outcomes': {},
                },
            },
        },
        custom_init=[-1],
        station_init={
            'allowed': 0,
            'stations': []
        },
        pre_init=[],
        default_init={
            'task_template': 0,  # disabled. Or -1 for parked at home
            'timeout': 1
        },
        default_paused=False,
        default_app={
            'app': '',  # disabled. Or 'task-runner'
            'timeout': 1
        },
        min_battery_level=30,
    )

    Variable.objects.update_or_create(name='executor_cfg', defaults={
        'value': json.dumps(executor_cfg),
    })


def load_map():
    with open(os.path.join(__dir__, 'map.json')) as f:
        data = json.loads(f.read())

    v = agv05_webserver.system.views.BackupRestoreView()
    v.request = type(str(''), (object,), {
        'user': User.objects.get(username='admin'),
    })
    v.load_map(data)


class ExecutorConfig(object):
    disable_laser_at_home = True


def setup_robots():
    # load fixtures and data
    load_map()

    # differential (unrestricted)
    load_agv()
    Cache.set_flag(Cache.Flag.Dirty.value)

    robot = TrackedSim()
    robot.dynamic_path_planning = False
    robot.registry = RegisterManager()
    robot.skill_manager = SkillManager(robot)
    robot.models = Models(robot)
    robot.variable = VariableManager(robot)

    assert robot.models.is_valid(), robot.models.get_validation_msg()
    assert not robot.models.is_fms_mode()
    robot.fms_manager = None
    robot.config = ExecutorConfig()

    robot.base.set_initial_map_and_location(
        robot.models.graph,
        robot.models.get_agv_home_location())

    # trailer (no reverse and no u-turn)
    load_agv(['forward', 'rotate_left', 'rotate_right'])
    Cache.set_flag(Cache.Flag.Dirty.value)

    robot2 = TrackedSim()
    robot2.dynamic_path_planning = False
    robot2.registry = RegisterManager()
    robot2.skill_manager = SkillManager(robot2)
    robot2.models = Models(robot2)
    robot2.variable = VariableManager(robot2)

    assert robot2.models.is_valid(), robot2.models.get_validation_msg()
    assert not robot2.models.is_fms_mode()
    robot2.fms_manager = None
    robot2.config = ExecutorConfig()

    robot2.base.set_initial_map_and_location(
        robot2.models.graph,
        robot2.models.get_agv_home_location())

    # no right-turn
    load_agv(['forward', 'reverse', 'rotate_left', 'uturn_left'])
    Cache.set_flag(Cache.Flag.Dirty.value)

    robot3 = TrackedSim()
    robot3.dynamic_path_planning = False
    robot3.registry = RegisterManager()
    robot3.skill_manager = SkillManager(robot3)
    robot3.models = Models(robot3)
    robot3.variable = VariableManager(robot3)

    assert robot3.models.is_valid(), robot3.models.get_validation_msg()
    assert not robot3.models.is_fms_mode()
    robot3.fms_manager = None
    robot3.config = ExecutorConfig()

    robot3.base.set_initial_map_and_location(
        robot3.models.graph,
        robot3.models.get_agv_home_location())

    return robot, robot2, robot3


def reconstruct_line_graph(models):
    for v in models.validators:
        if isinstance(v, ConstrainedMapValidator):
            v.from_cache()
