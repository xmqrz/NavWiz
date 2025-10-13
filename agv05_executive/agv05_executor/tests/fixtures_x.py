from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.models import Models
from agv05_executor.models.validators.map_x import ConstrainedMapXValidator
from agv05_executor.register import RegisterManager
from agv05_executor.robots.trackless_sim import TracklessSim
from agv05_executor.skill import SkillManager
from agv05_executor.variable import VariableManager
from agv05_webserver.systemx.models import Cache, MapOcg
import agv05_webserver.system.views  # ensure correct import order to prevent cyclic import
import agv05_webserver.systemx.views
import os
import ujson as json

# relative imports
from fixtures import User, load_agv


__all__ = ['setup_robots', 'reconstruct_line_graph', 'remove_ocgs']

__dir__ = os.path.dirname(os.path.abspath(__file__))


def load_map():
    with open(os.path.join(__dir__, 'map_x.json')) as f:
        data = json.loads(f.read())

    v = agv05_webserver.systemx.views.BackupRestoreView()
    v.request = type(str(''), (object,), {
        'user': User.objects.get(username='admin'),
    })
    v.load_map(data)


class ExecutorConfig(object):
    always_rotate_align_at_destination = True
    forward_speed_limit_by_next_path_length = False
    search_line_trial_max = 2
    straight_miss_junction_pre_check = False
    disable_laser_at_home = True


def setup_robots():
    # load fixtures and data
    load_map()

    # differential (unrestricted)
    load_agv()
    Cache.set_flag(Cache.Flag.Dirty.value)

    robot = TracklessSim()
    robot.dynamic_path_planning = True
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

    robot2 = TracklessSim()
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

    # no reverse
    load_agv(['forward', 'rotate_left', 'rotate_right', 'uturn_left', 'uturn_right'])
    Cache.set_flag(Cache.Flag.Dirty.value)

    robot3 = TracklessSim()
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
        if isinstance(v, ConstrainedMapXValidator):
            v.from_cache()


def remove_ocgs():
    for ocg in MapOcg.objects.all():
        ocg.png_file.delete()
