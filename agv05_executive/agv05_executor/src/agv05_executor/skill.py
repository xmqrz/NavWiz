from __future__ import absolute_import
from __future__ import unicode_literals

from django.utils.functional import cached_property
from six import python_2_unicode_compatible
import distutils.util
import importlib
import keyword
import re
import rospkg
import rospy
import smach
import threading
import ujson as json

__all__ = ['Skill', 'SkillManager']


class UserError(smach.InvalidUserCodeError):
    pass


class SkillMeta(type):
    def __new__(cls, name, bases, attrs):
        new_class = super(SkillMeta, cls).__new__(cls, name, bases, attrs)
        if '__str__' not in new_class.__dict__:
            return new_class
        return python_2_unicode_compatible(new_class)


class Skill(smach.State):
    __metaclass__ = SkillMeta
    _param_types = ['bool', 'int', 'double', 'str', 'Station', 'Register',
                    'vbool', 'vint', 'vdouble', 'vstr', 'vStation']

    class Meta(object):
        name = 'Base Skill (Extend Me Please~)'
        params = []
        outcomes = ['Done']
        mutexes = []

    def __init__(self, robot, **kwargs):
        super(Skill, self).__init__(self.Meta.outcomes + ['Preempted'])
        self.preempt_evt = threading.Event()
        self.robot = robot
        self.__dict__.update(**kwargs)

    def __str__(self):
        return self.Meta.name

    @cached_property
    def skill_id(self):
        return self.__module__ + '.' + self.__class__.__name__

    def request_preempt(self):
        super(Skill, self).request_preempt()
        self.preempt_evt.set()

    def fail(self, error_msg):
        raise UserError(error_msg)

    def is_paused(self, trigger=False):
        return smach.is_paused(trigger)


class SkillManager(object):

    def __init__(self, robot):
        self.robot = robot
        self.skills = {}
        self.skill_descriptions = []

        for skill_id, cls in self.load_plugins():
            try:
                params = []
                param_names = set()
                for p in cls.Meta.params:
                    p_name = p['name']
                    p_type = p['type']
                    p_version = p.get('version', 0)

                    if p_name.startswith('_'):
                        raise ValueError('Skill param name ("%s") must not start with underscore.' % p_name)

                    if not re.match(r'^[A-Za-z]\w*$', p_name):
                        raise ValueError('Skill param name ("%s") is not a valid Python identifier name.' % p_name)

                    if keyword.iskeyword(p_name):
                        raise ValueError('Skill param name ("%s") is a reserved keyword in Python' % p_name)

                    if p_name in Skill.__dict__.keys():
                        raise ValueError('Skill param name ("%s") conflicts with existing object attribute.' % p_name)

                    if p_type not in Skill._param_types:
                        raise ValueError('Skill param "%s" has invalid type: %s.' % (p_name, p_type))

                    if p_type in ['int', 'double']:
                        a = p.get('min')
                        b = p.get('max')

                        if a is not None:
                            try:
                                a = int(a) if p_type == 'int' else float(a)
                            except Exception:
                                raise ValueError('Skill param "%s" has invalid `min` value.' % p_name)
                            else:
                                p['min'] = a

                        if b is not None:
                            try:
                                b = int(b) if p_type == 'int' else float(b)
                            except Exception:
                                raise ValueError('Skill param "%s" has invalid `max` value.' % p_name)
                            else:
                                p['max'] = b

                        if a is not None and b is not None:
                            if a > b:
                                raise ValueError('Skill param "%s": `min` is larger than `max`.' % p_name)

                    try:
                        int(p_version)
                    except Exception:
                        raise ValueError('Skill param "%s": `version` must be an integer.' % p_name)

                    params.append(json.dumps(p, sort_keys=True))
                    param_names.add(p_name)

                if len(params) != len(param_names):
                    raise ValueError('Skill has duplicate param names.')

                assert isinstance(cls.Meta.outcomes, list), 'Skill outcomes must be a list.'
                assert isinstance(cls.Meta.mutexes, list), 'Skill mutexes must be a list.'

                desc = {
                    'id': skill_id,
                    'name': cls.Meta.name,
                    'params': params,
                    'outcomes': cls.Meta.outcomes,
                    'mutexes': cls.Meta.mutexes,
                }
                self.skill_descriptions.append(desc)
            except Exception as ex:
                rospy.logerr('Skill check failed [%s]: %s', skill_id, ex)
            else:
                self.skills[skill_id] = cls

    def get_skill_class(self, skill_id):
        if skill_id in self.skills:
            return self.skills[skill_id]
        else:
            return None

    def get_skill_descriptions(self):
        return self.skill_descriptions

    def get_summary(self):
        return [d['name'] for d in self.skill_descriptions]

    def load_plugins(self):
        for plugin in self._iter_plugins():
            cls = self._load_plugin0(plugin)
            if cls:
                yield (plugin, cls)

    def _load_plugin0(self, plugin):
        try:
            module_name, cls_name = plugin.rsplit('.', 1)
            module = importlib.import_module(module_name)
            cls = getattr(module, cls_name)

            if not issubclass(cls, Skill):
                rospy.logerr('Cannot load plugin [%s]: Plugin must inherit `agv05_executor.skill.Skill` class.', plugin)
                return None

            if not hasattr(cls.Meta, 'outcomes') or not hasattr(cls.Meta.outcomes, '__len__'):
                rospy.logerr('Cannot load plugin [%s]: Outcomes must be a list or sequence-typed object.', plugin)
                return None

            if not len(cls.Meta.outcomes):
                rospy.logerr('Cannot load plugin [%s]: Plugin must define at least one outcome in its Meta class.', plugin)
                return None

            return cls

        except Exception as ex:
            rospy.logerr('Cannot load plugin [%s]: %s', plugin, ex)

    def _iter_plugins(self):
        rospack = rospkg.RosPack()
        pkg_list = sorted(rospack.get_depends_on('agv05_executor', implicit=False),
            key=lambda x: x if x != 'agv05_skills' else '')

        for pkg in pkg_list:
            m = rospack.get_manifest(pkg)
            for e in m.exports:
                if e.tag != 'agv05_executor':
                    continue

                plugin = e.get('skill_plugin')
                if plugin is None:
                    continue

                tracked_only = False
                try:
                    tracked_only = distutils.util.strtobool(e.get('tracked_only'))
                except Exception:
                    pass

                trackless_only = False
                try:
                    trackless_only = distutils.util.strtobool(e.get('trackless_only'))
                except Exception:
                    pass

                if tracked_only and self.robot.trackless:
                    continue
                if trackless_only and not self.robot.trackless:
                    continue
                if plugin.startswith('agv05_skills.dynplan') and not self.robot.dynamic_path_planning:
                    continue
                yield plugin
