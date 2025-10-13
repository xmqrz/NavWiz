from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executive_msgs.srv import GetVariable, SetVariable
from contextlib import contextmanager
from django.utils.encoding import force_text
import rospy
import threading
import ujson as json


class Variable(object):
    vtype = None
    base_type = None
    class_type = None

    def __init__(self, id, value):
        self.id = id
        self._value = self.sanitize_type(value)
        self._default = value
        self._lock = threading.Lock()
        self._derived = None

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        value = self.sanitize_type(value)
        with self._update_ctx():
            self._value = value

    def sanitize_type(self, value):
        return self.class_type(value)

    def compare_and_set(self, expect_value, new_value):
        expect_value = self.sanitize_type(expect_value)
        new_value = self.sanitize_type(new_value)
        if self._value != expect_value:
            return False
        with self._update_ctx() as ctx:
            if self._value != expect_value:
                ctx.update = False
                return False
            self._value = new_value
            return True

    def is_vtype(self, t):
        return t == self.vtype

    def is_type(self, t):
        return t == self.base_type

    @contextmanager
    def _update_ctx(self):
        with self._lock:
            ctx = type(str(''), (), dict(update=True))
            yield ctx

    def update_derived_value(self):
        assert self._derived
        self.value = self._derived.value

    def derive(self):
        assert not self._derived
        v = type(self)(self.id, self.value)
        v._derived = self
        return v

    @property
    def is_derived(self):
        return self._derived is not None


class NumVariableMixin(Variable):

    def increment(self, inc):
        inc = self.sanitize_type(inc)
        with self._update_ctx():
            self._value += inc

    def decrement(self, dec):
        dec = self.sanitize_type(dec)
        with self._update_ctx():
            self._value -= dec

    def multiply(self, multiplier):
        multiplier = self.sanitize_type(multiplier)
        with self._update_ctx():
            self._value *= multiplier


class BoolVariable(Variable):
    vtype = 'vbool'
    base_type = 'bool'
    class_type = bool


class IntVariable(NumVariableMixin, Variable):
    vtype = 'vint'
    base_type = 'int'
    class_type = int

    def divide(self, divisor):
        divisor = self.sanitize_type(divisor)
        with self._update_ctx():
            self._value, remainder = divmod(self._value, divisor)
            return remainder


class DoubleVariable(NumVariableMixin, Variable):
    vtype = 'vdouble'
    base_type = 'double'
    class_type = float

    def divide(self, divisor):
        divisor = self.sanitize_type(divisor)
        with self._update_ctx():
            self._value /= divisor


class StrVariable(Variable):
    vtype = 'vstr'
    base_type = 'str'
    class_type = staticmethod(force_text)


class StationVariable(Variable):
    vtype = 'vStation'
    base_type = 'Station'
    class_type = staticmethod(force_text)

    def __init__(self, id, value, models):
        self.models = models
        super(StationVariable, self).__init__(id, value)

    def sanitize_type(self, value):
        value = super(StationVariable, self).sanitize_type(value)
        if value not in self.models.station_names:
            raise RuntimeError('Trying to set station with invalid station value "%s"' % value)
        return value

    def derive(self):
        assert not self._derived
        v = type(self)(self.id, self.value, self.models)
        v._derived = self
        return v


class VariableManager(object):
    variables_class = [
        BoolVariable,
        IntVariable,
        DoubleVariable,
        StrVariable,
        StationVariable,
    ]

    def __init__(self, robot):
        self.robot = robot
        self.variables_class_assoc = \
            {c.vtype: c for c in self.variables_class}

        self.__store = {}
        self.variable_list = []

        self.init()

        self.__get_variable_service = rospy.Service('~get_variable', GetVariable, self.handle_get_variable)
        self.__set_variable_service = rospy.Service('~set_variable', SetVariable, self.handle_set_variable)

    def init(self):
        if not self.robot.models.valid or not getattr(self.robot.models, 'variables', None):
            return

        for v in self.robot.models.variables:
            if v['vtype'] == 'vStation':
                self.__store[v['name']] = StationVariable(v['name'], v['default'], self.robot.models)
            else:
                self.__store[v['name']] = self.variables_class_assoc[v['vtype']](v['name'], v['default'])

        self.variable_list = sorted(self.__store.keys())

    def get(self, id):
        return self.__store.get(id)

    def is_variable(self, value):
        return isinstance(value, Variable)

    def get_variables(self):
        for k, v in self.__store.items():
            yield (k, v)

    def handle_get_variable(self, req):
        try:
            if req.name:
                v = self.get(req.name)
                return v.base_type, json.dumps(v.value)
            return '', json.dumps(self.variable_list)
        except Exception:
            pass

    def handle_set_variable(self, req):
        try:
            v = self.get(req.name)
            if not v:
                return
            v.value = json.loads(req.value)
            return True, v.base_type, json.dumps(v.value)
        except Exception:
            return False, '', ''
