from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executive_msgs.srv import GetRegister, SetRegister
from agv05_webserver.system.models import Variable, db_auto_reconnect
from contextlib import contextmanager
import rospy
import threading

__all__ = ['Register', 'RegisterManager']


class Register(object):

    def __init__(self, id):
        self.id = id
        self._value = 0
        self._lock = threading.Lock()

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        with self._update_ctx():
            self._value = value

    def increment(self, inc):
        with self._update_ctx():
            self._value += inc

    def decrement(self, dec):
        with self._update_ctx():
            self._value -= dec

    def multiply(self, multiplier):
        with self._update_ctx():
            self._value *= multiplier

    def divide(self, divisor):
        with self._update_ctx():
            self._value, remainder = divmod(self._value, divisor)
            return remainder

    def compare_and_set(self, expect_value, new_value):
        if self._value != expect_value:
            return False
        with self._update_ctx() as ctx:
            if self._value != expect_value:
                ctx.update = False
                return False
            self._value = new_value
            return True

    @contextmanager
    def _update_ctx(self):
        with self._lock:
            ctx = type(str(''), (), dict(update=True))
            yield ctx
            if ctx.update:
                self._update_db()

    def _update_db(self):
        pass


class PersistentRegister(Register):
    from_db = None
    key_prefix = '/agv05_executor/register_'

    def __init__(self, id):
        super(PersistentRegister, self).__init__(id)
        self.key = self.key_prefix + str(id)

        if PersistentRegister.from_db is None:
            with db_auto_reconnect():
                PersistentRegister.from_db = dict(Variable.objects.filter(name__startswith=self.key_prefix).values_list('name', 'value'))
        try:
            self._value = int(PersistentRegister.from_db[self.key])
        except Exception:
            pass

    def _update_db(self):
        try:
            with db_auto_reconnect():
                Variable.objects.update_or_create(name=self.key, defaults={
                    'value': str(self._value),
                })
        except Exception:
            pass


class RegisterManager(object):

    def __init__(self):
        self.__store = {}
        self.init_persistent()
        self.init_global()
        self.init_local()

        self.register_list = sorted(self.__store.keys())

        self.__get_register_service = rospy.Service('~get_register', GetRegister, self.handle_get_register)
        self.__set_register_service = rospy.Service('~set_register', SetRegister, self.handle_set_register)

    def init_persistent(self):
        self.__store.update({
            id: PersistentRegister(id) for id in ['PS%d' % d for d in range(10)]
        })

    def init_global(self):
        self.__store.update({
            id: Register(id) for id in ['GB%d' % d for d in range(10)]
        })

    def init_local(self):
        a = ord('A')
        self.__store.update({
            id: Register(id) for id in ['%c%d' % (chr(c), d) for c in range(a, a + 3) for d in range(10)]
        })

    def get(self, id):
        return self.__store.get(id)

    def get_register_list(self):
        return self.register_list

    def get_summary(self):
        return ['A0-A9', 'B0-B9', 'C0-C9', 'GB0-GB9', 'PS0-PS9']

    def handle_get_register(self, req):
        try:
            return self.get(req.name).value
        except Exception:
            pass

    def handle_set_register(self, req):
        try:
            self.get(req.name).value = req.value
            return ()
        except Exception:
            pass
