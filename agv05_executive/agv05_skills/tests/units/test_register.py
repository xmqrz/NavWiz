from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from django.test.utils import setup_databases
import agv05_skills.register
import django
import os
import unittest

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "agv05_webserver.settings-test")
django.setup()
setup_databases(verbosity=2, interactive=False)

if True:  # prevent autopep8 messing up the import sequence
    from agv05_executor.register import RegisterManager


def setUpModule():
    global _rm
    _rm = RegisterManager()


def tearDownModule():
    global _rm
    del _rm


class TestSetRegister(unittest.TestCase):

    def test_set_register(self):
        self._test_set_register('A0')
        self._test_set_register('PS0')

    def _test_set_register(self, name):
        a0 = _rm.get(name)

        skill = agv05_skills.register.SetRegister(None,
            register=a0, value=17167)

        skill2 = agv05_skills.register.SetRegister(None,
            register=a0, value=-1)

        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 17167)

        self.assertEqual(skill2.execute({}), 'Done')
        self.assertEqual(a0.value, -1)

        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 17167)


class TestCompareAndSetRegister(unittest.TestCase):

    def test_compare_and_set_register(self):
        self._test_compare_and_set_register('A0')
        self._test_compare_and_set_register('PS0')

    def _test_compare_and_set_register(self, name):
        a0 = _rm.get(name)
        a0.value = 100

        skill = agv05_skills.register.CompareAndSetRegister(None,
            register=a0, expected_value=0, new_value=17167)

        skill2 = agv05_skills.register.CompareAndSetRegister(None,
            register=a0, expected_value=100, new_value=17167)

        self.assertEqual(skill.execute({}), 'Not Equal')
        self.assertEqual(a0.value, 100)

        self.assertEqual(skill2.execute({}), 'Equal')
        self.assertEqual(a0.value, 17167)


class TestIncrRegister(unittest.TestCase):

    def test_incr_register(self):
        self._test_incr_register('A0')
        self._test_incr_register('PS0')

    def _test_incr_register(self, name):
        a0 = _rm.get(name)
        a0.value = 17167

        skill = agv05_skills.register.IncrRegister(None,
            register=a0, value=200)

        skill2 = agv05_skills.register.IncrRegister(None,
            register=a0, value=-1000)

        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 17367)

        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 17567)

        self.assertEqual(skill2.execute({}), 'Done')
        self.assertEqual(a0.value, 16567)


class TestDecrRegister(unittest.TestCase):

    def test_decr_register(self):
        self._test_decr_register('A0')
        self._test_decr_register('PS0')

    def _test_decr_register(self, name):
        a0 = _rm.get(name)
        a0.value = 17167

        skill = agv05_skills.register.DecrRegister(None,
            register=a0, value=200)

        skill2 = agv05_skills.register.DecrRegister(None,
            register=a0, value=-1000)

        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 16967)

        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 16767)

        self.assertEqual(skill2.execute({}), 'Done')
        self.assertEqual(a0.value, 17767)


class TestAddRegister(unittest.TestCase):

    def test_add_register(self):
        self._test_add_register('A0', 'A1')
        self._test_add_register('A0', 'PS1')
        self._test_add_register('PS0', 'A1')
        self._test_add_register('PS0', 'PS1')

    def _test_add_register(self, name1, name2):
        a0 = _rm.get(name1)
        a1 = _rm.get(name2)
        a0.value = 1234
        a1.value = 1000

        skill = agv05_skills.register.AddRegister(None,
            destination=a0, source=a1)

        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 2234)
        self.assertEqual(a1.value, 1000)

        a1.value = -100
        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 2134)
        self.assertEqual(a1.value, -100)


class TestSubtractRegister(unittest.TestCase):

    def test_subtract_register(self):
        self._test_subtract_register('A0', 'A1')
        self._test_subtract_register('A0', 'PS1')
        self._test_subtract_register('PS0', 'A1')
        self._test_subtract_register('PS0', 'PS1')

    def _test_subtract_register(self, name1, name2):
        a0 = _rm.get(name1)
        a1 = _rm.get(name2)
        a0.value = 8069
        a1.value = 10

        skill = agv05_skills.register.SubtractRegister(None,
            destination=a0, source=a1)

        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 8059)
        self.assertEqual(a1.value, 10)

        a1.value = -30
        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 8089)
        self.assertEqual(a1.value, -30)


class TestMultiplyRegister(unittest.TestCase):

    def test_multiply_register(self):
        self._test_multiply_register('A0', 'A1')
        self._test_multiply_register('A0', 'PS1')
        self._test_multiply_register('PS0', 'A1')
        self._test_multiply_register('PS0', 'PS1')

    def _test_multiply_register(self, name1, name2):
        a0 = _rm.get(name1)
        a1 = _rm.get(name2)
        a0.value = 12345679
        a1.value = 9

        skill = agv05_skills.register.MultiplyRegister(None,
            destination=a0, source=a1)

        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 111111111)
        self.assertEqual(a1.value, 9)

        a1.value = -9
        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, -999999999)
        self.assertEqual(a1.value, -9)


class TestDivideRegister(unittest.TestCase):

    def test_divide_register(self):
        self._test_divide_register('A0', 'A1', 'A2')
        self._test_divide_register('A0', 'PS1', 'PS2')
        self._test_divide_register('PS0', 'A1', 'A2')
        self._test_divide_register('PS0', 'PS1', 'PS2')

    def _test_divide_register(self, name1, name2, name3):
        a0 = _rm.get(name1)
        a1 = _rm.get(name2)
        a2 = _rm.get(name3)
        a0.value = 17167
        a1.value = 5

        skill = agv05_skills.register.DivideRegister(None,
            destination=a0, source=a1, remainder=a2)

        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 3433)
        self.assertEqual(a1.value, 5)
        self.assertEqual(a2.value, 2)

        a0.value = -3433
        a1.value = 2
        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, -1717)
        self.assertEqual(a1.value, 2)
        self.assertEqual(a2.value, 1)


class TestCopyRegister(unittest.TestCase):

    def test_copy_register(self):
        self._test_copy_register('A0', 'A1', 'A2')
        self._test_copy_register('A0', 'PS1', 'PS2')
        self._test_copy_register('PS0', 'A1', 'A2')
        self._test_copy_register('PS0', 'PS1', 'PS2')

    def _test_copy_register(self, name1, name2, name3):
        a0 = _rm.get(name1)
        a1 = _rm.get(name2)
        a2 = _rm.get(name3)
        a0.value = 0
        a1.value = 17167
        a2.value = -8089

        skill = agv05_skills.register.CopyRegister(None,
            destination=a0, source=a1)

        skill2 = agv05_skills.register.CopyRegister(None,
            destination=a0, source=a2)

        self.assertEqual(skill.execute({}), 'Done')
        self.assertEqual(a0.value, 17167)

        self.assertEqual(skill2.execute({}), 'Done')
        self.assertEqual(a0.value, -8089)


class TestDisplayRegister(unittest.TestCase):

    def test_display_register(self):
        self._test_display_register('A0')
        self._test_display_register('PS0')

    def _test_display_register(self, name):
        a0 = _rm.get(name)
        a0.value = 17167

        skill = agv05_skills.register.DisplayRegister(None,
            register=a0, message_format='Value: %d', timeout=1)

        self.assertEqual(skill.execute({}), 'Timeout')
