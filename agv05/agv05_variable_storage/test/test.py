#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_variable_storage import VariableStorage
import rospy
import unittest


class TestVariableStorage(unittest.TestCase):

    def test_main(self):
        storage = VariableStorage()
        name = "py_test_key_1"
        name2 = "py_test_key_2"
        value = "py test value 1024"
        value2 = "py test value 2048"

        storage.set_variable(name, value)
        storage.set_variable(name2, value2)
        self.assertTrue(storage.has_variable(name))
        self.assertTrue(storage.has_variable(name2))
        self.assertEqual(storage.get_variable(name), value)
        self.assertEqual(storage.get_variable(name, 'default'), value)
        self.assertEqual(storage.get_variable(name2), value2)
        self.assertEqual(storage.get_variable(name2, 'default'), value2)

        storage.set_variable(name, value2)
        storage.set_variable(name2, value)
        self.assertTrue(storage.has_variable(name))
        self.assertTrue(storage.has_variable(name2))
        self.assertEqual(storage.get_variable(name), value2)
        self.assertEqual(storage.get_variable(name, 'default'), value2)
        self.assertEqual(storage.get_variable(name2), value)
        self.assertEqual(storage.get_variable(name2, 'default'), value)

        storage.delete_variable(name)
        storage.delete_variable(name2)
        self.assertFalse(storage.has_variable(name))
        self.assertFalse(storage.has_variable(name2))
        self.assertEqual(storage.get_variable(name, 'default'), 'default')
        self.assertEqual(storage.get_variable(name2, 'default'), 'default')
        with self.assertRaises(KeyError):
            storage.get_variable(name)
        with self.assertRaises(KeyError):
            storage.get_variable(name2)


if __name__ == '__main__':
    rospy.init_node('test_py')
    import rostest
    rostest.rosrun('agv05_variable_storage', 'test_py', TestVariableStorage)
