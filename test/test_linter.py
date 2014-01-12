import unittest

import sys
sys.stderr = sys.stdout

from catkin_lint.linter import CMakeLinter
from .helper import create_env, create_manifest, mock_lint

class MainTest(unittest.TestCase):
    def test_circular_depend(self):
        def a(linter):
            linter.require(b)
        def b(linter):
            linter.require(a)
        linter = CMakeLinter(create_env())
        self.assertRaises(RuntimeError, linter.require, a)
