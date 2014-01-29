import unittest
import catkin_lint.checks.python as cc
from .helper import create_env, create_manifest, mock_lint

import sys
sys.stderr = sys.stdout

import os.path
import posixpath
import ntpath

try:
    from mock import patch
except ImportError:
    from unittest.mock import patch

class ChecksPythonTest(unittest.TestCase):

    @patch("os.path.isfile", lambda x: False)
    def test_setup_without_setup_py(self):
        env = create_env()
        pkg = create_manifest("mock")

        result = mock_lint(env, pkg, "", checks=cc.setup)
        self.assertEqual([], result)

        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_python_setup()", checks=cc.setup)
        self.assertEqual([ "MISSING_FILE" ], result)

    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/setup.py"))
    def do_setup_with_setup_py(self):
        env = create_env()
        pkg = create_manifest("mock")

        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_python_setup()", checks=cc.setup)
        self.assertEqual([], result)

        result = mock_lint(env, pkg, "project(mock) catkin_python_setup()", checks=cc.setup)
        self.assertEqual([ "CATKIN_ORDER_VIOLATION" ], result)

        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) generate_messages() catkin_python_setup()", checks=cc.setup)
        self.assertEqual([ "ORDER_VIOLATION" ], result)

        result = mock_lint(env, pkg, "project(mock)", checks=cc.setup)
        self.assertEqual([ "MISSING_PYTHON_SETUP" ], result)

        pkg = create_manifest("catkin")
        result = mock_lint(env, pkg, "project(catkin) catkin_python_setup()", checks=cc.setup)
        self.assertEqual([], result)

    @patch("os.path", posixpath)
    def test_posix(self):
        self.do_setup_with_setup_py()

    @patch("os.path", ntpath)
    def test_windows(self):
        self.do_setup_with_setup_py()
