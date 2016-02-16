import unittest
import catkin_lint.checks.cuda as cc
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


class ChecksCudaTest(unittest.TestCase):

    @patch("os.path.isfile", lambda x: x in [os.path.normpath("/mock-path/src/a.cpp"), os.path.normpath("/mock-path/src/b.cpp")])
    def do_targets(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "cuda_add_executable(target src/a.cpp src/b.cpp) cuda_add_library(target_lib src/a.cpp src/b.cpp)", checks=cc.targets)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "cuda_add_executable(target ${CMAKE_CURRENT_SOURCE_DIR}/src/a.cpp) cuda_add_library(target_lib ${CMAKE_CURRENT_SOURCE_DIR}/src/a.cpp)", checks=cc.targets)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "cuda_add_executable(target} src/missing.cpp)", checks=cc.targets)
        self.assertEqual([ "MISSING_FILE" ], result)
        result = mock_lint(env, pkg, "cuda_add_library(target src/missing.cpp)", checks=cc.targets)
        self.assertEqual([ "MISSING_FILE" ], result)
        result = mock_lint(env, pkg, "cuda_add_executable(target src/b.cpp src/a.cpp)", checks=cc.targets)
        self.assertEqual([ "UNSORTED_LIST" ], result)
        result = mock_lint(env, pkg, "cuda_add_library(target} src/b.cpp src/a.cpp)", checks=cc.targets)
        self.assertEqual([ "UNSORTED_LIST" ], result)

    @patch("os.path", posixpath)
    def test_posix(self):
        self.do_targets()

    @patch("os.path", ntpath)
    def test_windows(self):
        self.do_targets()
