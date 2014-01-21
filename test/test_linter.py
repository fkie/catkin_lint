import unittest

import sys
sys.stderr = sys.stdout

from catkin_lint.linter import CMakeLinter
from .helper import create_env, create_manifest, mock_lint
import catkin_lint.checks.build as cc

try:
    from mock import patch
except ImportError:
    from unittest.mock import patch

class LinterTest(unittest.TestCase):
    def test_circular_depend(self):
        def a(linter):
            linter.require(b)
        def b(linter):
            linter.require(a)
        linter = CMakeLinter(create_env())
        self.assertRaises(RuntimeError, linter.require, a)

    @patch("os.path.isdir", lambda x: x in [ "/mock-path/src", "/mock-path/include" ])
    @patch("os.path.isfile", lambda x: x in  [ "/mock-path/src/CMakeLists.txt", "/mock-path/src/mock.cpp" ])
    def test_subdir(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            {
              "/mock-path/CMakeLists.txt" : "project(mock) add_subdirectory(src)",
              "/mock-path/src/CMakeLists.txt" : """
              include_directories(../include)
              find_package(catkin REQUIRED)
              catkin_package()
              add_executable(mock_test mock.cpp)
              """
            }, checks=cc.all
        )
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            {
              "/mock-path/CMakeLists.txt" : "project(mock) add_subdirectory(src)",
              "/mock-path/src/CMakeLists.txt" : """
              include_directories(../include)
              find_package(catkin REQUIRED)
              catkin_package()
              add_executable(mock_test mock.cpp)
              add_subdirectory(../src)
              """
            }, checks=cc.all
        )
        self.assertEqual([ "DUPLICATE_SUBDIR" ], result)

        result = mock_lint(env, pkg,
            {
              "/mock-path/CMakeLists.txt" : """
              project(mock)
              find_package(catkin REQUIRED)
              catkin_package()
              add_subdirectory(src)
              """,
              "/mock-path/src/CMakeLists.txt" : """
              project(submock)
              """
            }, checks=cc.all
        )
        self.assertEqual([ "SUBPROJECT" ], result)
