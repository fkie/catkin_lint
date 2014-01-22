import unittest

import sys
sys.stderr = sys.stdout

from catkin_lint.linter import CMakeLinter
from .helper import create_env, create_manifest, mock_lint
import catkin_lint.checks.build as cc
from catkin_lint.cmake import SyntaxError as CMakeSyntaxError

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

    @patch("os.path.isfile", lambda x: x == "/mock-path/broken.cmake")
    def test_blacklist(self):
        env = create_env()
        pkg = create_manifest("catkin")
        result = mock_lint(env, pkg,
            {
               "/mock-path/CMakeLists.txt": "project(catkin) include(broken.cmake RESULT_VARIABLE gone) catkin_package()",
               "/mock-path/broken.cmake": "xxxxxx syntax error xxxxx"
            }, checks=cc.all
        )
        self.assertEqual([], result)

    def test_macro(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            macro(fun) endmacro()
            """, checks=cc.all
        )
        self.assertEqual([ "UNSUPPORTED_CMD" ], result)
        self.assertRaises(CMakeSyntaxError, mock_lint, env, pkg, "macro()")


    def test_function(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            function(fun) endfunction()
            """, checks=cc.all
        )
        self.assertEqual([ "UNSUPPORTED_CMD" ], result)
        self.assertRaises(CMakeSyntaxError, mock_lint, env, pkg, "function()")

    @patch("os.path.isdir", lambda x: x in [ "/mock-path/src", "/mock-path/include" ])
    @patch("os.path.isfile", lambda x: x in  [ "/other-path/CMakeLists.txt", "/mock-path/src/CMakeLists.txt", "/mock-path/src/mock.cpp" ])
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
              "/mock-path/CMakeLists.txt" : "project(mock) add_subdirectory(/other-path)",
              "/other-path/CMakeLists.txt" : """
              find_package(catkin REQUIRED)
              catkin_package()
              """
            }, checks=cc.all
        )
        self.assertEqual([ "EXTERNAL_SUBDIR" ], result)

        result = mock_lint(env, pkg,
              """
              project(mock)
              find_package(catkin REQUIRED)
              catkin_package()
              add_subdirectory(missing_subdir)
              """, checks=cc.all
        )
        self.assertEqual([ "MISSING_SUBDIR" ], result)

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
