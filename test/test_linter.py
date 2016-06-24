import unittest

import sys
sys.stderr = sys.stdout

from catkin_lint.linter import CMakeLinter, LintInfo
from .helper import create_env, create_manifest, mock_lint
import catkin_lint.checks.build as cc
import catkin_lint.environment
from catkin_lint.cmake import CMakeSyntaxError
from catkin_pkg.package import Export

import os.path
import posixpath
import ntpath

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

    def test_lower_case(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            PROJECT(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            """, checks=cc.all)
        self.assertEqual([ "CMD_CASE"], result)

    def test_pragma(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            #catkin_lint: ignore cmd_case
            PROJECT(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            """, checks=cc.all)
        self.assertEqual([], result)

    def test_if(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            if ("${var}" STREQUAL "foo")
            endif()
            if (EXISTS "filename")
            endif()
            """, checks=cc.all)
        self.assertEqual([], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            if (${var} STREQUAL "foo")
            endif()
            """, checks=cc.all)
        self.assertEqual(["UNQUOTED_STRING_OP"], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            if (EXISTS filename)
            endif()
            """, checks=cc.all)
        self.assertEqual(["UNQUOTED_STRING_OP"], result)
        self.assertRaises(CMakeSyntaxError, mock_lint, env, pkg, "if(STREQUAL) endif()")
        self.assertRaises(CMakeSyntaxError, mock_lint, env, pkg, "if(A STREQUAL) endif()")
        self.assertRaises(CMakeSyntaxError, mock_lint, env, pkg, "if(STREQUAL A) endif()")
        self.assertRaises(CMakeSyntaxError, mock_lint, env, pkg, "if(EXISTS) endif()")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            if (varname)
            endif()
            """, checks=cc.all)
        self.assertEqual([], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            if (${varname})
            endif()
            """, checks=cc.all)
        self.assertEqual([ "AMBIGUOUS_CONDITION" ], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            if ("${varname}")
            endif()
            """, checks=cc.all)
        self.assertEqual([ "AMBIGUOUS_CONDITION" ], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            if ("${varname}${othervarname}")
            endif()
            """, checks=cc.all)
        self.assertEqual([], result)

    def test_list(self):
        env = create_env()
        linter = CMakeLinter(env)
        info = LintInfo(env)
        linter._handle_list(info, ["APPEND", "test", "one" ])
        self.assertEqual(info.var["test"], "one")
        linter._handle_list(info, ["APPEND", "test", "three" ])
        self.assertEqual(info.var["test"], "one;three")
        linter._handle_list(info, ["INSERT", "test", "1", "two" ])
        self.assertEqual(info.var["test"], "one;two;three")
        linter._handle_list(info, ["GET", "test", "1", "result" ])
        self.assertEqual(info.var["result"], "two")
        linter._handle_list(info, ["GET", "test", "42", "result" ])
        self.assertEqual(info.var["result"], "")
        linter._handle_list(info, ["FIND", "test", "none", "result" ])
        self.assertEqual(info.var["result"], "-1")
        linter._handle_list(info, ["FIND", "test", "two", "result" ])
        self.assertEqual(info.var["result"], "1")
        info.var["test"] = "one;two;three;one;four;five"
        linter._handle_list(info, ["REMOVE_DUPLICATES", "test"])
        self.assertEqual(info.var["test"], "one;two;three;four;five")
        linter._handle_list(info, ["REMOVE_AT", "test", "15", "0", "4", "2"])
        self.assertEqual(info.var["test"], "two;four")
        linter._handle_list(info, ["APPEND", "test", "two" ])
        linter._handle_list(info, ["REMOVE_ITEM", "test", "two" ])
        self.assertEqual(info.var["test"], "four")
        info.var["test"] = "1;3;2;6;5;4"
        linter._handle_list(info, ["SORT", "test"])
        self.assertEqual(info.var["test"], "1;2;3;4;5;6")
        linter._handle_list(info, ["REVERSE", "test"])
        self.assertEqual(info.var["test"], "6;5;4;3;2;1")

        info.var["test"] = "one;two;three"
        linter._handle_list(info, ["UNKNOWN", "test"])
        self.assertEqual(info.var["test"], "one;two;three")
        linter._handle_list(info, [])
        self.assertEqual(info.var["test"], "one;two;three")

    def test_env_var(self):
        env = create_env()
        pkg = create_manifest("catkin")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            set(bla $ENV{PATH})
            """, checks=cc.all)
        self.assertEqual([ "ENV_VAR"], result)

    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/broken.cmake"))
    def do_blacklist(self):
        env = create_env()
        pkg = create_manifest("catkin")
        result = mock_lint(env, pkg,
            {
               "/mock-path/CMakeLists.txt": "project(catkin) include(broken.cmake RESULT_VARIABLE gone) catkin_package()",
               "/mock-path/broken.cmake": "xxxxxx syntax error xxxxx"
            }, checks=cc.all
        )
        self.assertEqual([], result)


    @patch("os.path.isdir", lambda x: x == "/" or x == "\\")
    @patch("os.path.realpath", lambda x: x)
    def do_environment(self):
        env = catkin_lint.environment.CatkinEnvironment(use_rosdep=False)
        mock_packages = {}
        mock_packages[os.path.normpath("/mock_catkin")] = create_manifest("mock_catkin")
        mock_packages[os.path.normpath("/mock_other")] = create_manifest("mock_other")
        mock_packages[os.path.normpath("/mock_other")].exports += [ Export("random_tag"), Export("build_type", "cmake") ]
        old_find = catkin_lint.environment.find_packages
        catkin_lint.environment.find_packages = lambda x, use_cache: mock_packages
        result = env.add_path(os.path.normpath("/"))
        self.assertEqual(1, len(result))
        self.assertTrue(env.is_catkin_pkg("mock_catkin"))
        self.assertFalse(env.is_catkin_pkg("mock_other"))
        self.assertFalse(env.is_system_pkg("mock_catkin"))
        self.assertTrue(env.is_system_pkg("mock_other"))
        result = env.add_path(os.path.normpath("/"))
        self.assertEqual(1, len(result))
        self.assertTrue(env.is_catkin_pkg("mock_catkin"))
        self.assertFalse(env.is_catkin_pkg("mock_other"))
        self.assertFalse(env.is_system_pkg("mock_catkin"))
        self.assertTrue(env.is_system_pkg("mock_other"))
        result = env.add_path(os.path.normpath("/missing"))
        self.assertEqual([], result)
        self.assertFalse(env.is_catkin_pkg("invalid"))
        self.assertFalse(env.is_system_pkg("invalid"))
        catkin_lint.environment.find_packages = old_find


    @patch("os.path.isdir", lambda x: x in [ os.path.normpath("/mock-path/src"), os.path.normpath("/mock-path/include") ])
    @patch("os.path.isfile", lambda x: x in  [ os.path.normpath("/other-path/CMakeLists.txt"), os.path.normpath("/mock-path/src/CMakeLists.txt"), os.path.normpath("/mock-path/src/source.cpp") ])
    def do_subdir(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            {
              "/mock-path/CMakeLists.txt" : "project(mock) add_subdirectory(src) add_executable(${PROJECT_NAME}_test2 src/source.cpp)",
              "/mock-path/src/CMakeLists.txt" : """
              include_directories(../include)
              find_package(catkin REQUIRED)
              catkin_package()
              add_executable(${PROJECT_NAME}_test source.cpp)
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
              add_executable(${PROJECT_NAME}_test source.cpp)
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

        var = mock_lint(env, pkg,
            {
              "/mock-path/CMakeLists.txt" : """
              project(mock)
              set(foo "toplevel")
              add_subdirectory(src)
              """,
              "/mock-path/src/CMakeLists.txt" : """
              set(foo "subdir")
              find_file(bar bar.txt)
              """
            }, checks=None, return_var=True
        )
        self.assertEqual("toplevel", var["foo"])
        self.assertFalse("bar" in var)
        var = mock_lint(env, pkg,
            {
              "/mock-path/CMakeLists.txt" : """
              project(mock)
              set(foo "toplevel")
              add_subdirectory(src)
              """,
              "/mock-path/src/CMakeLists.txt" : """
              set(foo "subdir" PARENT_SCOPE)
              """
            }, checks=None, return_var=True
        )
        self.assertEqual("subdir", var["foo"])


    @patch("os.path", posixpath)
    def test_posix(self):
        self.do_blacklist()
        self.do_environment()
        self.do_subdir()

    @patch("os.path", ntpath)
    def test_windows(self):
        self.do_blacklist()
        self.do_environment()
        self.do_subdir()
