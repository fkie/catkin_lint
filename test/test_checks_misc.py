import unittest
import catkin_lint.checks.misc as cc
from .helper import create_env, create_manifest, mock_lint

import sys
sys.stderr = sys.stdout

try:
    from mock import patch
except ImportError:
    from unittest.mock import patch

import os.path
import posixpath
import ntpath


class ChecksMiscTest(unittest.TestCase):

    def test_project(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock)", checks=cc.project)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(wrong)", checks=cc.project)
        self.assertEqual([ "PROJECT_NAME" ], result)

    def test_special_vars(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock)", checks=cc.special_vars)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) set(ENV{PATH} wrong)", checks=cc.special_vars)
        self.assertEqual([ "IMMUTABLE_VAR" ], result)
        result = mock_lint(env, pkg, "project(mock) set(CMAKE_BUILD_TYPE wrong)", checks=cc.special_vars)
        self.assertEqual([ "IMMUTABLE_VAR" ], result)
        result = mock_lint(env, pkg, "project(mock) set(CMAKE_CXX_FLAGS wrong)", checks=cc.special_vars)
        self.assertEqual([ "CRITICAL_VAR_OVERWRITE" ], result)
        result = mock_lint(env, pkg, "project(mock) unset(CMAKE_CXX_FLAGS)", checks=cc.special_vars)
        self.assertEqual([ "CRITICAL_VAR_OVERWRITE" ], result)
        result = mock_lint(env, pkg, "project(mock) set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} extra)", checks=cc.special_vars)
        self.assertEqual([ "CRITICAL_VAR_APPEND" ], result)
        result = mock_lint(env, pkg, "project(mock) list(GET CMAKE_MODULE_PATH 0)", checks=cc.special_vars)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) list(LENGTH CMAKE_MODULE_PATH len)", checks=cc.special_vars)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) list(APPEND CMAKE_MODULE_PATH extra)", checks=cc.special_vars)
        self.assertEqual(["CRITICAL_VAR_APPEND"], result)
        result = mock_lint(env, pkg, "project(mock) list(REVERSE CMAKE_MODULE_PATH)", checks=cc.special_vars)
        self.assertEqual(["CRITICAL_VAR_OVERWRITE"], result)
        result = mock_lint(env, pkg, "project(mock) list(INSERT PROJECT_NAME 0 wrong)", checks=cc.special_vars)
        self.assertEqual(["IMMUTABLE_VAR"], result)

    def test_global_vars(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) option(mock_option test OFF) set(mock_global CACHE STRING)", checks=cc.global_vars)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) set(global CACHE STRING)", checks=cc.global_vars)
        self.assertEqual([ "GLOBAL_VAR_COLLISION" ], result)
        result = mock_lint(env, pkg, "project(mock) option(optional test OFF)", checks=cc.global_vars)
        self.assertEqual([ "GLOBAL_VAR_COLLISION" ], result)

    def test_singleton_command(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock)", checks=cc.singleton_commands)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) project(mock2)", checks=cc.singleton_commands)
        self.assertEqual([ "DUPLICATE_CMD" ], result)

    def test_deprecated(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "add_gtest()", checks=cc.deprecated)
        self.assertEqual([ "DEPRECATED_CMD" ], result)
        result = mock_lint(env, pkg, "add_nosetests()", checks=cc.deprecated)
        self.assertEqual([ "DEPRECATED_CMD" ], result)
        result = mock_lint(env, pkg, "download_test_data()", checks=cc.deprecated)
        self.assertEqual([ "DEPRECATED_CMD" ], result)
        result = mock_lint(env, pkg, "parse_arguments()", checks=cc.deprecated)
        self.assertEqual([ "DEPRECATED_CMD" ], result)

    def test_cmake_modules(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) find_package(Eigen)", checks=cc.cmake_modules)
        self.assertEqual([ "MISSING_CMAKE_MODULES" ], result)

    def test_endblock(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "foreach(a) endforeach()", checks=cc.endblock)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "if(a) else() endif()", checks=cc.endblock)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "macro(a) endmacro()", checks=cc.endblock)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "function(a) endfunction()", checks=cc.endblock)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "foreach(a) endforeach(a)", checks=cc.endblock)
        self.assertEqual([ "ENDBLOCK_ARGS" ], result)
        result = mock_lint(env, pkg, "if(a) else(a) endif()", checks=cc.endblock)
        self.assertEqual([ "ENDBLOCK_ARGS" ], result)
        result = mock_lint(env, pkg, "if(a) else() endif(a)", checks=cc.endblock)
        self.assertEqual([ "ENDBLOCK_ARGS" ], result)
        result = mock_lint(env, pkg, "macro(a) endmacro(a)", checks=cc.endblock)
        self.assertEqual([ "ENDBLOCK_ARGS" ], result)
        result = mock_lint(env, pkg, "function(a) endfunction(a)", checks=cc.endblock)
        self.assertEqual([ "ENDBLOCK_ARGS" ], result)

    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/FindLocal.cmake"))
    def do_cmake_includes(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            include(FindLocal.cmake)
            include(FindOptional.cmake OPTIONAL)
            """,
        checks=cc.cmake_includes)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            include(missing.cmake)
            """,
        checks=cc.cmake_includes)
        self.assertEqual([ "MISSING_FILE" ], result)

        result = mock_lint(env, pkg,
            """
            include(FindStuff)
            """,
        checks=cc.cmake_includes)
        self.assertEqual([ "FIND_BY_INCLUDE" ], result)

    @patch("os.path", posixpath)
    def test_posix(self):
        self.do_cmake_includes()

    @patch("os.path", ntpath)
    def test_windows(self):
        self.do_cmake_includes()
