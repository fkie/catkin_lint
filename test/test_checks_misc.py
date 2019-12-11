import unittest
import os
import catkin_lint.checks.misc as cc
from .helper import create_env, create_manifest, mock_lint, patch, posix_and_nt


class ChecksMiscTest(unittest.TestCase):

    def test_project(self):
        """Test project()"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock)", checks=cc.project)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(wrong)", checks=cc.project)
        self.assertEqual(["PROJECT_NAME"], result)
        result = mock_lint(env, pkg, "project(mock) set(my_mock_var ON)", checks=cc.project)
        self.assertEqual(["LITERAL_PROJECT_NAME"], result)
        result = mock_lint(env, pkg, "project(mock) add_executable(mock mock.cpp)", checks=cc.project)
        self.assertEqual(["LITERAL_PROJECT_NAME"], result)
        result = mock_lint(env, pkg, "project(mock) add_executable(${PROJECT_NAME} mock.cpp)", checks=cc.project)
        self.assertEqual([], result)

    def test_special_vars(self):
        """Test checks for proper handling of special variables"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock)", checks=cc.special_vars)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) set(ENV{PATH} wrong)", checks=cc.special_vars)
        self.assertEqual(["IMMUTABLE_VAR"], result)
        result = mock_lint(env, pkg, "project(mock) set(PROJECT_NAME wrong)", checks=cc.special_vars)
        self.assertEqual(["IMMUTABLE_VAR"], result)
        result = mock_lint(env, pkg, "project(mock) set(CMAKE_BUILD_TYPE wrong)", checks=cc.special_vars)
        self.assertEqual(["CMAKE_BUILD_TYPE"], result)
        result = mock_lint(env, pkg, "project(mock) if(NOT CMAKE_BUILD_TYPE) set(CMAKE_BUILD_TYPE default) endif()", checks=cc.special_vars)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) set(CMAKE_CXX_FLAGS wrong)", checks=cc.special_vars)
        self.assertEqual(["CRITICAL_VAR_OVERWRITE"], result)
        result = mock_lint(env, pkg, "project(mock) unset(CMAKE_CXX_FLAGS)", checks=cc.special_vars)
        self.assertEqual(["CRITICAL_VAR_OVERWRITE"], result)
        result = mock_lint(env, pkg, "project(mock) set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} extra)", checks=cc.special_vars)
        self.assertEqual(["CRITICAL_VAR_APPEND"], result)
        result = mock_lint(env, pkg, "project(mock) list(GET CMAKE_PREFIX_PATH 0)", checks=cc.special_vars)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) list(LENGTH CMAKE_PREFIX_PATH len)", checks=cc.special_vars)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) list(APPEND CMAKE_PREFIX_PATH extra)", checks=cc.special_vars)
        self.assertEqual(["CRITICAL_VAR_APPEND"], result)
        result = mock_lint(env, pkg, "project(mock) list(REVERSE CMAKE_PREFIX_PATH)", checks=cc.special_vars)
        self.assertEqual(["CRITICAL_VAR_OVERWRITE"], result)
        result = mock_lint(env, pkg, "project(mock) list(INSERT PROJECT_NAME 0 wrong)", checks=cc.special_vars)
        self.assertEqual(["IMMUTABLE_VAR"], result)

    def test_global_vars(self):
        """Test global variable checks"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) option(${PROJECT_NAME}_option test OFF) set(${PROJECT_NAME}_global CACHE STRING)", checks=cc.global_vars)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) set(global CACHE STRING)", checks=cc.global_vars)
        self.assertEqual(["GLOBAL_VAR_COLLISION"], result)
        result = mock_lint(env, pkg, "project(mock) set(global \"value\")", checks=cc.global_vars)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) option(optional test OFF)", checks=cc.global_vars)
        self.assertEqual(["GLOBAL_VAR_COLLISION"], result)

    def test_singleton_command(self):
        """Test check for singleton commands"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock)", checks=cc.singleton_commands)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) project(mock2)", checks=cc.singleton_commands)
        self.assertEqual(["DUPLICATE_CMD"], result)

    def test_deprecated(self):
        """Test check for deprecated features"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "add_gtest()", checks=cc.deprecated)
        self.assertEqual(["DEPRECATED_CMD"], result)
        result = mock_lint(env, pkg, "add_nosetests()", checks=cc.deprecated)
        self.assertEqual(["DEPRECATED_CMD"], result)
        result = mock_lint(env, pkg, "download_test_data()", checks=cc.deprecated)
        self.assertEqual(["DEPRECATED_CMD"], result)
        result = mock_lint(env, pkg, "parse_arguments()", checks=cc.deprecated)
        self.assertEqual(["DEPRECATED_CMD"], result)

    def test_cmake_modules(self):
        """Test check for cmake_modules usage"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) find_package(NUMPY)", checks=cc.cmake_modules)
        self.assertEqual(["MISSING_CMAKE_MODULES"], result)
        result = mock_lint(env, pkg, "project(mock) find_package(Eigen)", checks=cc.cmake_modules)
        self.assertEqual(["DEPRECATED_CMAKE_MODULE"], result)

    def test_minimum_version(self):
        """Test check for CMake minimum compatible version"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "cmake_minimum_required(VERSION 2.8.12) project(mock)", checks=cc.minimum_version)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "project(mock) cmake_minimum_required(VERSION 2.8.12)", checks=cc.minimum_version)
        self.assertEqual(["ORDER_VIOLATION"], result)

    def test_endblock(self):
        """Test proper style for CMake code blocks"""
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
        self.assertEqual(["ENDBLOCK_ARGS"], result)
        result = mock_lint(env, pkg, "if(a) else(a) endif()", checks=cc.endblock)
        self.assertEqual(["ENDBLOCK_ARGS"], result)
        result = mock_lint(env, pkg, "if(a) else() endif(a)", checks=cc.endblock)
        self.assertEqual(["ENDBLOCK_ARGS"], result)
        result = mock_lint(env, pkg, "macro(a) endmacro(a)", checks=cc.endblock)
        self.assertEqual(["ENDBLOCK_ARGS"], result)
        result = mock_lint(env, pkg, "function(a) endfunction(a)", checks=cc.endblock)
        self.assertEqual(["ENDBLOCK_ARGS"], result)

    @posix_and_nt
    @patch("os.path.isfile", lambda x: x == os.path.normpath("/package-path/mock/FindLocal.cmake"))
    def test_cmake_includes(self):
        """Test CMake includes"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
                           {"/package-path/mock/CMakeLists.txt": """
            include(FindLocal.cmake)
            include(FindOptional.cmake OPTIONAL)
            include(FindPackageHandleStandardArgs)
            """,
                            "/package-path/mock/FindLocal.cmake": "",
                            },
                           checks=cc.cmake_includes)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
                           """
            include(missing.cmake)
            """,
                           checks=cc.cmake_includes)
        self.assertEqual(["MISSING_FILE"], result)

        result = mock_lint(env, pkg,
                           """
            include(FindStuff)
            """,
                           checks=cc.cmake_includes)
        self.assertEqual(["FIND_BY_INCLUDE"], result)
