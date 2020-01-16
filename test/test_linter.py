import unittest
import sys  # noqa
import os
from catkin_lint.linter import CMakeLinter, LintInfo, PathConstants
from .helper import create_env, create_manifest, mock_lint, patch, posix_and_nt
import catkin_lint.checks.build as cc
import catkin_lint.environment
from catkin_lint.cmake import CMakeSyntaxError
from catkin_pkg.package import Export


class LinterTest(unittest.TestCase):
    def test_circular_depend(self):
        """Test circular dependencies in custom linter modules"""
        def a(linter):
            linter.require(b)

        def b(linter):
            linter.require(a)
        linter = CMakeLinter(create_env())
        self.assertRaises(RuntimeError, linter.require, a)

    def test_lower_case(self):
        """Test check for lower-case command names"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
                           """
            PROJECT(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            """, checks=cc.all)
        self.assertEqual(["CMD_CASE"], result)

    def test_include(self):
        """Test edge cases for CMake include"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
                           """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            include()
            find_file(FOO_INCLUDE foo.cmake)
            include(${FOO_INCLUDE})
            """, checks=cc.all)
        self.assertEqual([], result)
        result = mock_lint(env, pkg,
                           """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            include(../foo.cmake)
            """, checks=cc.all)
        self.assertEqual(["EXTERNAL_FILE"], result)

    def test_pragma(self):
        """Test #catkin_lint: pragma handling"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
                           """
            #catkin_lint: ignore cmd_case
            PROJECT(mock)
            find_package(catkin REQUIRED)
            CATKIN_PACKAGE()
            """, checks=cc.all)
        self.assertEqual([], result)
        result = mock_lint(env, pkg,
                           """
            #catkin_lint: ignore cmd_case
            #catkin_lint: report cmd_case
            PROJECT(mock)
            find_package(catkin REQUIRED)
            CATKIN_PACKAGE()
            """, checks=cc.all)
        self.assertEqual(["CMD_CASE", "CMD_CASE"], result)
        result = mock_lint(env, pkg,
                           """
            #catkin_lint: ignore_once cmd_case
            PROJECT(mock)
            find_package(catkin REQUIRED)
            CATKIN_PACKAGE()
            """, checks=cc.all)
        self.assertEqual(["CMD_CASE"], result)
        result = mock_lint(env, pkg,
                           """
            #catkin_lint: ignore_once cmd_case
            project(mock)
            find_package(catkin REQUIRED)
            CATKIN_PACKAGE()
            """, checks=cc.all)
        self.assertEqual(["CMD_CASE"], result)

    def test_argparse_error(self):
        """Test invalid CMake command arguments"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
                           """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            add_executable(${PROJECT_NAME} IMPORTED)
            set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${empty_var}")
            """, checks=cc.all)
        self.assertEqual([], result)

    def test_if(self):
        """Test if()/else()/endif() block handling"""
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
            if ("foo" STREQUAL ${var})
            endif()
            """, checks=cc.all)
        self.assertEqual(["UNQUOTED_STRING_OP"], result)
        result = mock_lint(env, pkg,
                           """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            if (var STREQUAL foo)
            endif()
            """, checks=cc.all)
        self.assertEqual([], result)
        result = mock_lint(env, pkg,
                           """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            if (EXISTS filename)
            endif()
            """, checks=cc.all)
        self.assertEqual(["UNQUOTED_STRING_OP"], result)
        self.assertRaises(CMakeSyntaxError, mock_lint, env, pkg, "else()")
        self.assertRaises(CMakeSyntaxError, mock_lint, env, pkg, "endif()")
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
        self.assertEqual(["AMBIGUOUS_CONDITION"], result)
        result = mock_lint(env, pkg,
                           """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            if ("${varname}")
            endif()
            """, checks=cc.all)
        self.assertEqual(["AMBIGUOUS_CONDITION"], result)
        result = mock_lint(env, pkg,
                           """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            if ("${varname}${othervarname}")
            endif()
            """, checks=cc.all)
        self.assertEqual([], result)

    @posix_and_nt
    def test_package_path(self):
        """Test package path resolver"""
        env = create_env()
        info = LintInfo(env)
        info.var = {
            "CMAKE_CURRENT_SOURCE_DIR": PathConstants.PACKAGE_SOURCE,
        }
        self.assertEqual(info.source_relative_path("filename"), "filename")
        self.assertEqual(info.source_relative_path("subdir/filename"), "subdir/filename")
        self.assertEqual(info.source_relative_path("subdir/../filename"), "filename")
        self.assertEqual(info.source_relative_path("/filename"), "/filename")
        self.assertEqual(info.source_relative_path("../../../../filename"), "/filename")
        self.assertEqual(info.source_relative_path("../../../../subdir/filename"), "/subdir/filename")
        info.var = {
            "CMAKE_CURRENT_SOURCE_DIR": "%s/subdir" % PathConstants.PACKAGE_SOURCE,
        }
        self.assertEqual(info.source_relative_path("filename"), "subdir/filename")
        self.assertEqual(info.source_relative_path("../filename"), "filename")

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
                           """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            """, checks=cc.all, package_path="/package-path/other")
        self.assertEqual(["PACKAGE_PATH_NAME"], result)

    def test_list(self):
        """Test CMake list handling"""
        env = create_env()
        linter = CMakeLinter(env)
        info = LintInfo(env)
        linter._handle_list(info, ["APPEND", "test", "one"])
        self.assertEqual(info.var["test"], "one")
        linter._handle_list(info, ["APPEND", "test", "three"])
        self.assertEqual(info.var["test"], "one;three")
        linter._handle_list(info, ["INSERT", "test", "1", "two"])
        self.assertEqual(info.var["test"], "one;two;three")
        linter._handle_list(info, ["GET", "test", "1", "result"])
        self.assertEqual(info.var["result"], "two")
        linter._handle_list(info, ["GET", "test", "42", "result"])
        self.assertEqual(info.var["result"], "")
        linter._handle_list(info, ["FIND", "test", "none", "result"])
        self.assertEqual(info.var["result"], "-1")
        linter._handle_list(info, ["FIND", "test", "two", "result"])
        self.assertEqual(info.var["result"], "1")
        linter._handle_list(info, ["PREPEND", "test", "zero"])
        self.assertEqual(info.var["test"], "zero;one;two;three")
        linter._handle_list(info, ["POP_FRONT", "test"])
        self.assertEqual(info.var["test"], "one;two;three")
        linter._handle_list(info, ["POP_BACK", "test"])
        self.assertEqual(info.var["test"], "one;two")

        info.var["test"] = "one;two;three;four;five"
        linter._handle_list(info, ["POP_FRONT", "test", "v1", "v2"])
        self.assertEqual(info.var["test"], "three;four;five")
        self.assertEqual(info.var["v1"], "one")
        self.assertEqual(info.var["v2"], "two")
        linter._handle_list(info, ["POP_BACK", "test", "v1", "v2"])
        self.assertEqual(info.var["test"], "three")
        self.assertEqual(info.var["v1"], "five")
        self.assertEqual(info.var["v2"], "four")

        info.var["test"] = "one;two;three;one;four;five"
        linter._handle_list(info, ["REMOVE_DUPLICATES", "test"])
        self.assertEqual(info.var["test"], "one;two;three;four;five")
        linter._handle_list(info, ["JOIN", "test", "+", "result"])
        self.assertEqual(info.var["result"], "one+two+three+four+five")
        linter._handle_list(info, ["SUBLIST", "test", "2", "2", "result"])
        self.assertEqual(info.var["result"], "three;four")
        linter._handle_list(info, ["SUBLIST", "test", "-2", "3", "result"])
        self.assertEqual(info.var["result"], "four;five")
        linter._handle_list(info, ["SUBLIST", "test", "0", "-1", "result"])
        self.assertEqual(info.var["result"], "one;two;three;four;five")
        linter._handle_list(info, ["REMOVE_AT", "test", "15", "0", "4", "2"])
        self.assertEqual(info.var["test"], "two;four")
        linter._handle_list(info, ["APPEND", "test", "two"])
        linter._handle_list(info, ["REMOVE_ITEM", "test", "two"])
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
        """Test environment variable handling"""
        env = create_env()
        pkg = create_manifest("catkin")
        result = mock_lint(env, pkg,
                           """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            set(bla $ENV{PATH})
            """, checks=cc.all)
        self.assertEqual(["ENV_VAR"], result)

    @posix_and_nt
    @patch("os.path.isfile", lambda x: x == os.path.normpath("/package-path/catkin/broken.cmake"))
    def test_blacklist(self):
        """Test CMake inclusion blacklist"""
        env = create_env()
        pkg = create_manifest("catkin")
        result = mock_lint(env, pkg,
                           {
                               "/package-path/catkin/CMakeLists.txt": "project(catkin) include(broken.cmake RESULT_VARIABLE gone) catkin_package()",
                               "/package-path/catkin/broken.cmake": "xxxxxx syntax error xxxxx"
                           }, checks=cc.all
                           )
        self.assertEqual([], result)

    @posix_and_nt
    @patch("os.path.isdir", lambda x: x == "/" or x == "\\")
    @patch("os.path.realpath", lambda x: x)
    def test_environment(self):
        """Test catkin environment"""
        env = catkin_lint.environment.CatkinEnvironment(use_rosdep=False)
        mock_packages = {}
        mock_packages[os.path.normpath("/mock_catkin")] = create_manifest("mock_catkin")
        mock_packages[os.path.normpath("/mock_other")] = create_manifest("mock_other")
        mock_packages[os.path.normpath("/mock_other")].exports += [Export("random_tag"), Export("build_type", "cmake")]
        with patch("catkin_lint.environment.find_packages", lambda x, use_cache: mock_packages):
            result = env.add_path(os.path.normpath("/"))
            self.assertEqual(1, len(result))
            self.assertTrue(env.is_catkin_pkg("mock_catkin"))
            self.assertFalse(env.is_catkin_pkg("mock_other"))
            result = env.add_path(os.path.normpath("/"))
            self.assertEqual(1, len(result))
            self.assertTrue(env.is_catkin_pkg("mock_catkin"))
            self.assertFalse(env.is_catkin_pkg("mock_other"))
            result = env.add_path(os.path.normpath("/missing"))
            self.assertEqual([], result)
            self.assertFalse(env.is_catkin_pkg("invalid"))

        def raiseError():
            raise RuntimeError()
        with open(os.devnull, "w") as devnull:
            with patch("catkin_lint.environment.get_rosdep", raiseError):
                with patch("sys.stderr", devnull):
                    env = catkin_lint.environment.CatkinEnvironment()
                    self.assertFalse(env.ok)
        self.assertFalse(catkin_lint.environment.is_catkin_package(None))

    @posix_and_nt
    @patch("os.path.isdir", lambda x: x in [os.path.normpath(p) for p in ["/package-path/mock/src", "/package-path/mock/src/2ndlevel", "/package-path/mock/include"]])
    @patch("os.path.isfile", lambda x: x in [os.path.normpath(p) for p in ["/other-path/CMakeLists.txt", "/package-path/mock/src/CMakeLists.txt", "/package-path/mock/src/2ndlevel/CMakeLists.txt", "/package-path/mock/src/source.cpp", "/package-path/mock/src/2ndlevel/source2.cpp"]])
    def test_subdir(self):
        """Test add_subdirectory()"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
                           {
                               "/package-path/mock/CMakeLists.txt": "project(mock) add_subdirectory(src)",
                               "/package-path/mock/src/CMakeLists.txt": """
              include_directories(../include)
              find_package(catkin REQUIRED)
              catkin_package()
              add_executable(${PROJECT_NAME}_test source.cpp)
              install(DIRECTORY ../include DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
              """
                           }, checks=cc.all
                           )
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
                           {
                               "/package-path/mock/CMakeLists.txt": "project(mock) add_subdirectory(src)",
                               "/package-path/mock/src/CMakeLists.txt": "add_subdirectory(2ndlevel)",
                               "/package-path/mock/src/2ndlevel/CMakeLists.txt": """\
              find_package(catkin REQUIRED)
              catkin_package()
              add_executable(${PROJECT_NAME}_test source2.cpp)
              """
                           }, checks=cc.all
                           )
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
                           {
                               "/package-path/mock/CMakeLists.txt": "project(mock) add_subdirectory(src)",
                               "/package-path/mock/src/CMakeLists.txt": """
              include_directories(../include)
              find_package(catkin REQUIRED)
              catkin_package()
              add_executable(${PROJECT_NAME}_test source.cpp)
              add_subdirectory(../src)
              """
                           }, checks=cc.all
                           )
        self.assertEqual(["DUPLICATE_SUBDIR"], result)

        result = mock_lint(env, pkg,
                           {
                               "/package-path/mock/CMakeLists.txt": "project(mock) add_subdirectory(/other-path)",
                               "/other-path/CMakeLists.txt": """
              find_package(catkin REQUIRED)
              catkin_package()
              """
                           }, checks=cc.all
                           )
        self.assertEqual(["EXTERNAL_SUBDIR"], result)

        result = mock_lint(env, pkg,
                           """
              project(mock)
              find_package(catkin REQUIRED)
              catkin_package()
              add_subdirectory(missing_subdir)
              """, checks=cc.all
                           )
        self.assertEqual(["MISSING_SUBDIR"], result)

        result = mock_lint(env, pkg,
                           {
                               "/package-path/mock/CMakeLists.txt": """
              project(mock)
              find_package(catkin REQUIRED)
              catkin_package()
              add_subdirectory(src)
              """,
                               "/package-path/mock/src/CMakeLists.txt": """
              project(submock)
              """
                           }, checks=cc.all
                           )
        self.assertEqual(["SUBPROJECT"], result)

        var = mock_lint(env, pkg,
                        {
                            "/package-path/mock/CMakeLists.txt": """
              project(mock)
              set(foo "toplevel")
              add_subdirectory(src)
              """,
                            "/package-path/mock/src/CMakeLists.txt": """
              set(foo "subdir")
              find_file(bar bar.txt)
              """
                        }, checks=None, return_var=True
                        )
        self.assertEqual("toplevel", var["foo"])
        self.assertFalse("bar" in var)
        var = mock_lint(env, pkg,
                        {
                            "/package-path/mock/CMakeLists.txt": """
              project(mock)
              set(foo "toplevel")
              add_subdirectory(src)
              """,
                            "/package-path/mock/src/CMakeLists.txt": """
              set(foo "subdir" PARENT_SCOPE)
              """
                        }, checks=None, return_var=True
                        )
        self.assertEqual("subdir", var["foo"])
