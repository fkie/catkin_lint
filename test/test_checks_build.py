import unittest
import catkin_lint.checks.build as cc
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

class ChecksBuildTest(unittest.TestCase):

    @patch("os.path.isdir", lambda x: x == os.path.normpath("/mock-path/include"))
    def do_includes(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "include_directories(include)", checks=cc.includes)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "find_package(catkin REQUIRED) include_directories(${catkin_INCLUDE_DIRS})", checks=cc.includes)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "include_directories(missing_include)", checks=cc.includes)
        self.assertEqual([ "MISSING_BUILD_INCLUDE_PATH" ], result)


    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/src/existing.cpp"))
    def do_source_files(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "add_executable(mock src/existing.cpp) add_library(mock_lib src/existing.cpp)", checks=cc.source_files)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "add_executable(mock ${CMAKE_CURRENT_SOURCE_DIR}/src/existing.cpp) add_library(mock_lib ${CMAKE_CURRENT_SOURCE_DIR}/src/existing.cpp)", checks=cc.source_files)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "add_executable(mock src/missing.cpp)", checks=cc.source_files)
        self.assertEqual([ "MISSING_FILE" ], result)
        result = mock_lint(env, pkg, "add_library(mock src/missing.cpp)", checks=cc.source_files)
        self.assertEqual([ "MISSING_FILE" ], result)


    @patch("os.path.isdir", lambda x: x == os.path.normpath("/mock-path/in_package"))
    def do_link_directories(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "link_directories(in_package)", checks=cc.link_directories)
        self.assertEqual([ "LINK_DIRECTORY" ], result)
        result = mock_lint(env, pkg, "link_directories(/not/in/package)", checks=cc.link_directories)
        self.assertEqual([ "EXTERNAL_LINK_DIRECTORY" ], result)


    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/FindLocal.cmake"))
    def do_depends(self):
        env = create_env()
        pkg = create_manifest("mock", build_depends=[ "other_catkin" ])

        result = mock_lint(env, pkg, 
            """
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            """,
        checks=cc.depends)
        self.assertEqual([ "ORDER_VIOLATION" ], result)

        result = mock_lint(env, pkg, 
            """
            project(mock)
            catkin_package()
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            """,
        checks=cc.depends)
        self.assertEqual([ "ORDER_VIOLATION" ], result)

        result = mock_lint(env, pkg, 
            """
            project(mock)
            find_package(catkin COMPONENTS other_catkin)
            """,
        checks=cc.depends)
        self.assertEqual([ "MISSING_REQUIRED" ], result)

        result = mock_lint(env, pkg, 
            """
            project(mock)
            find_package(catkin REQUIRED other_catkin)
            """,
        checks=cc.depends)
        self.assertEqual([ "MISSING_COMPONENTS", "UNCONFIGURED_BUILD_DEPEND" ], result)

        result = mock_lint(env, pkg, 
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            find_package(other_catkin)
            """,
        checks=cc.depends)
        self.assertEqual([ "DUPLICATE_FIND" ], result)

        result = mock_lint(env, pkg, 
            """
            project(mock)
            find_package(other_catkin)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            """,
        checks=cc.depends)
        self.assertEqual([ "DUPLICATE_FIND" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS other_system)
            find_package(other_catkin)
            """,
        checks=cc.depends)
        self.assertEqual([ "NO_CATKIN_COMPONENT", "MISSING_DEPEND" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            """,
        checks=cc.depends)
        self.assertEqual([ "UNCONFIGURED_BUILD_DEPEND" ], result)


    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/src/mock.cpp"))
    def do_targets(self):
        env = create_env()
        pkg = create_manifest("mock", build_depends=[ "other_catkin" ], run_depends=[ "other_catkin" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            catkin_package()
            include_directories(${catkin_INCLUDE_DIRS})
            add_executable(mock/prog src/mock.cpp)
            set_target_properties(mock/prog PROPERTIES OUTPUT_NAME "prog")
            target_link_libraries(mock/prog ${catkin_LIBRARIES})
            """,
        checks=cc.targets)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            catkin_package()
            add_executable(mock_prog src/mock.cpp)
            target_link_libraries(mock_prog ${catkin_LIBRARIES})
            """,
        checks=cc.targets)
        self.assertEqual([ "MISSING_CATKIN_INCLUDE" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            catkin_package()
            include_directories(${catkin_INCLUDE_DIRS})
            add_executable(mock_prog src/mock.cpp)
            target_link_libraries(mock_prog ${catkin_LIBRARIES})
            """,
        checks=cc.targets)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            add_executable(mock_prog src/mock.cpp)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            catkin_package()
            include_directories(${catkin_INCLUDE_DIRS})
            target_link_libraries(mock_prog ${catkin_LIBRARIES})
            """,
        checks=cc.targets)
        self.assertEqual([ "CATKIN_ORDER_VIOLATION", "ORDER_VIOLATION" ], result)

        pkg = create_manifest("mock", meta=True)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_metapackage()
            add_executable(mock_prog src/mock.cpp)
            """,
        checks=cc.targets)
        self.assertEqual([ "INVALID_META_COMMAND" ], result)


    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/src/mock.cpp"))
    def do_name_check(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            add_executable(mock/prog src/mock.cpp)
            target_link_libraries(mock/prog ${catkin_LIBRARIES})
            """,
        checks=cc.name_check)
        self.assertEqual([ "INVALID_TARGET_OUTPUT" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            add_executable(prog src/mock.cpp)
            target_link_libraries(prog ${catkin_LIBRARIES})
            """,
        checks=cc.name_check)
        self.assertEqual([ "TARGET_NAME_COLLISION" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            add_library(libmock src/mock.cpp)
            """,
        checks=cc.name_check)
        self.assertEqual([ "REDUNDANT_LIB_PREFIX" ], result)


    @patch("os.path.isfile", lambda x: x in [ os.path.normpath("/mock-path/bin/script"), os.path.normpath("/mock-path/share/file"), os.path.normpath("/mock-path/src/mock.cpp") ])
    @patch("os.path.isdir", lambda x: x == os.path.normpath("/mock-path/include"))
    def do_installs(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            include_directories(include)
            catkin_package(INCLUDE_DIRS include)
            add_executable(mock src/mock.cpp)
            install(PROGRAMS bin/script DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
            install(FILES share/file DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
            install(TARGETS mock RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
            install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
            """,
        checks=cc.installs)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            install(PROGRAMS bin/script DESTINATION bin)
            """,
        checks=cc.installs)
        self.assertEqual([ "INSTALL_DESTINATION" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            install(PROGRAMS bin/script DESTINATION "${missing_variable}")
            """,
        checks=cc.installs)
        self.assertEqual([ "INSTALL_DESTINATION" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package(LIBRARIES mock)
            add_library(mock src/mock.cpp)
            add_executable(mock_prog src/mock.cpp)
            """,
        checks=cc.installs)
        self.assertEqual([ "UNINSTALLED_EXPORT_LIB", "MISSING_INSTALL_TARGET" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package(INCLUDE_DIRS include)
            add_executable(test_mock src/mock.cpp)
            """,
        checks=cc.installs)
        self.assertEqual([ "MISSING_BUILD_INCLUDE", "MISSING_INSTALL_INCLUDE" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            add_library(mock_lib src/mock.cpp)
            add_executable(mock src/mock.cpp)
            target_link_libraries(mock mock_lib)
            install(TARGETS mock RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
            """,
        checks=cc.installs)
        self.assertEqual([ "UNINSTALLED_DEPEND" ], result)


    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/src/mock.cpp"))
    @patch("os.path.isdir", lambda x: x == os.path.normpath("/mock-path/include"))
    def do_exports(self):
        env = create_env()
        pkg = create_manifest("mock", build_depends=[ "other_catkin", "other_system" ], run_depends=[ "other_catkin", "other_system" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin)
            find_package(other_system)
            catkin_package(
            INCLUDE_DIRS include
            CATKIN_DEPENDS other_catkin
            DEPENDS other_system
            LIBRARIES mock
            )
            add_library(mock src/mock.cpp)
            """,
        checks=cc.exports)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin)
            find_package(other_system)
            catkin_package(
            INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}
            CATKIN_DEPENDS other_catkin
            DEPENDS other_system
            )
            """,
        checks=cc.exports)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            find_package(other_system)
            find_path(Stuff_INCLUDE_DIRS stuff.h)
            find_library(Stuff_LIBRARIES stuff)
            catkin_package(
            CATKIN_DEPENDS other_catkin
            DEPENDS other_system Stuff
            )
            """,
        checks=cc.exports)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin)
            find_package(other_system)
            catkin_package(
            INCLUDE_DIRS missing_include
            CATKIN_DEPENDS other_catkin
            DEPENDS other_system
            )
            """,
        checks=cc.exports)
        self.assertEqual([ "MISSING_EXPORT_INCLUDE_PATH" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin)
            find_package(other_system)
            catkin_package(
            DEPENDS other_catkin other_system
            )
            """,
        checks=cc.exports)
        self.assertEqual([ "CATKIN_AS_SYSTEM_DEPEND" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin)
            find_package(other_system)
            catkin_package(
            CATKIN_DEPENDS other_catkin other_system
            )
            """,
        checks=cc.exports)
        self.assertEqual([ "SYSTEM_AS_CATKIN_DEPEND" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin)
            catkin_package(
            CATKIN_DEPENDS other_catkin
            DEPENDS other_system
            )
            """,
        checks=cc.exports)
        self.assertEqual([ "UNCONFIGURED_SYSTEM_DEPEND" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin)
            catkin_package(
            CATKIN_DEPENDS other_catkin
            INCLUDE_DIRS /not/in/package
            )
            """,
        checks=cc.exports)
        self.assertEqual([ "EXTERNAL_INCLUDE_PATH" ], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package(CATKIN_DEPENDS other_catkin)
            """,
        checks=cc.exports)
        self.assertEqual([ "MISSING_DEPEND" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            pkg_check_modules(FOO foo)
            catkin_package(DEPENDS FOO)
            """, 
        checks=cc.exports)
        self.assertEqual([ "EXPORTED_PKG_CONFIG"], result)

        pkg = create_manifest("mock", build_depends=[ "other_msgs" ], run_depends=[ "other_msgs"])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS other_msgs)
            catkin_package()
            """,
        checks=cc.exports)
        self.assertEqual([ "SUGGEST_CATKIN_DEPEND" ], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package(INCLUDE_DIRS include)
            include_directories(include)
            add_library(mock src/mock.cpp)
            """,
        checks=cc.exports)
        self.assertEqual([ "MISSING_EXPORT_LIB" ], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package(INCLUDE_DIRS include LIBRARIES mock)
            include_directories(include)
            add_library(mock src/mock.cpp)
            set_target_properties(mock PROPERTIES OUTPUT_NAME "renamed")
            """,
        checks=cc.exports)
        self.assertEqual([ "EXPORT_LIB_RENAMED" ], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package(INCLUDE_DIRS include LIBRARIES mock)
            include_directories(include)
            add_executable(mock src/mock.cpp)
            """,
        checks=cc.exports)
        self.assertEqual([ "EXPORT_LIB_NOT_LIB" ], result)


    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/config.xml"))
    def do_plugins(self):
        from catkin_pkg.package import Export
        env = create_env()
        pkg = create_manifest("mock", run_depends=[ "other_catkin" ])
        plugin = Export("other_catkin")
        plugin.attributes = { "plugin": "${prefix}/config.xml" }
        pkg.exports += [ plugin ]
        result = mock_lint(env, pkg, "install(FILES config.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})", checks=cc.plugins)
        self.assertEqual([], result)

        result = mock_lint(env, pkg, "", checks=cc.plugins)
        self.assertEqual([ "PLUGIN_MISSING_INSTALL" ], result)

        result = mock_lint(env, pkg, "install(FILES config.xml DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})", checks=cc.plugins)
        self.assertEqual([ "PLUGIN_MISSING_INSTALL" ], result)

        pkg = create_manifest("mock", run_depends=[ "other_catkin" ])
        plugin = Export("other_catkin")
        plugin.attributes = { "plugin": "config.xml" }
        pkg.exports += [ plugin ]
        result = mock_lint(env, pkg, "install(FILES config.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})", checks=cc.plugins)
        self.assertEqual([ "PLUGIN_EXPORT_PREFIX" ], result)

        pkg = create_manifest("mock")
        plugin = Export("other_catkin")
        plugin.attributes = { "plugin": "${prefix}/config.xml" }
        pkg.exports += [ plugin ]
        result = mock_lint(env, pkg, "install(FILES config.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})", checks=cc.plugins)
        self.assertEqual([ "PLUGIN_DEPEND" ], result)

        pkg = create_manifest("mock", run_depends=[ "other_catkin" ])
        plugin = Export("other_catkin")
        plugin.attributes = { "plugin": "${prefix}/missing_config.xml" }
        pkg.exports += [ plugin ]
        result = mock_lint(env, pkg, "install(FILES missing_config.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})", checks=cc.plugins)
        self.assertEqual([ "PLUGIN_MISSING_FILE" ], result)


    def test_message_generation(self):
        env = create_env()
        pkg = create_manifest("mock", build_depends=[ "message_generation", "other_catkin" ], run_depends=[ "message_runtime", "other_catkin" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation)
            find_package(other_catkin)
            add_message_files(FILES mock.msg)
            generate_messages(DEPENDENCIES other_catkin)
            catkin_package(CATKIN_DEPENDS other_catkin message_runtime)
            """,
        checks=cc.message_generation)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation)
            find_package(other_catkin)
            generate_messages(DEPENDENCIES other_catkin)
            add_message_files(FILES mock.msg)
            catkin_package(CATKIN_DEPENDS other_catkin message_runtime)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "ORDER_VIOLATION" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation)
            find_package(other_catkin)
            catkin_package(CATKIN_DEPENDS other_catkin message_runtime)
            add_message_files(FILES mock.msg)
            generate_messages(DEPENDENCIES other_catkin)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "ORDER_VIOLATION", "ORDER_VIOLATION" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(message_generation)
            find_package(other_catkin)
            add_message_files(FILES mock.msg)
            generate_messages(DEPENDENCIES other_catkin)
            find_package(catkin REQUIRED)
            catkin_package(CATKIN_DEPENDS other_catkin message_runtime)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "CATKIN_ORDER_VIOLATION", "CATKIN_ORDER_VIOLATION" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation)
            find_package(other_catkin)
            add_message_files(FILES mock.msg)
            generate_messages(DEPENDENCIES other_catkin)
            catkin_package(CATKIN_DEPENDS message_runtime)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "MISSING_MSG_CATKIN" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation)
            find_package(other_catkin)
            add_message_files(FILES mock.msg)
            catkin_package(CATKIN_DEPENDS other_catkin message_runtime)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "MISSING_GENERATE_MSG" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation)
            find_package(other_catkin)
            generate_messages(DEPENDENCIES other_catkin)
            catkin_package(CATKIN_DEPENDS other_catkin message_runtime)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "UNUSED_GENERATE_MSG" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation)
            find_package(other_catkin)
            add_message_files(FILES mock.msg)
            generate_messages(DEPENDENCIES other_catkin)
            catkin_package(CATKIN_DEPENDS other_catkin)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "MISSING_CATKIN_DEPEND" ], result)

        pkg = create_manifest("mock")
        pkg = create_manifest("mock", meta=True)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            add_message_files(FILES mock.msg)
            generate_messages()
            catkin_metapackage()
            """,
        checks=cc.message_generation)
        self.assertEqual([ "INVALID_META_COMMAND", "INVALID_META_COMMAND" ], result)

        pkg = create_manifest("mock", build_depends=[ "other_catkin" ], run_depends=[ "message_runtime", "other_catkin" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin)
            add_message_files(FILES mock.msg)
            generate_messages(DEPENDENCIES other_catkin)
            catkin_package(CATKIN_DEPENDS other_catkin message_runtime)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "UNCONFIGURED_BUILD_DEPEND" ], result)

        pkg = create_manifest("mock", build_depends=[ "message_generation" ], run_depends=[ "message_runtime" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS message_generation)
            add_message_files(FILES mock.msg)
            generate_messages(DEPENDENCIES other_catkin)
            catkin_package(CATKIN_DEPENDS other_catkin message_runtime)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "MISSING_DEPEND", "UNCONFIGURED_MSG_DEPEND", "MISSING_MSG_DEPEND", "MISSING_MSG_DEPEND" ], result)


    @patch("os.path", posixpath)
    def test_posix(self):
        self.do_includes()
        self.do_source_files()
        self.do_link_directories()
        self.do_depends()
        self.do_targets()
        self.do_name_check()
        self.do_installs()
        self.do_exports()
        self.do_plugins()

    @patch("os.path", ntpath)
    def test_windows(self):
        self.do_includes()
        self.do_source_files()
        self.do_link_directories()
        self.do_depends()
        self.do_targets()
        self.do_name_check()
        self.do_installs()
        self.do_exports()
        self.do_plugins()
