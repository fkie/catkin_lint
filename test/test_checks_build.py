import unittest
import catkin_lint.checks.build as cc
from .helper import create_env, create_manifest, create_manifest2, mock_lint

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


    @patch("os.path.isfile", lambda x: x in [os.path.normpath("/mock-path/src/a.cpp"), os.path.normpath("/mock-path/src/b.cpp")])
    def do_source_files(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "add_executable(mock src/a.cpp src/b.cpp) add_library(mock_lib src/a.cpp src/b.cpp)", checks=cc.source_files)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "add_executable(mock ${CMAKE_CURRENT_SOURCE_DIR}/src/a.cpp) add_library(mock_lib ${CMAKE_CURRENT_SOURCE_DIR}/src/a.cpp)", checks=cc.source_files)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "add_executable(mock src/missing.cpp)", checks=cc.source_files)
        self.assertEqual([ "MISSING_FILE" ], result)
        result = mock_lint(env, pkg, "add_library(mock src/missing.cpp)", checks=cc.source_files)
        self.assertEqual([ "MISSING_FILE" ], result)
        result = mock_lint(env, pkg, "add_executable(mock src/b.cpp src/a.cpp)", checks=cc.source_files)
        self.assertEqual([ "UNSORTED_LIST" ], result)
        result = mock_lint(env, pkg, "add_library(mock src/b.cpp src/a.cpp)", checks=cc.source_files)
        self.assertEqual([ "UNSORTED_LIST" ], result)


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
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            find_package(unknown_package REQUIRED)
            """,
        checks=cc.depends)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(other_catkin REQUIRED)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            """,
        checks=cc.depends)
        self.assertEqual([ "DUPLICATE_FIND" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS other_system)
            find_package(other_catkin REQUIRED)
            """,
        checks=cc.depends)
        self.assertEqual([ "NO_CATKIN_COMPONENT", "MISSING_DEPEND" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS unknown_package)
            find_package(other_catkin REQUIRED)
            """,
        checks=cc.depends)
        self.assertEqual([ "UNKNOWN_PACKAGE" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            """,
        checks=cc.depends)
        self.assertEqual([ "UNCONFIGURED_BUILD_DEPEND" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin)
            """,
        checks=cc.depends)
        self.assertEqual([ "MISSING_REQUIRED" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin)
            if(other_catkin_FOUND)
            endif()
            """,
        checks=cc.depends)
        self.assertEqual([], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            if(CATKIN_ENABLE_TESTING)
            find_package(other_catkin REQUIRED)
            endif()
            """,
        checks=cc.depends)
        self.assertEqual(["UNCONFIGURED_BUILD_DEPEND"], result)

        pkg = create_manifest("mock", test_depends=[ "other_catkin" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            if(CATKIN_ENABLE_TESTING)
            find_package(other_catkin REQUIRED)
            endif()
            """,
        checks=cc.depends)
        self.assertEqual([], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin REQUIRED)
            """,
        checks=cc.depends)
        self.assertEqual(["UNGUARDED_TEST_DEPEND"], result)
        pkg = create_manifest("mock", build_depends=[ "other_catkin"], test_depends=[ "other_catkin" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            if(CATKIN_ENABLE_TESTING)
            find_package(other_catkin REQUIRED)
            endif()
            """,
        checks=cc.depends)
        self.assertEqual(["UNCONFIGURED_BUILD_DEPEND"], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin REQUIRED)
            """,
        checks=cc.depends)
        self.assertEqual([], result)
        pkg = create_manifest("mock", build_depends=[ "first_pkg", "second_pkg" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS first_pkg second_pkg)
            """,
        checks=cc.depends)
        self.assertEqual([], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS second_pkg first_pkg)
            """,
        checks=cc.depends)
        self.assertEqual([ "UNSORTED_LIST" ], result)


    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/src/source.cpp"))
    def do_targets(self):
        env = create_env()
        pkg = create_manifest("mock", build_depends=[ "other_catkin" ], run_depends=[ "other_catkin" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            catkin_package()
            include_directories(${catkin_INCLUDE_DIRS})
            add_executable(${PROJECT_NAME}/prog src/source.cpp)
            set_target_properties(${PROJECT_NAME}/prog PROPERTIES OUTPUT_NAME "prog")
            target_link_libraries(${PROJECT_NAME}/prog ${catkin_LIBRARIES})
            """,
        checks=cc.targets)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            catkin_package()
            add_executable(${PROJECT_NAME}_prog src/source.cpp)
            target_link_libraries(${PROJECT_NAME}_prog ${catkin_LIBRARIES})
            """,
        checks=cc.targets)
        self.assertEqual([ "MISSING_CATKIN_INCLUDE" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            catkin_package()
            include_directories(${catkin_INCLUDE_DIRS} ${other_catkin_INCLUDE_DIRS})
            add_executable(${PROJECT_NAME}_prog src/source.cpp)
            target_link_libraries(${PROJECT_NAME}_prog ${catkin_LIBRARIES})
            """,
        checks=cc.targets)
        self.assertEqual([ "DUPLICATE_BUILD_INCLUDE" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            catkin_package()
            include_directories(${catkin_INCLUDE_DIRS})
            add_executable(${PROJECT_NAME}_prog src/source.cpp)
            target_link_libraries(${PROJECT_NAME}_prog ${catkin_LIBRARIES})
            """,
        checks=cc.targets)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            add_executable(${PROJECT_NAME}_prog src/source.cpp)
            find_package(catkin REQUIRED COMPONENTS other_catkin)
            catkin_package()
            include_directories(${catkin_INCLUDE_DIRS})
            target_link_libraries(${PROJECT_NAME}_prog ${catkin_LIBRARIES})
            """,
        checks=cc.targets)
        self.assertEqual([ "CATKIN_ORDER_VIOLATION", "ORDER_VIOLATION" ], result)

        pkg = create_manifest("mock", meta=True)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_metapackage()
            add_executable(${PROJECT_NAME}_prog src/source.cpp)
            """,
        checks=cc.targets)
        self.assertEqual([ "INVALID_META_COMMAND" ], result)


    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/src/source.cpp"))
    def do_name_check(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            add_executable(${PROJECT_NAME}/prog src/source.cpp)
            target_link_libraries(${PROJECT_NAME}/prog ${catkin_LIBRARIES})
            """,
        checks=cc.name_check)
        self.assertEqual([ "INVALID_TARGET_OUTPUT" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            add_executable(prog src/source.cpp)
            target_link_libraries(prog ${catkin_LIBRARIES})
            """,
        checks=cc.name_check)
        self.assertEqual([ "TARGET_NAME_COLLISION" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            add_library(lib${PROJECT_NAME} src/source.cpp)
            """,
        checks=cc.name_check)
        self.assertEqual([ "REDUNDANT_LIB_PREFIX" ], result)


    @patch("os.path.isfile", lambda x: x in [ os.path.normpath("/mock-path/bin/script"), os.path.normpath("/mock-path/share/file"), os.path.normpath("/mock-path/src/source.cpp") ])
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
            add_executable(${PROJECT_NAME} src/source.cpp)
            add_executable(test_${PROJECT_NAME} src/source.cpp)
            add_executable(${PROJECT_NAME}_example src/source.cpp)
            install(PROGRAMS bin/script DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
            install(FILES share/file DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
            install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
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
            catkin_package(LIBRARIES ${PROJECT_NAME})
            add_library(${PROJECT_NAME} src/source.cpp)
            add_executable(${PROJECT_NAME}_prog src/source.cpp)
            """,
        checks=cc.installs)
        self.assertEqual([ "UNINSTALLED_EXPORT_LIB", "MISSING_INSTALL_TARGET" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package(INCLUDE_DIRS include)
            add_executable(test_${PROJECT_NAME} src/source.cpp)
            """,
        checks=cc.installs)
        self.assertEqual([ "MISSING_BUILD_INCLUDE", "MISSING_INSTALL_INCLUDE" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            add_library(${PROJECT_NAME}_lib src/source.cpp)
            add_executable(${PROJECT_NAME} src/source.cpp)
            target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
            install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
            """,
        checks=cc.installs)
        self.assertEqual([ "UNINSTALLED_DEPEND" ], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
            """,
        checks=cc.installs)
        self.assertEqual([ "UNDEFINED_INSTALL_TARGET" ], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package()
            add_library(${PROJECT_NAME}_target1 src/source.cpp)
            add_executable(${PROJECT_NAME}_target2 src/source.cpp)
            install(TARGETS ${PROJECT_NAME}_target2 ${PROJECT_NAME}_target1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
            """,
        checks=cc.installs)
        self.assertEqual([ "UNSORTED_LIST" ], result)


    def test_tests(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_download_test_data()
            """,
        checks=cc.tests)
        self.assertEqual(["UNGUARDED_TEST_CMD"], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            if(CATKIN_ENABLE_TESTING)
            add_rostest()
            endif()
            """,
        checks=cc.tests)
        self.assertEqual(["MISSING_DEPEND"], result)


    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/src/source.cpp"))
    @patch("os.path.isdir", lambda x: x in [ os.path.normpath("/mock-path/include"), os.path.normpath("/mock-path/include/mock") ])
    def do_exports(self):
        env = create_env()
        pkg = create_manifest("mock", build_depends=[ "other_catkin", "other_system" ], run_depends=[ "other_catkin", "other_system" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin REQUIRED)
            find_package(other_system REQUIRED)
            catkin_package(
            INCLUDE_DIRS include
            CATKIN_DEPENDS other_catkin
            DEPENDS other_system
            LIBRARIES ${PROJECT_NAME}
            )
            include_directories(include)
            add_library(${PROJECT_NAME} src/source.cpp)
            """,
        checks=cc.exports)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin REQUIRED)
            find_package(other_system REQUIRED)
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
            find_package(other_system REQUIRED)
            find_path(Stuff_INCLUDE_DIRS stuff.h)
            find_library(Stuff_LIBRARIES stuff)
            catkin_package(
            CATKIN_DEPENDS other_catkin
            DEPENDS Stuff other_system
            )
            """,
        checks=cc.exports)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin REQUIRED)
            find_package(other_system REQUIRED)
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
            find_package(other_catkin REQUIRED)
            find_package(other_system REQUIRED)
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
            find_package(other_catkin REQUIRED)
            find_package(other_system REQUIRED)
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
            find_package(other_catkin REQUIRED)
            find_package(unknown_package REQUIRED)
            catkin_package(
            CATKIN_DEPENDS other_catkin unknown_package
            )
            """,
        checks=cc.exports)
        self.assertEqual([ "UNKNOWN_PACKAGE" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(other_catkin REQUIRED)
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
            find_package(other_catkin REQUIRED)
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
            add_library(${PROJECT_NAME} src/source.cpp)
            """,
        checks=cc.exports)
        self.assertEqual([ "MISSING_EXPORT_LIB" ], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})
            include_directories(include)
            add_library(${PROJECT_NAME} src/source.cpp)
            set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "renamed")
            """,
        checks=cc.exports)
        self.assertEqual([ "EXPORT_LIB_RENAMED" ], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})
            include_directories(include)
            add_executable(${PROJECT_NAME} src/source.cpp)
            """,
        checks=cc.exports)
        self.assertEqual([ "EXPORT_LIB_NOT_LIB" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package(INCLUDE_DIRS include)
            add_executable(test_${PROJECT_NAME} src/source.cpp)
            """,
        checks=cc.exports)
        self.assertEqual([ "MISSING_BUILD_INCLUDE" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            catkin_package(INCLUDE_DIRS include)
            include_directories(include/${PROJECT_NAME})
            add_executable(test_${PROJECT_NAME} src/source.cpp)
            """,
        checks=cc.exports)
        self.assertEqual([ "MISSING_BUILD_INCLUDE", "AMBIGUOUS_BUILD_INCLUDE" ], result)
        pkg = create_manifest("mock", build_depends=[ "first_pkg", "second_pkg" ], run_depends=[ "first_pkg", "second_pkg" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS first_pkg second_pkg)
            catkin_package(CATKIN_DEPENDS first_pkg second_pkg)
            """,
        checks=cc.exports)
        self.assertEqual([], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS first_pkg second_pkg)
            catkin_package(CATKIN_DEPENDS second_pkg first_pkg)
            """,
        checks=cc.exports)
        self.assertEqual([ "UNSORTED_LIST" ], result)


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
            find_package(message_generation REQUIRED)
            find_package(other_catkin REQUIRED)
            add_message_files(FILES message.msg)
            generate_messages(DEPENDENCIES other_catkin)
            catkin_package(CATKIN_DEPENDS message_runtime other_catkin)
            """,
        checks=cc.message_generation)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation REQUIRED)
            find_package(other_catkin REQUIRED)
            generate_messages(DEPENDENCIES other_catkin)
            add_message_files(FILES message.msg)
            catkin_package(CATKIN_DEPENDS message_runtime other_catkin)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "ORDER_VIOLATION" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation REQUIRED)
            find_package(other_catkin REQUIRED)
            catkin_package(CATKIN_DEPENDS message_runtime other_catkin)
            add_message_files(FILES message.msg)
            generate_messages(DEPENDENCIES other_catkin)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "ORDER_VIOLATION", "ORDER_VIOLATION" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(message_generation REQUIRED)
            find_package(other_catkin REQUIRED)
            add_message_files(FILES message.msg)
            generate_messages(DEPENDENCIES other_catkin)
            find_package(catkin REQUIRED)
            catkin_package(CATKIN_DEPENDS message_runtime other_catkin)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "CATKIN_ORDER_VIOLATION", "CATKIN_ORDER_VIOLATION" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation REQUIRED)
            find_package(other_catkin REQUIRED)
            add_message_files(FILES message.msg)
            generate_messages(DEPENDENCIES other_catkin)
            catkin_package(CATKIN_DEPENDS message_runtime)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "MISSING_MSG_CATKIN" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation REQUIRED)
            find_package(other_catkin REQUIRED)
            add_message_files(FILES message.msg)
            catkin_package(CATKIN_DEPENDS message_runtime other_catkin)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "MISSING_GENERATE_MSG" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation REQUIRED)
            find_package(other_catkin REQUIRED)
            generate_messages(DEPENDENCIES other_catkin)
            catkin_package(CATKIN_DEPENDS message_runtime other_catkin)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "UNUSED_GENERATE_MSG" ], result)

        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED)
            find_package(message_generation REQUIRED)
            find_package(other_catkin REQUIRED)
            add_message_files(FILES message.msg)
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
            add_message_files(FILES message.msg)
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
            find_package(other_catkin REQUIRED)
            add_message_files(FILES message.msg)
            generate_messages(DEPENDENCIES other_catkin)
            catkin_package(CATKIN_DEPENDS message_runtime other_catkin)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "UNCONFIGURED_BUILD_DEPEND" ], result)

        pkg = create_manifest("mock", build_depends=[ "message_generation" ], run_depends=[ "message_runtime" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS message_generation)
            add_message_files(FILES message.msg)
            generate_messages(DEPENDENCIES other_catkin)
            catkin_package(CATKIN_DEPENDS message_runtime other_catkin)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "MISSING_DEPEND", "UNCONFIGURED_MSG_DEPEND", "MISSING_MSG_DEPEND", "MISSING_MSG_DEPEND" ], result)
        pkg = create_manifest("mock", build_depends=[ "message_generation", "first_pkg", "second_pkg" ], run_depends=[ "message_runtime", "first_pkg", "second_pkg" ])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS first_pkg message_generation second_pkg)
            add_message_files(FILES message1.msg message2.msg)
            generate_messages(DEPENDENCIES first_pkg second_pkg)
            catkin_package(CATKIN_DEPENDS first_pkg message_runtime second_pkg)
            """,
        checks=cc.message_generation)
        self.assertEqual([], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS first_pkg message_generation second_pkg)
            add_message_files(FILES message1.msg message2.msg)
            generate_messages(DEPENDENCIES second_pkg first_pkg)
            catkin_package(CATKIN_DEPENDS first_pkg message_runtime second_pkg)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "UNSORTED_LIST" ], result)
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS first_pkg message_generation second_pkg)
            add_message_files(FILES message2.msg message1.msg)
            generate_messages(DEPENDENCIES first_pkg second_pkg)
            catkin_package(CATKIN_DEPENDS first_pkg message_runtime second_pkg)
            """,
        checks=cc.message_generation)
        self.assertEqual([ "UNSORTED_LIST" ], result)

    def test_format2_message_exports(self):
        env = create_env()
        pkg = create_manifest2("mock", build_depends=["message_generation"], exec_depends=["message_runtime"])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS message_generation)
            catkin_package(CATKIN_DEPENDS message_runtime)
            """,
        checks=cc.exports)
        self.assertEqual([], result)

        pkg = create_manifest2("mock", build_depends=["message_generation"], build_export_depends=["message_runtime"])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS message_generation)
            catkin_package(CATKIN_DEPENDS message_runtime)
            """,
        checks=cc.exports)
        self.assertEqual([], result)

        pkg = create_manifest2("mock", build_depends=["message_generation"])
        result = mock_lint(env, pkg,
            """
            project(mock)
            find_package(catkin REQUIRED COMPONENTS message_generation)
            catkin_package(CATKIN_DEPENDS message_runtime)
            """,
        checks=cc.exports)
        self.assertEqual(["MISSING_DEPEND"], result)


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
