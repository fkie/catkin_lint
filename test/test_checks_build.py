import unittest
import catkin_lint.checks.build as cc
from .helper import create_env, create_manifest, mock_lint

import sys
sys.stderr = sys.stdout

try:
    from mock import patch
except ImportError:
    from unittest.mock import patch

class ChecksBuildTest(unittest.TestCase):

    @patch("os.path.isdir", lambda x: x == "/mock-path/include")
    def test_includes(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "include_directories(include)", checks=cc.includes)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "find_package(catkin REQUIRED) include_directories(${catkin_INCLUDE_DIRS})", checks=cc.includes)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "include_directories(missing_include)", checks=cc.includes)
        self.assertEqual([ "MISSING_BUILD_INCLUDE_PATH" ], result)


    @patch("os.path.isdir", lambda x: x == "/mock-path/in_package")
    def test_link_directories(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "link_directories(in_package)", checks=cc.link_directories)
        self.assertEqual([ "LINK_DIRECTORY" ], result)
        result = mock_lint(env, pkg, "link_directories(/not/in/package)", checks=cc.link_directories)
        self.assertEqual([ "EXTERNAL_LINK_DIRECTORY" ], result)

    @patch("os.path.isfile", lambda x: x == "/mock-path/config.xml")
    def test_plugins(self):
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
