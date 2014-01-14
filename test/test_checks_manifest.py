import unittest
import catkin_lint.checks.manifest as cc
from .helper import create_env, create_manifest, mock_lint

import sys
sys.stderr = sys.stdout


class ChecksManifestTest(unittest.TestCase):

    def test_depends(self):
        env = create_env()

        pkg = create_manifest("mock", build_depends=[ "other_catkin" ])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual([], result)

        pkg = create_manifest("mock", buildtool_depends=[ "invalid" ])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual([ "UNKNOWN_DEPEND" ], result)

        pkg = create_manifest("mock", build_depends=[ "invalid" ])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual([ "UNKNOWN_DEPEND" ], result)

        pkg = create_manifest("mock", run_depends=[ "invalid" ])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual([ "UNKNOWN_DEPEND" ], result)

        pkg = create_manifest("mock", test_depends=[ "invalid" ])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual([ "UNKNOWN_DEPEND" ], result)

        pkg = create_manifest("mock", run_depends=[ "other_catkin" ], meta=True)
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual([], result)

        pkg = create_manifest("mock", build_depends=[ "other_catkin" ], meta=True)
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual([ "INVALID_META_DEPEND" ], result)

        pkg = create_manifest("mock", test_depends=[ "other_catkin" ], meta=True)
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual([ "INVALID_META_DEPEND" ], result)

        pkg = create_manifest("mock", run_depends=[ "other_catkin" ], test_depends=[ "other_catkin" ])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual([ "REDUNDANT_TEST_DEPEND" ], result)


    def test_catkin_build(self):
        env = create_env()

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.catkin_build)
        self.assertEqual([], result)

        pkg = create_manifest("mock", buildtool_depends=[])
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.catkin_build)
        self.assertEqual([ "MISSING_DEPEND" ], result)

        pkg = create_manifest("mock", buildtool_depends=[], build_depends=[ "catkin" ])
        result = mock_lint(env, pkg, "", checks=cc.catkin_build)
        self.assertEqual([ "WRONG_DEPEND" ], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "", checks=cc.catkin_build)
        self.assertEqual([ "UNUSED_DEPEND" ], result)

        pkg = create_manifest("mock", meta=True)
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.catkin_build)
        self.assertEqual([ "CATKIN_PKG_VS_META" ], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_metapackage()", checks=cc.catkin_build)
        self.assertEqual([ "CATKIN_META_VS_PKG" ], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) catkin_package()", checks=cc.catkin_build)
        self.assertEqual([ "CATKIN_ORDER_VIOLATION", "MISSING_FIND" ], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) generate_messages()", checks=cc.catkin_build)
        self.assertEqual([ "MISSING_FIND", "MISSING_CMD" ], result)

        pkg = create_manifest("mock", meta=True)
        result = mock_lint(env, pkg, "project(mock) catkin_metapackage()", checks=cc.catkin_build)
        self.assertEqual([ "CATKIN_ORDER_VIOLATION", "MISSING_FIND" ], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED)", checks=cc.catkin_build)
        self.assertEqual([ "MISSING_CMD" ], result)

        pkg = create_manifest("mock", meta=True)
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED)", checks=cc.catkin_build)
        self.assertEqual([ "MISSING_CMD" ], result)

        pkg = create_manifest("catkin")
        result = mock_lint(env, pkg, "project(catkin) catkin_package()")
        self.assertEqual([], result)

    def test_export_targets(self):
        env = create_env()

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package(EXPORTED_TARGETS valid) add_custom_target(valid)", checks=cc.export_targets)
        self.assertEqual([], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package(EXPORTED_TARGETS invalid)", checks=cc.export_targets)
        self.assertEqual([ "UNDEFINED_TARGET" ], result)


    def test_package_description(self):
        env = create_env()
        pkg = create_manifest("mock", description="Cool Worf")
        result = mock_lint(env, pkg, "", checks=cc.package_description)
        self.assertEqual([], result)
        pkg = create_manifest("mock", description="The mock package provides a Cool Worf")
        result = mock_lint(env, pkg, "", checks=cc.package_description)
        self.assertEqual([ "DESCRIPTION_BOILERPLATE" ], result)
        pkg = create_manifest("mock", description="This mock package is a package for ROS nodes")
        result = mock_lint(env, pkg, "", checks=cc.package_description)
        self.assertEqual([ "DESCRIPTION_MEANINGLESS" ], result)
        pkg = create_manifest("mock", description="Mock Cool Worf")
        result = mock_lint(env, pkg, "", checks=cc.package_description)
        self.assertEqual([], result)
