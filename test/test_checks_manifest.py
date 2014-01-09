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
