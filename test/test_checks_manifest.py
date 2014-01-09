import unittest
import catkin_lint.checks.manifest as cc
from .helper import create_env, create_manifest, mock_lint

import sys
sys.stderr = sys.stdout


class ChecksManifestTest(unittest.TestCase):

    def test_package_description(self):
        env = create_env()
        pkg = create_manifest("mock", description="Cool Worf")
        result = mock_lint(env, pkg, "project(mock)", checks=cc.package_description)
        self.assertEqual([], result)
        pkg = create_manifest("mock", description="The mock package provides a Cool Worf")
        result = mock_lint(env, pkg, "project(mock)", checks=cc.package_description)
        self.assertEqual([ "DESCRIPTION_BOILERPLATE" ], result)
        pkg = create_manifest("mock", description="This mock package is a package for ROS nodes")
        result = mock_lint(env, pkg, "project(mock)", checks=cc.package_description)
        self.assertEqual([ "DESCRIPTION_MEANINGLESS" ], result)
        pkg = create_manifest("mock", description="Mock Cool Worf")
        result = mock_lint(env, pkg, "project(mock)", checks=cc.package_description)
        self.assertEqual([], result)
