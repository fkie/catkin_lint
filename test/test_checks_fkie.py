import unittest
import catkin_lint.checks.fkie as cc
from .helper import create_env, create_manifest, mock_lint

import sys
sys.stderr = sys.stdout


class ChecksFkieTest(unittest.TestCase):

    def test_fkie_find_package(self):
        env = create_env()
        pkg = create_manifest("mock", build_depends=[ "other_catkin"] )

        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) find_package(other_catkin)", checks=cc.rosbuild_compat)
        self.assertEqual([], result)

        result = mock_lint(env, pkg, "project(mock) fkie_find_package(catkin REQUIRED) find_package(other_catkin)", checks=cc.rosbuild_compat)
        self.assertEqual([ "DEPRECATED_ROSBUILD" ], result)

        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) find_package(other_catkin) fkie_find_package(other_catkin)", checks=cc.rosbuild_compat)
        self.assertEqual([ "DEPRECATED_ROSBUILD", "DUPLICATE_FIND" ], result)

        result = mock_lint(env, pkg, "fkie_find_package(catkin REQUIRED) find_package(other_catkin)", checks=cc.rosbuild_compat)
        self.assertEqual([ "DEPRECATED_ROSBUILD", "ORDER_VIOLATION", "ORDER_VIOLATION" ], result)
