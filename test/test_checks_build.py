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

    @patch("os.path.isdir", lambda x: True)
    def test_includes_with_existing_dir(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "include_directories(include)", checks=cc.includes)
        self.assertEqual([], result)


    @patch("os.path.isdir", lambda x: False)
    def test_includes_without_existing_dir(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "find_package(catkin REQUIRED) include_directories(${catkin_INCLUDE_DIRS})", checks=cc.includes)
        self.assertEqual([], result)

        result = mock_lint(env, pkg, "include_directories(include)", checks=cc.includes)
        self.assertEqual([ "MISSING_BUILD_INCLUDE_PATH" ], result)


    def test_link_directories(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "link_directories(in_package)", checks=cc.link_directories)
        self.assertEqual([ "LINK_DIRECTORY" ], result)
        result = mock_lint(env, pkg, "link_directories(/not/in/package)", checks=cc.link_directories)
        self.assertEqual([ "EXTERNAL_LINK_DIRECTORY" ], result)
