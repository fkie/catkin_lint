import unittest
import catkin_lint.checks.cuda as cc
from .helper import create_env, create_manifest, mock_lint, patch, posix_and_nt

import os


class ChecksCudaTest(unittest.TestCase):

    @posix_and_nt
    @patch("os.path.isfile", lambda x: x in [os.path.normpath("/package-path/mock/src/a.cpp"), os.path.normpath("/package-path/mock/src/b.cpp")])
    def test_targets(self):
        """Test CUDA checks"""
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "cuda_add_executable(target src/a.cpp src/b.cpp) cuda_add_library(target_lib src/a.cpp src/b.cpp)", checks=cc.targets)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "cuda_add_executable(target ${CMAKE_CURRENT_SOURCE_DIR}/src/a.cpp) cuda_add_library(target_lib ${CMAKE_CURRENT_SOURCE_DIR}/src/a.cpp)", checks=cc.targets)
        self.assertEqual([], result)
        result = mock_lint(env, pkg, "cuda_add_executable(target} src/missing.cpp)", checks=cc.targets)
        self.assertEqual(["MISSING_FILE"], result)
        result = mock_lint(env, pkg, "cuda_add_library(target src/missing.cpp)", checks=cc.targets)
        self.assertEqual(["MISSING_FILE"], result)
        result = mock_lint(env, pkg, "cuda_add_executable(target src/b.cpp src/a.cpp)", checks=cc.targets)
        self.assertEqual(["UNSORTED_LIST"], result)
        result = mock_lint(env, pkg, "cuda_add_library(target} src/b.cpp src/a.cpp)", checks=cc.targets)
        self.assertEqual(["UNSORTED_LIST"], result)
