# coding=utf-8
#
# catkin_lint
# Copyright 2013-2022 Fraunhofer FKIE
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
# IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import unittest
import catkin_lint.checks.cuda as cc
from .helper import create_env, create_manifest, mock_lint, patch, posix_and_nt

import os


class ChecksCudaTest(unittest.TestCase):

    @posix_and_nt
    @patch("catkin_lint.checks.build.os.path.isfile", lambda x: x in [os.path.normpath("/package-path/mock/src/a.cpp"), os.path.normpath("/package-path/mock/src/b.cpp")])
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
