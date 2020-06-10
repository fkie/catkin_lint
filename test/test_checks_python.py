# coding=utf-8
#
# catkin_lint
# Copyright (c) 2013-2020 Fraunhofer FKIE
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
#  * Neither the name of the Fraunhofer organization nor the names of its
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
import os
import catkin_lint.checks.python as cc
from .helper import create_env, create_manifest, mock_lint, patch, posix_and_nt


class ChecksPythonTest(unittest.TestCase):

    @patch("os.path.isfile", lambda x: False)
    def test_setup_without_setup_py(self):
        """Test catkin_python_setup() call without setup.py"""
        env = create_env()
        pkg = create_manifest("mock")

        result = mock_lint(env, pkg, "", checks=cc.setup)
        self.assertEqual([], result)

        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_python_setup()", checks=cc.setup)
        self.assertEqual(["MISSING_FILE"], result)

    @posix_and_nt
    @patch("os.path.isfile", lambda x: x in [os.path.normpath("/package-path/mock/setup.py"), os.path.normpath("/package-path/catkin/setup.py")])
    def test_setup_with_setup_py(self):
        """Test proper placement and handling of catkin_python_setup()"""
        env = create_env()
        pkg = create_manifest("mock")

        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_python_setup()", checks=cc.setup)
        self.assertEqual([], result)

        result = mock_lint(env, pkg, "project(mock) catkin_python_setup()", checks=cc.setup)
        self.assertEqual(["CATKIN_ORDER_VIOLATION"], result)

        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) generate_messages() catkin_python_setup()", checks=cc.setup)
        self.assertEqual(["ORDER_VIOLATION"], result)

        result = mock_lint(env, pkg, "project(mock)", checks=cc.setup)
        self.assertEqual(["MISSING_PYTHON_SETUP"], result)

        pkg = create_manifest("catkin")
        result = mock_lint(env, pkg, "project(catkin) catkin_python_setup()", checks=cc.setup)
        self.assertEqual([], result)
