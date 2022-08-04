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

import os
import sys
import unittest
from catkin_pkg.package import Package, Dependency

import catkin_lint.checks.manifest as cc

from .helper import create_env, create_manifest, create_manifest2, mock_lint, mock_open, patch, posix_and_nt


class ChecksManifestTest(unittest.TestCase):

    def test_depends(self):
        """Test dependency checks for package.xml"""
        env = create_env()

        pkg = create_manifest("mock", build_depends=["other_catkin"])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual([], result)

        pkg = create_manifest("mock", buildtool_depends=["invalid"])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual(["UNKNOWN_PACKAGE"], result)

        pkg = create_manifest("mock", build_depends=["invalid"])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual(["UNKNOWN_PACKAGE"], result)

        pkg = create_manifest("mock", run_depends=["invalid"])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual(["UNKNOWN_PACKAGE"], result)

        pkg = create_manifest("mock", test_depends=["invalid"])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual(["UNKNOWN_PACKAGE"], result)

        pkg = create_manifest2("mock", build_export_depends=["invalid"])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual(["UNKNOWN_PACKAGE"], result)

        pkg = create_manifest2("mock", buildtool_export_depends=["invalid"])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual(["UNKNOWN_PACKAGE"], result)

        pkg = create_manifest2("mock", exec_depends=["invalid"])
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual(["UNKNOWN_PACKAGE"], result)

        pkg = create_manifest("mock", run_depends=["other_catkin"], meta=True)
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual([], result)

        pkg = create_manifest("mock", build_depends=["other_catkin"], meta=True)
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual(["INVALID_META_DEPEND"], result)

        pkg = create_manifest("mock", test_depends=["other_catkin"], meta=True)
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual(["INVALID_META_DEPEND"], result)

    def test_catkin_build(self):
        """Test catkin build system checks"""
        env = create_env()

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.catkin_build)
        self.assertEqual([], result)

        pkg = create_manifest("mock", buildtool_depends=[])
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.catkin_build)
        self.assertEqual(["MISSING_DEPEND"], result)

        pkg = create_manifest("mock", buildtool_depends=[], build_depends=["catkin"])
        result = mock_lint(env, pkg, "", checks=cc.catkin_build)
        self.assertEqual(["WRONG_DEPEND"], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "", checks=cc.catkin_build)
        self.assertEqual(["UNUSED_DEPEND"], result)

        pkg = create_manifest("mock", meta=True)
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.catkin_build)
        self.assertEqual(["WRONG_CATKIN_PACKAGE"], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_metapackage()", checks=cc.catkin_build)
        self.assertEqual(["WRONG_CATKIN_METAPACKAGE"], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) catkin_package()", checks=cc.catkin_build)
        self.assertEqual(["CATKIN_ORDER_VIOLATION", "MISSING_FIND"], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) generate_messages()", checks=cc.catkin_build)
        self.assertEqual(["MISSING_FIND", "MISSING_CMD"], result)

        pkg = create_manifest("mock", meta=True)
        result = mock_lint(env, pkg, "project(mock) catkin_metapackage()", checks=cc.catkin_build)
        self.assertEqual(["CATKIN_ORDER_VIOLATION", "MISSING_FIND"], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED)", checks=cc.catkin_build)
        self.assertEqual(["MISSING_CMD"], result)

        pkg = create_manifest("mock", meta=True)
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED)", checks=cc.catkin_build)
        self.assertEqual(["MISSING_CMD"], result)

        pkg = create_manifest("catkin")
        result = mock_lint(env, pkg, "project(catkin) catkin_package()")
        self.assertEqual([], result)

    @posix_and_nt
    @patch("os.walk", lambda x, topdown: iter([(os.path.normpath("/package-path/mock"), [], ["mock.launch"])]))
    @patch("catkin_lint.checks.build.os.path.isfile", lambda x: "missing" not in x)
    def test_launch_depends(self):
        """Test check for package dependencies which are used in launch files"""
        env = create_env()
        open_func = "builtins.open" if sys.version_info[0] >= 3 else "__builtin__.open"
        pkg = create_manifest("mock")
        with patch(open_func, mock_open(read_data=b'<launch></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual([], result)
        with patch(open_func, mock_open(read_data=b'<?xml version="1.0"?>\n<launch></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual([], result)
        with patch(open_func, mock_open(read_data=b'<?xml version="1.0" encoding="UTF-8"?>\n<launch></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual([], result)
        with patch(open_func, mock_open(read_data=b'<?xml version="1.0" encoding="UTF-8"?>\n<launch invalid-utf8="\xFF"></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual(["PARSE_ERROR"], result)
        with patch(open_func, mock_open(read_data=b'<launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual(["PARSE_ERROR"], result)
        with patch(open_func, mock_open(read_data=b'<launch><node pkg="$(arg weird)"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual([], result)
        with patch(open_func, mock_open(read_data=b'<launch><node pkg="other_system"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual([], result)
        with patch(open_func, mock_open(read_data=b'<launch><node pkg="other_catkin"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual(["LAUNCH_DEPEND"], result)
        with patch(open_func, mock_open(read_data=b'<launch><node pkg="other_catkin"/><node pkg="other_catkin"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual(["LAUNCH_DEPEND", "LAUNCH_DEPEND"], result)
        with patch(open_func, mock_open(read_data=b'<launch><!-- catkin_lint: ignore launch_depend --><node pkg="other_catkin"/><node pkg="other_catkin"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual([], result)
        with patch(open_func, mock_open(read_data=b'<launch><!-- catkin_lint: ignore launch_depend --><node pkg="other_catkin"/><!-- catkin_lint: report launch_depend --><node pkg="other_catkin"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual(["LAUNCH_DEPEND"], result)
        with patch(open_func, mock_open(read_data=b'<launch><!-- --><node pkg="other_catkin"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual(["LAUNCH_DEPEND"], result)
        with patch(open_func, mock_open(read_data=b'<launch><!-- catkin_lint: ignore_once launch_depend --><node pkg="other_catkin"/><node pkg="other_catkin"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual(["LAUNCH_DEPEND"], result)
        with patch(open_func, mock_open(read_data=b'<launch><include file="$(find other_catkin)/path/to/other.launch"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual(["LAUNCH_DEPEND"], result)
        with patch(open_func, mock_open(read_data=b'<launch><test test-name="mytest" pkg="other_catkin" type="testnode"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package() add_rostest(${PROJECT_NAME}.launch)", checks=cc.launch_depends)
            self.assertEqual(["LAUNCH_DEPEND"], result)
        with patch(open_func, mock_open(read_data=b'<launch></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package() add_rostest(missing.launch)", checks=cc.launch_depends)
            self.assertEqual(["MISSING_FILE"], result)

    def test_export_targets(self):
        """Test check for valid exported targets"""
        env = create_env()

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package(EXPORTED_TARGETS valid) add_custom_target(valid)", checks=cc.export_targets)
        self.assertEqual([], result)

        pkg = create_manifest("mock")
        result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package(EXPORTED_TARGETS invalid)", checks=cc.export_targets)
        self.assertEqual(["UNDEFINED_TARGET"], result)

    def test_package_description(self):
        """Test check for descriptive package descriptions"""
        env = create_env()
        pkg = create_manifest("mock", description="Cool Worf")
        result = mock_lint(env, pkg, "", checks=cc.package_description)
        self.assertEqual([], result)
        pkg = create_manifest("mock", description="The mock package provides a Cool Worf")
        result = mock_lint(env, pkg, "", checks=cc.package_description)
        self.assertEqual(["DESCRIPTION_BOILERPLATE"], result)
        pkg = create_manifest("mock", description="This mock package is a package for ROS nodes")
        result = mock_lint(env, pkg, "", checks=cc.package_description)
        self.assertEqual(["DESCRIPTION_MEANINGLESS"], result)
        pkg = create_manifest("mock", description="Mock Cool Worf")
        result = mock_lint(env, pkg, "", checks=cc.package_description)
        self.assertEqual([], result)

    @unittest.skipUnless(hasattr(Package, "evaluate_conditions"), "catkin_pkg module is too old")
    def test_evaluate_conditions(self):
        """Test if dependency conditions are properly evaluated"""

        env = create_env(system_pkgs=['python-yaml'])
        pkg = Package(
            name="mock",
            package_format=3,
            exec_depends=[Dependency('python-yaml', condition='$ROS_PYTHON_VERSION == 2'),
                          Dependency('python3-yaml', condition='$ROS_PYTHON_VERSION == 3')],
        )
        pkg.evaluate_conditions({'ROS_PYTHON_VERSION': 2})
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual([], result)

        pkg.evaluate_conditions({'ROS_PYTHON_VERSION': 3})
        result = mock_lint(env, pkg, "", checks=cc.depends)
        self.assertEqual(["UNKNOWN_PACKAGE"], result)
