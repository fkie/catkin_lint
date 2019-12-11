import unittest
import os
import sys
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
    def test_launch_depends(self):
        """Test check for package dependencies which are used in launch files"""
        env = create_env()
        open_func = "builtins.open" if sys.version_info[0] >= 3 else "__builtin__.open"
        pkg = create_manifest("mock")
        with patch(open_func, mock_open(read_data='<launch></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual([], result)
        with patch(open_func, mock_open(read_data='<launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual(["PARSE_ERROR"], result)
        with patch(open_func, mock_open(read_data='<launch><node pkg="$(arg weird)"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual([], result)
        with patch(open_func, mock_open(read_data='<launch><node pkg="other_system"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual([], result)
        with patch(open_func, mock_open(read_data='<launch><node pkg="other_catkin"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual(["LAUNCH_DEPEND"], result)
        with patch(open_func, mock_open(read_data='<launch><include file="$(find other_catkin)/path/to/other.launch"/></launch>')):
            result = mock_lint(env, pkg, "project(mock) find_package(catkin REQUIRED) catkin_package()", checks=cc.launch_depends)
            self.assertEqual(["LAUNCH_DEPEND"], result)

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
