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
import shutil
import argparse
import os
from .helper import create_env, create_manifest, mock_lint, patch, maybe_patch, requires_module
from tempfile import mkdtemp
try:
    from StringIO import StringIO
except ImportError:
    from io import StringIO
from catkin_lint.main import prepare_arguments, run_linter
import catkin_lint.environment
import catkin_lint.ros


class AllChecksTest(unittest.TestCase):

    @patch("os.path.isfile", lambda x: x == os.path.normpath("/package-path/mock/src/source.cpp"))
    def test_project(self):
        """Test minimal catkin project for compliance"""
        env = create_env(catkin_pkgs=["catkin", "foo", "foo_msgs"])
        pkg = create_manifest("mock", description="Cool Worf", build_depends=["foo", "foo_msgs"], run_depends=["foo_msgs"])
        result = mock_lint(env, pkg,
                           """\
            project(mock)
            find_package(catkin REQUIRED COMPONENTS foo foo_msgs)
            catkin_package(CATKIN_DEPENDS foo_msgs)
            include_directories(${catkin_INCLUDE_DIRS})
            add_executable(${PROJECT_NAME}_node src/source.cpp)
            target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
            install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
            """)
        self.assertEqual([], result)

    @patch("os.path.isfile", lambda x: x == os.path.normpath("/package-path/mock/src/source.cpp"))
    def test_project_with_skip(self):
        """Test minimal catkin project with skip directive"""
        env = create_env(catkin_pkgs=["catkin", "foo", "foo_msgs"])
        pkg = create_manifest("mock", description="Cool Worf", build_depends=["foo", "foo_msgs"], run_depends=["foo_msgs"])
        result = mock_lint(env, pkg,
                           """\
            project(mock)
            find_package(catkin REQUIRED COMPONENTS foo foo_msgs)
            catkin_package(CATKIN_DEPENDS foo_msgs)
            include_directories(${catkin_INCLUDE_DIRS})
            add_executable(${PROJECT_NAME}_node src/source.cpp)
            target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
            install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
            if(CONDITION_THAT_IS_USUALLY_FALSE) #catkin_lint: skip
                find_package(catkin)
            endif()
            """)
        self.assertEqual([], result)


class DummyDist(object):
    def get_release_package_xml(self, name):
        if name == "roscpp":
            raise KeyError("Mock Error")
        if name in ["catkin", "message_runtime"]:
            return '''<package format="2">
                <name>%s</name>
                <version>0.0.0</version>
                <description>Mock package</description>
                <maintainer email="mock@example.com">Mister Mock</maintainer>
                <license>none</license>
            </package>''' % name
        if "msg" in name:
            return '''<package format="2">
                <name>%s</name>
                <version>0.0.0</version>
                <description>Mock package</description>
                <maintainer email="mock@example.com">Mister Mock</maintainer>
                <exec_depend>message_runtime</exec_depend>
                <license>none</license>
            </package>''' % name
        assert False, "should never reach this"  # pragma: no cover


def get_dummy_index_url():
    return "http://127.0.0.1:9"


def get_dummy_index(url):
    return None


def get_dummy_cached_distribution(index, dist_name, cache=None, allow_lazy_load=False):
    return DummyDist()


def raise_io_error(*args, **kwargs):
    raise IOError("mock exception")


class CatkinInvokationTest(unittest.TestCase):

    def fake_package(self, name, depends, wsdir, broken=False, uses_env=False):
        pkgdir = os.path.join(wsdir, "src", name)
        os.makedirs(pkgdir)
        with open(os.path.join(pkgdir, "package.xml"), "w") as f:
            f.write(
                '<package format="2"><name>%s</name><version>0.0.0</version>'
                '<description>Mock package</description>'
                '<maintainer email="mock@example.com">Mister Mock</maintainer>'
                '<license>none</license>' % name
            )
            if name != "catkin":
                f.write('<buildtool_depend>catkin</buildtool_depend>')
            f.write(''.join(['<depend>%s</depend>' % dep for dep in depends]))
            f.write('</package>\n')
        with open(os.path.join(pkgdir, "CMakeLists.txt"), "w") as f:
            f.write(
                'cmake_minimum_required(VERSION 3.0)\n'
                'project(%(name)s)\n'
                'find_package(catkin REQUIRED %(components)s)\n'
                'message(STATUS "%(env)s")\n'
                'catkin_package()\n' % {
                    "name": name,
                    "components": ('COMPONENTS ' + ' '.join(dep for dep in depends)) if depends else '',
                    "env": '$ENV{ROS_DISTRO}' if uses_env else ''
                }
            )
            if broken:
                f.write("catkin_package()\n")
        os.makedirs(os.path.join(pkgdir, ".git"))
        with open(os.path.join(pkgdir, ".git", "script"), "w") as f:
            f.write("Random executable file")
        os.chmod(os.path.join(pkgdir, ".git", "script"), 0o755)

    @maybe_patch("rosdistro.get_index_url", get_dummy_index_url)
    @maybe_patch("rosdistro.get_index", get_dummy_index)
    def run_catkin_lint(self, *argv, **kwargs):
        print("RUN: catkin_lint " + " ".join(argv))
        with_rosdistro = kwargs.pop("with_rosdistro", False)
        if kwargs:  # pragma: no cover
            raise SyntaxError("unknown extra arguments: %s" % ", ".join("%s=%s" % (k, v) for k, v in kwargs.items()))
        catkin_lint.environment._cache = None  # force cache reloads
        catkin_lint.ros._rosdistro_cache = {}
        parser = prepare_arguments(argparse.ArgumentParser())
        args = parser.parse_args(argv)
        stdout = StringIO()
        with patch("sys.stdout", stdout):
            with patch("sys.stderr", stdout):
                try:
                    with patch("rosdistro.get_cached_distribution", get_dummy_cached_distribution if with_rosdistro else raise_io_error):
                        returncode = run_linter(args)
                except ImportError:
                    returncode = run_linter(args)
        print(stdout.getvalue())
        return returncode, stdout.getvalue()

    def setUp(self):
        self.oldcwd = os.getcwd()
        self.old_environ = os.environ
        self.upstream_ws = mkdtemp()
        self.upstream_ws_srcdir = os.path.join(self.upstream_ws, "src")
        self.fake_package("gamma", [], wsdir=self.upstream_ws, broken=True)
        self.fake_package("missing_dep", ["missing"], wsdir=self.upstream_ws)
        self.fake_package("invalid_dep", ["boost"], wsdir=self.upstream_ws)
        self.fake_package("epsilon", [], wsdir=self.upstream_ws, uses_env=True)
        self.fake_package("delta", ["std_msgs"], wsdir=self.upstream_ws)
        self.homedir = mkdtemp()
        self.wsdir = mkdtemp()
        self.ws_srcdir = os.path.join(self.wsdir, "src")
        os.makedirs(os.path.join(self.wsdir, "src", "no_packages_here"))
        self.fake_package("alpha", ["roscpp", "beta"], wsdir=self.wsdir)
        self.fake_package("beta", [], wsdir=self.wsdir)
        self.fake_package("gamma", ["delta"], wsdir=self.wsdir)
        self.fake_package("broken", [], wsdir=self.wsdir, broken=True)
        with open(os.path.join(self.wsdir, "src", "broken", "CATKIN_IGNORE"), "w"):
            pass
        self.fake_package(".dotdir_with_broken_package", [], wsdir=self.wsdir, broken=True)
        catkin_lint.environment._cache_dir = os.path.join(self.homedir, ".ros", "catkin_lint")
        os.makedirs(catkin_lint.environment._cache_dir)
        os.environ = {
            "PATH": "/usr/bin:/bin",
            "ROS_DISTRO": "melodic",
            "HOME": self.homedir,
            "ROS_PACKAGE_PATH": self.upstream_ws_srcdir,
        }
        shutil.copytree(os.path.join(os.path.dirname(__file__), "sources.cache"), os.path.join(self.homedir, ".ros", "rosdep", "sources.cache"))
        os.chdir(self.homedir)

    def tearDown(self):
        os.chdir(self.oldcwd)
        shutil.rmtree(self.homedir, ignore_errors=True)
        shutil.rmtree(self.wsdir, ignore_errors=True)
        shutil.rmtree(self.upstream_ws, ignore_errors=True)
        os.environ = self.old_environ

    def test_cli(self):
        """Test correct behavior of CLI"""
        exitcode, stdout = self.run_catkin_lint()
        self.assertEqual(exitcode, os.EX_NOINPUT)
        self.assertIn("no path given and no package.xml in current directory", stdout)

        exitcode, stdout = self.run_catkin_lint("--help-problem")
        self.assertEqual(exitcode, 0)

        exitcode, stdout = self.run_catkin_lint("--help-problem", "does-not-exist")
        self.assertEqual(exitcode, 1)

        exitcode, stdout = self.run_catkin_lint("--help-problem", "duplicate_cmd")
        self.assertEqual(exitcode, 0)

        os.chdir(os.path.join(self.ws_srcdir, "beta"))
        exitcode, stdout = self.run_catkin_lint()
        self.assertEqual(exitcode, 0)
        self.assertIn("checked 1 packages and found 0 problems", stdout)
        with patch("catkin_lint.linter.CMakeLinter._read_file", raise_io_error):
            exitcode, stdout = self.run_catkin_lint()
        self.assertEqual(exitcode, 1)
        self.assertIn("OS error: mock exception", stdout)

        exitcode, stdout = self.run_catkin_lint(self.ws_srcdir, "--quiet")
        self.assertEqual(exitcode, 0)
        self.assertNotIn("checked 3 packages and found 0 problems", stdout)

        exitcode, stdout = self.run_catkin_lint(self.ws_srcdir, "--no-quiet")
        self.assertEqual(exitcode, 0)
        self.assertIn("checked 3 packages and found 0 problems", stdout)

        exitcode, stdout = self.run_catkin_lint(self.ws_srcdir, "--disable-cache", "--xml")
        self.assertEqual(exitcode, 0)
        self.assertIn("</catkin_lint>", stdout)
        self.assertIn("checked 3 packages and found 0 problems", stdout)

        exitcode, stdout = self.run_catkin_lint("--pkg", "gamma", "-c", "catkin_lint.checks.misc.project")
        self.assertEqual(exitcode, 0)
        self.assertIn("checked 1 packages and found 0 problems", stdout)

        exitcode, stdout = self.run_catkin_lint(self.ws_srcdir, "-c", "does.not.exist")
        self.assertEqual(exitcode, 1)
        self.assertIn("cannot import 'does.not.exist'", stdout)
        self.assertRaises(ImportError, self.run_catkin_lint, self.wsdir, "-c", "does.not.exist", "--debug")

        exitcode, stdout = self.run_catkin_lint("--pkg", "alpha")
        self.assertEqual(exitcode, 1)
        self.assertIn("no such package: alpha", stdout)

        exitcode, stdout = self.run_catkin_lint("--package-path", self.ws_srcdir, "--pkg", "alpha")
        self.assertEqual(exitcode, 0)
        self.assertIn("checked 1 packages and found 0 problems", stdout)

        exitcode, stdout = self.run_catkin_lint(os.path.join(self.ws_srcdir, "beta", "."), "-W2")
        self.assertEqual(exitcode, 0)
        self.assertIn("checked 1 packages and found 0 problems", stdout)

        os.environ["ROS_PACKAGE_PATH"] = os.pathsep.join([self.ws_srcdir, self.upstream_ws_srcdir])
        exitcode, stdout = self.run_catkin_lint("--pkg", "alpha")
        self.assertEqual(exitcode, 0)
        self.assertIn("checked 1 packages and found 0 problems", stdout)
        os.environ["ROS_PACKAGE_PATH"] = self.upstream_ws_srcdir

        bad_path = os.path.join(self.ws_srcdir, "does_not_exist")
        exitcode, stdout = self.run_catkin_lint(bad_path)
        self.assertEqual(exitcode, 1)
        self.assertIn("not a directory", stdout)
        self.assertIn("no packages to check", stdout)

        empty_path = os.path.join(self.ws_srcdir, "no_packages_here")
        exitcode, stdout = self.run_catkin_lint(empty_path)
        self.assertEqual(exitcode, 0)
        self.assertIn("no packages to check", stdout)

        exitcode, stdout = self.run_catkin_lint(self.ws_srcdir, "--skip-path", "alpha")
        self.assertEqual(exitcode, 0)
        self.assertIn("checked 2 packages and found 0 problems", stdout)

        exitcode, stdout = self.run_catkin_lint("--pkg", "epsilon")
        self.assertIn("warning: environment variables should not be used", stdout)

        exitcode, stdout = self.run_catkin_lint("--pkg", "epsilon", "--resolve-env")
        self.assertNotIn("warning: environment variables should not be used", stdout)

        broken_gamma_path = os.path.join(self.upstream_ws_srcdir, "gamma")
        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "--ignore", "duplicate_cmd")
        self.assertEqual(exitcode, 0)
        self.assertIn("messages have been ignored", stdout)

        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "--color=always")
        self.assertIn(chr(0x1b), stdout)
        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "--color=never")
        self.assertNotIn(chr(0x1b), stdout)

        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "-W2", "--ignore", "duplicate_cmd", "--show-ignored")
        self.assertEqual(exitcode, 1)
        self.assertIn("error: duplicate catkin_package()", stdout)

        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "-W2", "--notice", "duplicate_cmd")
        self.assertEqual(exitcode, 0)
        self.assertIn("notice: duplicate catkin_package()", stdout)

        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "-W2", "--warning", "duplicate_cmd")
        self.assertEqual(exitcode, 0)
        self.assertIn("warning: duplicate catkin_package()", stdout)

        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "-W0", "--warning", "duplicate_cmd")
        self.assertIn("additional warning", stdout)
        self.assertEqual(exitcode, 0)

        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "-W2", "--error", "duplicate_cmd")
        self.assertEqual(exitcode, 1)
        self.assertIn("error: duplicate catkin_package()", stdout)

        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "--strict", "--warning", "duplicate_cmd")
        self.assertEqual(exitcode, 1)

        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "--explain")
        self.assertIn("You can ignore this problem with --ignore duplicate_cmd", stdout)

        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "--json")
        self.assertIn("\"errors\":", stdout)

        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "--xml")
        self.assertIn("</catkin_lint>", stdout)

        exitcode, stdout = self.run_catkin_lint("--package-path", os.pathsep.join([self.ws_srcdir, self.upstream_ws_srcdir]), "--pkg", "gamma")
        self.assertEqual(exitcode, 0)

        exitcode, stdout = self.run_catkin_lint("--package-path", os.pathsep.join([self.upstream_ws_srcdir, self.ws_srcdir]), "--pkg", "gamma")
        self.assertEqual(exitcode, 1)

        exitcode, stdout = self.run_catkin_lint("--list-check-ids")
        self.assertEqual(exitcode, 0)
        self.assertIn("\nproject_name\n", stdout)

        exitcode, stdout = self.run_catkin_lint("--dump-cache")
        self.assertEqual(exitcode, 0)
        self.assertIn("alpha", stdout)
        self.assertIn("beta", stdout)
        self.assertIn(self.wsdir, stdout)

        exitcode, stdout = self.run_catkin_lint("--clear-cache")
        self.assertEqual(exitcode, 0)

        exitcode, stdout = self.run_catkin_lint("--dump-cache")
        self.assertEqual(exitcode, 0)
        self.assertIn("Cached local paths: 0\n", stdout)

    def test_config(self):
        """Test configuration file handling from CLI"""

        broken_gamma_path = os.path.join(self.upstream_ws_srcdir, "gamma")

        with open(os.path.join(self.homedir, ".catkin_lint"), "w") as f:
            f.write("[*]\nduplicate_cmd = ignore")
        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "-W2")
        self.assertEqual(exitcode, 0)
        self.assertIn("messages have been ignored", stdout)

        with open(os.path.join(self.homedir, ".catkin_lint"), "w") as f:
            f.write("[catkin_lint]\noutput = invalid\n\n[*]\nduplicate_cmd = ignore")
        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "-W2")
        self.assertEqual(exitcode, 1)
        self.assertIn("unknown output format", stdout)

        with open(os.path.join(self.homedir, ".catkin_lint"), "w") as f:
            f.write("[*]\nduplicate_cmd = notice")
        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "-W2")
        self.assertEqual(exitcode, 0)
        self.assertIn("notice: duplicate catkin_package()", stdout)

        with open(os.path.join(self.homedir, ".catkin_lint"), "w") as f:
            f.write("[*]\nduplicate_cmd = warning")
        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "-W2")
        self.assertEqual(exitcode, 0)
        self.assertIn("warning: duplicate catkin_package()", stdout)

        with open(os.path.join(self.homedir, ".catkin_lint"), "w") as f:
            f.write("[*]\nduplicate_cmd = error")
        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "-W2")
        self.assertEqual(exitcode, 1)
        self.assertIn("error: duplicate catkin_package()", stdout)

        with open(os.path.join(self.homedir, ".catkin_lint"), "w") as f:
            f.write("[*]\nduplicate_cmd = default")
        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "-W2")
        self.assertEqual(exitcode, 1)
        self.assertIn("error: duplicate catkin_package()", stdout)

        with open(os.path.join(self.homedir, ".catkin_lint"), "w") as f:
            f.write("[*]\nduplicate_cmd = notice\n\n[gamma]\nduplicate_cmd = warning\n")
        exitcode, stdout = self.run_catkin_lint(broken_gamma_path, "-W2")
        self.assertIn("warning: duplicate catkin_package()", stdout)

        with open(os.path.join(self.ws_srcdir, "catkin_lint_config"), "w") as f:
            f.write("[catkin_lint]\npackage_path = %s\n" % self.ws_srcdir)
        exitcode, stdout = self.run_catkin_lint("--pkg", "alpha", "--pkg", "beta", "--config", os.path.join(self.ws_srcdir, "catkin_lint_config"))
        self.assertEqual(exitcode, 0)
        self.assertIn("checked 2 packages and found 0 problems", stdout)

        exitcode, stdout = self.run_catkin_lint("--config", "no_such_file")
        self.assertEqual(exitcode, 1)
        self.assertIn("cannot read 'no_such_file'", stdout)

    @requires_module("rosdep2")
    @requires_module("rosdistro")
    def test_rosdep_and_rosdistro(self):
        """Test rosdep2 and rosdistro integration from CLI"""
        exitcode, stdout = self.run_catkin_lint("--pkg", "delta", with_rosdistro=True)
        self.assertIn("package 'std_msgs' must be in CATKIN_DEPENDS", stdout)

        exitcode, stdout = self.run_catkin_lint("--pkg", "missing_dep", "--ignore", "unknown_package", with_rosdistro=True)
        self.assertEqual(exitcode, 0)
        self.assertIn("messages have been ignored", stdout)

        exitcode, stdout = self.run_catkin_lint("--package-path", os.path.join(self.ws_srcdir, "alpha"), "--pkg", "alpha", with_rosdistro=True)
        self.assertEqual(exitcode, 1)
        self.assertIn("error: unknown package 'beta'", stdout)

        del os.environ["ROS_DISTRO"]
        exitcode, stdout = self.run_catkin_lint("--pkg", "missing_dep", with_rosdistro=True)
        self.assertEqual(exitcode, 0)

        exitcode, stdout = self.run_catkin_lint("--pkg", "invalid_dep", with_rosdistro=True)
        self.assertIn("not a catkin package", stdout)
        self.assertEqual(exitcode, 1)

        exitcode, stdout = self.run_catkin_lint("--pkg", "missing_dep", "--rosdistro", "melodic", with_rosdistro=True)
        self.assertEqual(exitcode, 1)

        exitcode, stdout = self.run_catkin_lint("--pkg", "missing_dep", "--offline", "--rosdistro", "melodic", with_rosdistro=True)
        self.assertEqual(exitcode, 0)

        exitcode, stdout = self.run_catkin_lint(self.ws_srcdir, "--rosdistro", "melodic")
        self.assertEqual(exitcode, 0)
        self.assertIn("cannot initialize rosdistro", stdout)

        exitcode, stdout = self.run_catkin_lint(self.ws_srcdir, "--rosdistro", "melodic")
        self.assertEqual(exitcode, 0)
        self.assertIn("checked 3 packages and found 0 problems", stdout)
