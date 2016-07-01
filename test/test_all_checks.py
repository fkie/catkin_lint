import unittest
from .helper import create_env, create_manifest, mock_lint

import sys
sys.stderr = sys.stdout
import os

try:
    from mock import patch
except ImportError:
    from unittest.mock import patch
from tempfile import mkdtemp
import shutil
import argparse
try:
    from StringIO import StringIO
except ImportError:
    from io import StringIO
from catkin_lint.main import prepare_arguments, run_linter

class AllChecksTest(unittest.TestCase):

    @patch("os.path.isfile", lambda x: x == os.path.normpath("/mock-path/src/source.cpp"))
    def test_project(self):
        env = create_env(catkin_pkgs=[ "catkin", "foo", "foo_msgs" ])
        pkg = create_manifest("mock", description="Cool Worf", build_depends=[ "foo", "foo_msgs" ], run_depends=[ "foo_msgs" ])
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


class CatkinInvokationTest(unittest.TestCase):
    
    def fake_package(self, name, depends, wsdir):
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
                'project(%s)\n' % name +
                'find_package(catkin REQUIRED ' +
                (('COMPONENTS ' + ' '.join(dep for dep in depends)) if depends else '') + ')\n'
                'catkin_package()'
            )

    def run_catkin_lint(self, *argv):
        parser = prepare_arguments(argparse.ArgumentParser())
        args = parser.parse_args(argv)
        stdout = StringIO()
        with patch("sys.stdout", stdout):
            with patch("sys.stderr", stdout):
                returncode =  run_linter(args)
        return returncode, stdout.getvalue()
     
    def setUp(self):
        self.old_environ = os.environ
        self.upstream_ws = mkdtemp()
        self.fake_package("catkin", [], wsdir=self.upstream_ws)
        self.fake_package("gamma", ["missing"], wsdir=self.upstream_ws)
        self.wsdir = mkdtemp()
        self.fake_package("alpha", ["beta"], wsdir=self.wsdir)
        self.fake_package("beta", [], wsdir=self.wsdir)
        self.fake_package("gamma", [], wsdir=self.wsdir)
        self.fake_package("broken", ["missing"], wsdir=self.wsdir)
        with open(os.path.join(self.wsdir, "src", "broken", "CATKIN_IGNORE"), "w"):
            pass
        self.fake_package(".broken", ["missing"], wsdir=self.wsdir)
        os.environ = {
            "PATH": "/usr/bin:/bin",
            "ROS_DISTRO": "indigo",
            "HOME": self.wsdir,
            "ROS_PACKAGE_PATH": self.upstream_ws,
        }
    
    def tearDown(self):
        shutil.rmtree(self.wsdir, ignore_errors=True)
        shutil.rmtree(self.upstream_ws, ignore_errors=True)
        os.environ = self.old_environ

    def runTest(self):
        self.assertEqual(
            self.run_catkin_lint(),
            (0, "catkin_lint: no path given and no package.xml in current directory\n")
        )
        pwd = os.getcwd()
        os.chdir(os.path.join(self.wsdir, "src", "beta"))
        self.assertEqual(
            self.run_catkin_lint(),
            (0, "catkin_lint: checked 1 packages and found 0 problems\n")
        )
        os.chdir(pwd)

        self.assertEqual(
            self.run_catkin_lint(self.wsdir),
            (0, "catkin_lint: checked 3 packages and found 0 problems\n")
        )
        self.assertEqual(
            self.run_catkin_lint(self.wsdir, "--disable-cache"),
            (0, "catkin_lint: checked 3 packages and found 0 problems\n")
        )
        self.assertEqual(
            self.run_catkin_lint(self.wsdir, "-c", "catkin_lint.checks.all"),
            (0, "catkin_lint: checked 3 packages and found 0 problems\n")
        )
        self.assertEqual(
            self.run_catkin_lint(self.wsdir, "-c", "does.not.exist"),
            (1, "catkin_lint: cannot import 'does.not.exist': No module named does.not\n")
        )
        self.assertEqual(
            self.run_catkin_lint("--pkg", "alpha"),
            (1, "catkin_lint: no such package: alpha\ncatkin_lint: no packages to check\n")
        )
        self.assertEqual(
            self.run_catkin_lint("--package-path", self.wsdir, "--pkg", "alpha", "--pkg", "beta"),
            (0, "catkin_lint: checked 2 packages and found 0 problems\n")
        )
        os.environ["ROS_PACKAGE_PATH"] = os.pathsep.join(os.path.join([self.wsdir, "src", self.upstream_ws]))
        self.assertEqual(
            self.run_catkin_lint("--pkg", "alpha", "--pkg", "beta"),
            (0, "catkin_lint: checked 2 packages and found 0 problems\n")
        )
        os.environ["ROS_PACKAGE_PATH"] = self.upstream_ws

        bad_path = os.path.join(self.wsdir, "src", "does_not_exist")
        self.assertEqual(
            self.run_catkin_lint(os.path.join(self.wsdir, bad_path)),
            (1, "catkin_lint: not a directory: %s\ncatkin_lint: no packages to check\n" % bad_path)
        )

        self.assertEqual(
            self.run_catkin_lint("--package-path", os.pathsep.join([self.wsdir, self.upstream_ws]), "--pkg", "gamma"),
            (0, "catkin_lint: checked 1 packages and found 0 problems\n")
        )
        exitcode, stdout = self.run_catkin_lint("--package-path", os.pathsep.join([self.upstream_ws, self.wsdir]), "--pkg", "gamma")
        self.assertEqual(exitcode, 1)
        
        exitcode, stdout = self.run_catkin_lint("--list-check-ids")
        self.assertEqual(exitcode, 0)
        self.assertIn("\nproject_name\n", stdout)

        exitcode, stdout = self.run_catkin_lint("--dump-cache")
        self.assertEqual(exitcode, 0)
        self.assertIn("alpha", stdout)
        self.assertIn("beta", stdout)
        self.assertIn(self.wsdir, stdout)

        self.assertEqual(
            self.run_catkin_lint("--clear-cache", "--dump-cache"),
            (0, "Cache version is 1\nCached local paths: 0\n")
        )

        del os.environ["ROS_DISTRO"]
        self.assertEqual(
            self.run_catkin_lint(os.path.join(self.wsdir, "src", "alpha")),
            (0, "catkin_lint: neither ROS_DISTRO environment variable nor --rosdistro option set\n"
                "catkin_lint: unknown dependencies will be ignored\n"
                "catkin_lint: checked 1 packages and found 0 problems\n")
        )
        self.assertEqual(
            self.run_catkin_lint(os.path.join(self.wsdir, "src", "alpha"), "--rosdistro", "indigo"),
            (1, "alpha: error: unknown build_export_depend 'beta'\n"
                "alpha: error: unknown exec_depend 'beta'\n"
                "alpha: error: unknown build_depend 'beta'\n"
                "alpha: CMakeLists.txt(2): error: unknown package 'beta'\n"
                "catkin_lint: checked 1 packages and found 4 problems\n")
        )
        self.assertEqual(
            self.run_catkin_lint(os.path.join(self.wsdir, "src", "alpha"), "--rosdistro", "indigo", "--ignore", "unknown_depend"),
            (1, "alpha: CMakeLists.txt(2): error: unknown package 'beta'\n"
                "catkin_lint: checked 1 packages and found 1 problems\n"
                "catkin_lint: 3 messages have been ignored explicitly\n")
        )
        self.assertEqual(
            self.run_catkin_lint(self.wsdir, "--rosdistro", "indigo"),
            (0, "catkin_lint: checked 3 packages and found 0 problems\n")
        )
