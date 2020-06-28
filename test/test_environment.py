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
from tempfile import mkdtemp
from shutil import rmtree
from time import time
from catkin_pkg.package import Package

from .helper import patch, requires_module
import catkin_lint.environment as environment


class DummyDist(object):
    def get_release_package_xml(self, name):
        return '''<package format="2">
            <name>%s</name>
            <version>0.0.0</version>
            <description>Mock package</description>
            <maintainer email="mock@example.com">Mister Mock</maintainer>
            <license>none</license>
        </package>''' % name


def get_dummy_index_url():
    return "http://127.0.0.1:9"


def get_dummy_index(url):
    return None


def get_dummy_cached_distribution(index, dist_name, cache=None, allow_lazy_load=False):
    return DummyDist()


class EnvironmentTest(unittest.TestCase):

    def fake_package(self, name, srcdir):
        pkgdir = os.path.join(srcdir, name)
        os.makedirs(pkgdir)
        with open(os.path.join(pkgdir, "package.xml"), "w") as f:
            f.write(
                '<package format="2"><name>%s</name><version>0.0.0</version>'
                '<description>Mock package</description>'
                '<maintainer email="mock@example.com">Mister Mock</maintainer>'
                '<license>none</license>'
                '</package>' % name
            )

    def setUp(self):
        self.old_environ = os.environ
        os.environ["ROS_DISTRO"] = "dummy"

        self.cachedir = mkdtemp()
        environment._cache_dir = self.cachedir
        environment._clear_cache()

        self.ws_srcdir = mkdtemp()
        self.fake_package("alpha", self.ws_srcdir)
        self.fake_package("beta", self.ws_srcdir)
        self.fake_package("gamma", self.ws_srcdir)

    def tearDown(self):
        os.environ = self.old_environ
        rmtree(self.ws_srcdir, ignore_errors=True)
        rmtree(self.cachedir, ignore_errors=True)

    def test_invalid_cache(self):
        """Test cache versioning"""
        environment._cache = environment.Cache()
        environment._cache.packages = {"bogus": True}
        environment._cache.version = environment.CACHE_VERSION - 1
        environment._store_cache()
        environment._cache = None
        environment._load_cache()
        self.assertNotIn("bogus", environment._cache.packages)
        self.assertEqual(environment._cache.version, environment.CACHE_VERSION)

    def test_local_packages(self):
        """Test catkin environment for local packages"""
        env = environment.CatkinEnvironment(use_rosdep=False, use_rosdistro=False, quiet=True)
        self.assertRaises(KeyError, env.get_manifest, "alpha")
        env.add_path(self.ws_srcdir)
        self.assertIsInstance(env.get_manifest("alpha"), Package)
        rmtree(os.path.join(self.ws_srcdir, "alpha"))
        self.assertRaises(KeyError, env.get_manifest, "alpha")

    @requires_module("rosdistro")
    @patch("rosdistro.get_index_url", get_dummy_index_url)
    @patch("rosdistro.get_index", get_dummy_index)
    @patch("rosdistro.get_cached_distribution", get_dummy_cached_distribution)
    def test_rostdistro_packages(self):
        """Test catkin environment for rosdistro packages"""
        now = time()
        old = now - environment.DOWNLOAD_CACHE_EXPIRY // 2
        expired = now - 2 * environment.DOWNLOAD_CACHE_EXPIRY
        env = environment.CatkinEnvironment(use_rosdep=False, use_rosdistro=True, quiet=True)
        self.assertIsInstance(env.get_manifest("foo"), Package)
        self.assertGreaterEqual(environment._cache.packages["dummy"]["foo"].timestamp, now)
        environment._cache.packages["dummy"]["foo"].timestamp = old
        self.assertIsInstance(env.get_manifest("foo"), Package)
        self.assertEqual(environment._cache.packages["dummy"]["foo"].timestamp, old)
        environment._cache.packages["dummy"]["foo"].timestamp = expired
        self.assertIsInstance(env.get_manifest("foo"), Package)
        self.assertGreaterEqual(environment._cache.packages["dummy"]["foo"].timestamp, now)
