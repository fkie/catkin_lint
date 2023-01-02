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
import gc
import sys
from time import time
try:
    import cPickle as pickle
except ImportError:
    import pickle
from catkin_pkg.package import parse_package, PACKAGE_MANIFEST_FILENAME
from catkin_pkg.packages import find_package_paths
from .ros import get_rosdep, get_rosdistro
from .util import iteritems, is_active, write_atomic

DOWNLOAD_CACHE_EXPIRY = 2592000  # 30 days
CACHE_VERSION = 1


class Cache(object):
    def __init__(self):
        self.version = CACHE_VERSION
        self.local_paths = {}
        self.packages = {}


class CacheItem(object):
    def __init__(self, data, timestamp):
        self.data = data
        self.timestamp = timestamp


class PackageCacheData(object):
    def __init__(self, path, manifest):
        self.path = path  # if path is None, we know the package but it is not installed
        self.manifest = manifest


def find_packages(basepath, use_cache=True):
    global _cache
    if use_cache:
        _load_cache()
        distro_id = os.environ.get("ROS_DISTRO", None)
    packages = {}
    package_paths = find_package_paths(basepath)
    cache_updated = False
    for path in package_paths:
        pkg_dir = os.path.realpath(os.path.join(basepath, path))
        if use_cache:
            last_modified = os.path.getmtime(os.path.join(pkg_dir, PACKAGE_MANIFEST_FILENAME))
            path_ts = _cache.local_paths[pkg_dir].timestamp if pkg_dir in _cache.local_paths else 0
            if last_modified > path_ts:
                manifest = parse_package(pkg_dir)
                _cache.local_paths[pkg_dir] = CacheItem(manifest, last_modified)
                cache_updated = True
            else:
                manifest = _cache.local_paths[pkg_dir].data
            if distro_id not in _cache.packages:
                _cache.packages[distro_id] = {}
            manifest_ts = _cache.packages[distro_id][manifest.name].timestamp if manifest.name in _cache.packages[distro_id] else 0
            if last_modified > manifest_ts:
                _cache.packages[distro_id][manifest.name] = CacheItem(PackageCacheData(path=pkg_dir, manifest=manifest), last_modified)
                cache_updated = True
        else:
            manifest = parse_package(pkg_dir)
        packages[path] = manifest
    if cache_updated:
        _store_cache()
    for package in packages.values():
        if hasattr(package, "evaluate_conditions"):
            package.evaluate_conditions(os.environ)
    return packages


def is_catkin_package(manifest):
    if manifest is None:
        return False
    for e in manifest.exports:
        if is_active(e) and e.tagname == "build_type" and e.content != "catkin":
            return False
    return True


class PackageType:
    UNKNOWN = 0
    CATKIN = 1
    OTHER = 2
    ANY = 3  # CATKIN | OTHER
    INDETERMINATE = 32768


class CatkinEnvironment(object):
    def __init__(self, os_env=None, use_rosdep=True, use_rosdistro=True, use_cache=True, quiet=False):
        self.package_path_order = []
        self.searched_paths = {}
        self.known_catkin_pkgs = set()
        self.known_other_pkgs = set()
        self.knows_everything = use_rosdep and use_rosdistro
        self.os_env = os_env
        self.use_cache = use_cache
        self.use_rosdistro = use_rosdistro
        self.rosdistro = None
        self.rosdep = None
        self.quiet = quiet
        if use_rosdep:
            try:
                gc.disable()
                self.rosdep = get_rosdep(quiet=self.quiet)
                if len(self.rosdep.view.keys()) == 0:
                    raise Exception("empty view, please call 'rosdep update' first")
            except Exception as err:
                if not self.quiet:
                    sys.stderr.write("catkin_lint: cannot load rosdep database: %s\n" % str(err))
                    sys.stderr.write("catkin_lint: unknown dependencies will be ignored\n")
                self.knows_everything = False
            finally:
                gc.enable()

    def add_path(self, path):
        if not os.path.isdir(path):
            return []
        realpath = os.path.realpath(path)
        if realpath in self.searched_paths:
            return self.searched_paths[realpath]
        self.package_path_order.append(realpath)
        pkgs = find_packages(path, use_cache=self.use_cache)
        found = []
        for p, m in iteritems(pkgs):
            if is_catkin_package(m):
                self.known_catkin_pkgs.add(m.name)
                pm = (os.path.join(path, p), m)
                found.append(pm)
            else:
                self.known_other_pkgs.add(m.name)
        self.searched_paths[realpath] = found
        return found

    def find_local_pkg(self, name):
        for path in self.package_path_order:
            for p, m in self.searched_paths[path]:
                if m.name == name:
                    return p, m
        raise KeyError()

    @property
    def ok(self):
        if not self.quiet:  # pragma: no cover
            sys.stderr.write("catkin_lint: deprecated access to env.ok, use env.get_package_type() instead\n")
        return self.knows_everything

    def is_catkin_pkg(self, name):
        if not self.quiet:  # pragma: no cover
            sys.stderr.write("catkin_lint: deprecated call to env.is_catkin_pkg(), use env.get_package_type() instead\n")
        return self.get_package_type(name) == PackageType.CATKIN

    def is_known_pkg(self, name):
        if not self.quiet:  # pragma: no cover
            sys.stderr.write("catkin_lint: deprecated call to env.is_known_pkg(), use env.get_package_type() instead\n")
        return self.get_package_type(name) & PackageType.ANY

    def get_package_type(self, name):
        if name in self.known_catkin_pkgs:
            return PackageType.CATKIN
        if name in self.known_other_pkgs:
            return PackageType.OTHER
        if self.rosdep is not None:
            if self.rosdep.has_key(name):  # noqa
                try:
                    if self.rosdep.is_ros(name):
                        manifest = self.get_manifest(name)
                        if manifest is not None and is_catkin_package(manifest):
                            return PackageType.CATKIN
                    return PackageType.OTHER
                except (IOError, KeyError):
                    return PackageType.INDETERMINATE
        return PackageType.UNKNOWN if self.knows_everything else PackageType.INDETERMINATE

    def get_manifest(self, name):
        global _cache
        if self.use_cache:
            cache_updated = False
            distro_id = os.environ.get("ROS_DISTRO", None)
            _load_cache()
            if distro_id not in _cache.packages:
                _cache.packages[distro_id] = {}
            if name in _cache.packages[distro_id]:
                data = _cache.packages[distro_id][name].data
                ts = _cache.packages[distro_id][name].timestamp
                if data.path is not None and not os.path.isdir(data.path):
                    if data.path in _cache.local_paths:
                        del _cache.local_paths[data.path]
                    del _cache.packages[distro_id][name]
                    cache_updated = True
                elif self.use_rosdistro and data.path is None and ts + DOWNLOAD_CACHE_EXPIRY < time():
                    del _cache.packages[distro_id][name]
                    cache_updated = True
                else:
                    return data.manifest
            if cache_updated:
                _store_cache()
        if self.use_rosdistro:
            if self.rosdistro is None:
                self.rosdistro = get_rosdistro(quiet=self.quiet)
            if not self.rosdistro.ok():
                if not self.quiet and self.knows_everything:
                    sys.stderr.write("catkin_lint: unknown dependencies will be ignored\n")
                self.knows_everything = False
                raise KeyError()
            manifest = self.rosdistro.download_manifest(name)
            if self.use_cache:
                _cache.packages[distro_id][name] = CacheItem(PackageCacheData(path=None, manifest=manifest), time())
                _store_cache()
            return manifest
        raise KeyError()


_cache = None
try:  # pragma: no cover
    from rospkg import get_ros_home
    _cache_dir = os.path.join(get_ros_home(), "catkin_lint")
except ImportError:  # pragma: no cover
    _cache_dir = os.path.join(os.path.expanduser("~"), ".ros", "catkin_lint")


def _load_cache():
    global _cache
    global _cache_dir
    if _cache is None:
        try:
            gc.disable()
            with open(os.path.join(_cache_dir, "packages.pickle"), "rb") as f:
                _cache = pickle.loads(f.read())
                if not isinstance(_cache, Cache) or _cache.version != CACHE_VERSION:
                    raise RuntimeError()
        except Exception:
            _cache = Cache()
        finally:
            gc.enable()


def _store_cache():
    global _cache
    global _cache_dir
    try:
        os.makedirs(_cache_dir)
    except OSError:
        pass
    write_atomic(os.path.join(_cache_dir, "packages.pickle"), pickle.dumps(_cache, -1))


def _clear_cache():
    global _cache
    _cache = Cache()
    _store_cache()


def _dump_cache():
    global _cache
    _load_cache()
    sys.stdout.write("Cache version is %d\n" % _cache.version)
    sys.stdout.write("Cached local paths: %d\n" % len(_cache.local_paths))
    t0 = time()
    for p, c in iteritems(_cache.local_paths):
        sys.stdout.write("  * %s\n    => %s (%ds)\n" % (p, c.data.name, t0 - c.timestamp))
    for distro_id in _cache.packages:
        sys.stdout.write("Cached packages for distribution %s: %d\n" % (distro_id if distro_id is not None else "(None)", len(_cache.packages[distro_id])))
        for p, c in iteritems(_cache.packages[distro_id]):
            sys.stdout.write("  * %s (%s, %ds)\n" % (p, "available" if c.data.path is not None else "not found" if c.data.manifest is None else "not installed", t0 - c.timestamp))
