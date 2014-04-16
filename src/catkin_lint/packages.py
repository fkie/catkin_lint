#!/usr/bin/env python
"""
Copyright (c) 2013,2014 Fraunhofer FKIE

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import os
try:
    import cPickle as pickle
except ImportError:
    import pickle
from catkin_pkg.package import parse_package, PACKAGE_MANIFEST_FILENAME
from .util import write_atomic


class ManifestCacheItem(object):
    def __init__(self, manifest, last_modified):
        self.manifest = manifest
        self.last_modified = last_modified


def find_packages(basepath):
    global _manifest_cache
    if _manifest_cache is None:
        _load_manifest_cache()
    packages = {}
    package_paths = []
    for dirpath, dirnames, filenames in os.walk(basepath, followlinks=True):
        if "CATKIN_IGNORE" in filenames:
            del dirnames[:]
            continue
        elif PACKAGE_MANIFEST_FILENAME in filenames:
            package_paths.append(os.path.relpath(dirpath, basepath))
            del dirnames[:]
            continue
        for dirname in dirnames:
            if dirname.startswith('.'):
                dirnames.remove(dirname)
    cache_updated = False
    for path in package_paths:
        pkg_dir = os.path.join(basepath, path)
        last_modified = os.path.getmtime(os.path.join(pkg_dir, PACKAGE_MANIFEST_FILENAME))
        known_modified = _manifest_cache[pkg_dir].last_modified if pkg_dir in _manifest_cache else 0
        if last_modified > known_modified:
            manifest = parse_package(pkg_dir)
            _manifest_cache[pkg_dir] = ManifestCacheItem(manifest, last_modified)
            cache_updated = True
        else:
            manifest = _manifest_cache[pkg_dir].manifest
        packages[path] = manifest
    if cache_updated:
        _store_manifest_cache()
    return packages


_manifest_cache = None
try:
    from rospkg import get_ros_home
    _manifest_cache_dir = os.path.join(get_ros_home(), "catkin_lint")
except ImportError:
    _manifest_cache_dir = os.path.join(os.path.expanduser("~"), ".ros", "catkin_lint")


def _load_manifest_cache():
    global _manifest_cache
    global _manifest_cache_dir
    try:
        with open(os.path.join(_manifest_cache_dir, "packages.pickle"), "rb") as f:
            _manifest_cache = pickle.loads(f.read())
            f.close()
    except:
        _manifest_cache = {}


def _store_manifest_cache():
    global _manifest_cache
    global _manifest_cache_dir
    try:
        os.makedirs(_manifest_cache_dir)
    except:
        pass
    write_atomic(os.path.join(_manifest_cache_dir, "packages.pickle"), pickle.dumps(_manifest_cache, -1))

