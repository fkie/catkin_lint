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

from catkin_lint.linter import CMakeLinter, LintInfo
from catkin_lint.environment import CatkinEnvironment
from catkin_pkg.package import Package, Dependency, Person, Export
from catkin_lint.checks import all as all_checks
from catkin_lint.util import iteritems
from functools import wraps
from unittest import skip

import os
try:
    from unittest.mock import patch, mock_open, DEFAULT  # noqa
except ImportError:
    from mock import patch, mock_open, DEFAULT  # noqa

import posixpath
import ntpath


# Decorator to run test functions for both POSIX and Windows file systems
def posix_and_nt(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        with patch("os.path", posixpath):
            func(*args, **kwargs)
        with patch("os.path", ntpath):
            func(*args, **kwargs)
    if wrapper.__doc__ is not None:
        wrapper.__doc__ += " (POSIX/NT)"
    return wrapper


# Decorator to patch function if the module can be imported
def maybe_patch(target, new=DEFAULT):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            try:
                with patch(target, new):
                    return func(*args, **kwargs)
            except ImportError:
                return func(*args, **kwargs)
        return wrapper
    return decorator


def requires_module(name):  # pragma: no cover
    try:
        __import__(name)
        return lambda func: func
    except Exception:
        return skip("cannot import %s" % name)


def create_env(catkin_pkgs=["catkin", "message_generation", "message_runtime", "dynamic_reconfigure", "other_catkin", "other_msgs", "first_pkg", "second_pkg"], system_pkgs=["other_system"]):
    env = CatkinEnvironment(use_rosdep=False, use_rosdistro=False, use_cache=False)
    env.known_catkin_pkgs = set(catkin_pkgs)
    env.known_other_pkgs = set(system_pkgs)
    env.knows_everything = True

    def mock_get_manifest(name):
        assert name in catkin_pkgs
        return create_manifest(name, description="mocked %s" % name, buildtool_depends=["catkin"] if name != "catkin" else [], run_depends=["message_runtime"] if name.endswith("_msgs") or name == "dynamic_reconfigure" else [])

    env.get_manifest = mock_get_manifest
    return env


def create_manifest(name, description="", buildtool_depends=["catkin"], build_depends=[], run_depends=[], test_depends=[], meta=False):
    package = Package(
        name=name,
        version="0.0.0",
        package_format=1,
        description=description,
        maintainers=[Person("John Foo", "foo@bar.com")],
        buildtool_depends=[Dependency(d) for d in buildtool_depends],
        build_depends=[Dependency(d) for d in build_depends],
        run_depends=[Dependency(d) for d in run_depends],
        test_depends=[Dependency(d) for d in test_depends],
        exports=[Export("metapackage")] if meta else []
    )
    if hasattr(package, "evaluate_conditions"):
        package.evaluate_conditions({})
    return package


def create_manifest2(name, description="", buildtool_depends=["catkin"], build_depends=[], depends=[], buildtool_export_depends=[], build_export_depends=[], exec_depends=[], test_depends=[], meta=False):
    package = Package(
        name=name,
        version="0.0.0",
        package_format=2,
        description=description,
        maintainers=[Person("John Foo", "foo@bar.com")],
        depends=[Dependency(d) for d in depends],
        buildtool_depends=[Dependency(d) for d in buildtool_depends],
        build_depends=[Dependency(d) for d in build_depends],
        buildtool_export_depends=[Dependency(d) for d in buildtool_export_depends],
        build_export_depends=[Dependency(d) for d in build_export_depends],
        exec_depends=[Dependency(d) for d in exec_depends],
        test_depends=[Dependency(d) for d in test_depends],
        exports=[Export("metapackage")] if meta else []
    )
    if hasattr(package, "evaluate_conditions"):
        package.evaluate_conditions({})
    return package


def mock_lint(env, manifest, cmakelist, checks=all_checks, indentation=False, return_var=False, package_path=None):
    linter = CMakeLinter(env)
    if package_path is None:
        package_path = "/package-path/%s" % manifest.name
    if type(cmakelist) is dict:
        tmp = {}
        for key, value in iteritems(cmakelist):
            tmp[os.path.normpath(key)] = value
        cmakelist = tmp

    def get_cmakelist(filename):
        if type(cmakelist) is dict:
            if filename in cmakelist:
                return cmakelist[filename]
            else:
                raise OSError("Mock CMake file not found: %s" % filename)
        else:
            assert filename == os.path.normpath(package_path + "/CMakeLists.txt")
            return cmakelist

    linter._read_file = get_cmakelist
    if checks is not None:
        linter.require(checks)
    info = LintInfo(env)
    linter.lint(os.path.normpath(package_path), manifest, info=info)
    if not indentation:
        linter.messages = [m for m in linter.messages if m.id != "INDENTATION"]
    if return_var:
        return info.var
    else:
        return [m.id for m in linter.messages]
