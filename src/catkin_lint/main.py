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
import sys
import argparse
import textwrap
from . import __version__ as catkin_lint_version
from .linter import CatkinEnvironment, CMakeLinter, ERROR, WARNING, NOTICE

def main():
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--version", action="version", version=catkin_lint_version)
        parser.add_argument("path", nargs="*", default=[], help="path to catkin packages")
        parser.add_argument("-W", metavar="LEVEL", type=int, default=0, help="set warning level (0-2)")
        parser.add_argument("--strict", action="store_true", help="treat warnings as errors")
        parser.add_argument("--explain", action="store_true", help="show additional explanations")
        parser.add_argument("--pkg", action="append", default=[], help="specify catkin package by name")
        parser.add_argument("--debug", action="store_true", help="show stack trace on exceptions")
        args = parser.parse_args()
        nothing_to_do = 0
        pkgs_to_check = []
        env = CatkinEnvironment()
        if not args.path and not args.pkg:
            if os.path.isfile("package.xml"):
                pkgs_to_check += env.add_path(os.getcwd())
            else:
                sys.stderr.write("catkin_lint: no path given and no package.xml in current directory\n")
                sys.exit(0)
        if "ROS_PACKAGE_PATH" in os.environ:
            for path in os.environ["ROS_PACKAGE_PATH"].split(os.pathsep):
                env.add_path(path)
        for path in args.path:
            if not os.path.isdir(path):
                sys.stderr.write("catkin_lint: not a directory: %s\n" % path)
                nothing_to_do = 1
                continue
            pkgs_to_check += env.add_path(path)
        for name in args.pkg:
            if not name in env.known_catkin_pkgs:
                sys.stderr.write("catkin_lint: no such package: %s\n" % name)
                nothing_to_do = 1
                continue
            pkgs_to_check.append(env.manifests[name])
        if not pkgs_to_check:
            sys.stderr.write ("catkin_lint: no packages to check\n")
            sys.exit(nothing_to_do)
        linter = CMakeLinter(env)
        import catkin_lint.checks
        linter.require(catkin_lint.checks.all)
        for path, manifest in pkgs_to_check:
            try:
                linter.lint(path, manifest)
            except Exception as err:
                sys.stderr.write("catkin_lint: cannot lint %s: %s\n" % (manifest.name, str(err)))
                if args.debug: raise
        explained = set([])
        diagnostic_label = { ERROR : "error", WARNING : "warning", NOTICE : "notice" }
        suppressed = { ERROR: 0, WARNING: 0, NOTICE: 0 }
        problems = 0
        exit_code = 0
        for pkg, filename, line, level, msg_id, text, explanation in sorted(linter.messages):
            if args.W < level:
                suppressed[level] += 1
                continue
            if args.strict or level == ERROR: exit_code = 1
            problems += 1
            loc = pkg
            if filename:
                if line:
                    fn = "%s(%d)" % (filename, line)
                else:
                    fn = filename
                loc = "%s: %s" % (pkg, fn)
            sys.stdout.write("%s: %s: %s\n" % (loc, diagnostic_label[ERROR] if args.strict else diagnostic_label[level], text))
            if args.explain and not msg_id in explained:
                explained.add(msg_id)
                sys.stdout.write("%s\n" % textwrap.fill(explanation, initial_indent="     * ", subsequent_indent="     * "))
        sys.stderr.write ("catkin_lint: checked %d packages and found %d problems\n" % (len(pkgs_to_check), problems))
        for level in [ ERROR, WARNING, NOTICE ]:
            if suppressed[level] > 0:
                sys.stderr.write ("catkin_lint: %d %ss have been ignored. Use -W%d to see them\n" % (suppressed[level], diagnostic_label[level], level))
        sys.exit(exit_code)
    except Exception as err:
        sys.stderr.write("catkin_lint: internal error: %s\n\n" % str(err))
        if args and args.debug: raise
        sys.exit(2)

