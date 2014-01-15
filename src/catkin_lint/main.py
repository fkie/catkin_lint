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
import importlib
from . import __version__ as catkin_lint_version
from .linter import CatkinEnvironment, CMakeLinter, ERROR, WARNING, NOTICE
from .output import TextOutput, ExplainedTextOutput, XmlOutput

import catkin_lint.checks

def add_linter_check(linter, check):
    if "." in check:
        pkg, func = check.rsplit(".", 1)
        check_module = importlib.import_module(pkg, "catkin_lint.checks")
    else:
        check_module = catkin_lint.checks
        func = check
    linter.require(getattr(check_module, func))


def main():
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--version", action="version", version=catkin_lint_version)
        parser.add_argument("path", nargs="*", default=[], help="path to catkin packages")
        parser.add_argument("-q", "--quiet", action="store_true", help="suppress final summary")
        parser.add_argument("-W", metavar="LEVEL", type=int, default=0, help="set warning level (0-2)")
        parser.add_argument("-c", "--check", metavar="MODULE.CHECK", action="append", default=[ "all" ], help=argparse.SUPPRESS)
        parser.add_argument("--strict", action="store_true", help="treat warnings as errors")
        parser.add_argument("--pkg", action="append", default=[], help="specify catkin package by name (can be used multiple times)")
        group = parser.add_mutually_exclusive_group()
        group.add_argument("--text", action="store_true", help="output result as text (default)")
        group.add_argument("--explain", action="store_true", help="output result as text with explanations")
        group.add_argument("--xml", action="store_true", help="output result as XML")
        parser.add_argument("--debug", action="store_true", help=argparse.SUPPRESS)
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
        if args.xml:
            output = XmlOutput()
        elif args.explain:
            output = ExplainedTextOutput()
        else:
            output = TextOutput()
        linter = CMakeLinter(env)
        for check in args.check:
            try:
                add_linter_check(linter, check)
            except Exception as err:
                sys.stderr.write("catkin_lint: cannot import '%s': %s\n" % (check, str(err)))
                if args.debug: raise
                sys.exit(1)
        for path, manifest in pkgs_to_check:
            try:
                linter.lint(path, manifest)
            except Exception as err:
                sys.stderr.write("catkin_lint: cannot lint %s: %s\n" % (manifest.name, str(err)))
                if args.debug: raise
        suppressed = { ERROR: 0, WARNING: 0, NOTICE: 0 }
        problems = 0
        exit_code = 0
        diagnostic_label = { ERROR : "error", WARNING : "warning", NOTICE : "notice" }
        output.prolog(file=sys.stdout)
        for msg in sorted(linter.messages):
            if args.W < msg.level:
                suppressed[msg.level] += 1
                continue
            if args.strict: msg.level = ERROR
            if msg.level == ERROR:
                exit_code = 1
            output.message(msg, file=sys.stdout)
            problems += 1
        output.epilog(file=sys.stdout)
        if not args.quiet:
            sys.stderr.write ("catkin_lint: checked %d packages and found %d problems\n" % (len(pkgs_to_check), problems))
            for level in [ ERROR, WARNING, NOTICE ]:
                if suppressed[level] > 0:
                    sys.stderr.write ("catkin_lint: %d %ss have been ignored. Use -W%d to see them\n" % (suppressed[level], diagnostic_label[level], level))
        sys.exit(exit_code)
    except Exception as err:
        sys.stderr.write("catkin_lint: internal error: %s\n\n" % str(err))
        if args and args.debug: raise
        sys.exit(2)

