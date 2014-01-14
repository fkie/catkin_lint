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
from ..linter import ERROR, WARNING
from ..cmake import argparse as cmake_argparse
from .build import depends as build_depends


def rosbuild_compat(linter):
    def on_fkie_find_package(info, cmd, args):
        opts, args = cmake_argparse(args, { "REQUIRED": "-", "COMPONENTS": "*" })
        info.report(ERROR if args[0] == "catkin" else WARNING, "DEPRECATED_ROSBUILD", cmd=cmd)
        if not "project" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="project")
        if args[0] in info.find_packages:
            info.report(ERROR, "DUPLICATE_FIND", pkg=args[0])
        if opts["REQUIRED"]: info.required_packages.add(args[0])
        info.find_packages.add(args[0])

    linter.require(build_depends)
    linter.add_command_hook("fkie_find_package", on_fkie_find_package)


def all(linter):
    linter.require(rosbuild_compat)
