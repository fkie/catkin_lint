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
from catkin_lint.linter import ERROR, WARNING


def project(linter):
    def on_project(info, cmd, args):
        if args[0] != info.manifest.name:
            info.report(ERROR, "PROJECT_NAME", name=args[0])

    linter.add_command_hook("project", on_project)


def special_vars(linter):
    # Immutable variables must not be changed at all
    immutable_vars = frozenset([
        "CMAKE_BUILD_TYPE",
        "CMAKE_C_COMPILER",
        "CMAKE_CXX_COMPILER",
        "PROJECT_NAME",
        "EXECUTABLE_OUTPUT_PATH",
        "LIBRARY_OUTPUT_PATH",
        "CATKIN_PACKAGE_BIN_DESTINATION",
        "CATKIN_PACKAGE_ETC_DESTINATION",
        "CATKIN_PACKAGE_INCLUDE_DESTINATION",
        "CATKIN_PACKAGE_LIB_DESTINATION",
        "CATKIN_PACKAGE_PYTHON_DESTINATION",
        "CATKIN_PACKAGE_SHARE_DESTINATION",
        "CATKIN_GLOBAL_BIN_DESTINATION",
        "CATKIN_GLOBAL_ETC_DESTINATION",
        "CATKIN_GLOBAL_INCLUDE_DESTINATION",
        "CATKIN_GLOBAL_LIB_DESTINATION",
        "CATKIN_GLOBAL_LIBEXEC_DESTINATION",
        "CATKIN_GLOBAL_PYTHON_DESTINATION",
        "CATKIN_GLOBAL_SHARE_DESTINATION"
    ])
    # Critical variables contain important values that must not be overwritten,
    # but appending additional items is okay
    critical_vars = frozenset([
        "CMAKE_C_FLAGS",
        "CMAKE_CXX_FLAGS",
        "CMAKE_INCLUDE_PATH",
        "CMAKE_LIBRARY_PATH",
        "CMAKE_FIND_ROOT_PATH",
        "CMAKE_MODULE_PATH","CMAKE_PREFIX_PATH"
    ])

    def on_init(info):
        for key in critical_vars:
            info.var[key] = "@%s@" % key
    def on_set_or_unset(info, cmd, args):
        if args[0] in immutable_vars:
            info.report(ERROR, "IMMUTABLE_VAR", var=args[0])
        if args[0] in critical_vars:
            value = ';'.join(args[1:])
            if cmd == "unset" or not "@%s@" % args[0] in value:
                info.report(ERROR, "CRITICAL_VAR_OVERWRITE", var=args[0])
            else:
                info.report(WARNING, "CRITICAL_VAR_APPEND", var=args[0])

    linter.add_init_hook(on_init)
    linter.add_command_hook("set", on_set_or_unset)
    linter.add_command_hook("unset", on_set_or_unset)


def singleton_commands(linter):
    # Singleton commands may not appear more than once
    singleton_cmds = frozenset([
        "project",
        "generate_messages",
        "catkin_package",
        "catkin_metapackage",
        "catkin_python_setup"
    ])
    def on_command(info, cmd, args):
        if cmd in info.commands:
            info.report(ERROR, "DUPLICATE_CMD", cmd=cmd)

    for cmd in singleton_cmds:
        linter.add_command_hook(cmd, on_command)


def all(linter):
    linter.require(project)
    linter.require(special_vars)
    linter.require(singleton_commands)
