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
from catkin_lint.main import ERROR, WARNING, NOTICE
import catkin_lint.cmake as cmake
import os
import re

def catkin_depends(linter):
    def on_init(info):
        info.is_catkin = False
        info.required_packages = set([])
        info.catkin_packages = set([])
        info.export_targets = set([])
    def on_find_package(info, cmd, args):
        opts, args = cmake.argparse(args, { "REQUIRED": "-", "COMPONENTS": "*" })
        if not "project" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="project")
        if args[0] in info.find_packages:
            info.report(ERROR, "DUPLICATE_FIND", pkg=args[0])
        if opts["REQUIRED"]: info.required_packages.add(args[0])
        if args[0] != "catkin": return
        info.is_catkin = True
        if "catkin_package" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="catkin_package", second_cmd=cmd)
        if not opts["REQUIRED"]:
            info.report(WARNING, "MISSING_REQUIRED", pkg="catkin")
        for pkg in opts["COMPONENTS"]:
            if pkg in info.find_packages:
                info.report(ERROR, "DUPLICATE_FIND", pkg=pkg)
            if not info.env.is_catkin_pkg(pkg):
                info.report(ERROR, "NO_CATKIN_COMPONENT", pkg=pkg)
        info.find_packages |= set(opts["COMPONENTS"])
        info.required_packages |= set(opts["COMPONENTS"])
    def on_fkie_find_package(info, cmd, args):
        info.report(NOTICE, "DEPRECATED_ROSBUILD", cmd=cmd)
        on_find_package(info, cmd, args)
        info.find_packages.add(args[0])
    def on_catkin_package(info, cmd, args):
        info.is_catkin = True
        if info.manifest.is_metapackage():
            info.report(ERROR, "CATKIN_PKG_VS_META")
        if not "catkin" in info.find_packages:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="find_package")
        opts, args = cmake.argparse(args, { "INCLUDE_DIRS": "*", "LIBRARIES": "*", "DEPENDS": "*", "CATKIN_DEPENDS": "*", "CFG_EXTRAS": "*", "EXPORTED_TARGETS": "*" })
        for pkg in opts["CATKIN_DEPENDS"]:
            if not info.env.is_catkin_pkg(pkg):
                info.report(ERROR, "SYSTEM_AS_CATKIN_DEPEND", pkg=pkg)
        for pkg in opts["DEPENDS"]:
            if info.env.is_catkin_pkg(pkg):
                info.report(ERROR, "CATKIN_AS_SYSTEM_DEPEND", pkg=pkg)
            elif not pkg in info.find_packages and not ("%s_INCLUDE_DIRS" % pkg in info.var and "%s_LIBRARIES" % pkg in info.var):
                info.report(ERROR, "UNCONFIGURED_SYSTEM_DEPEND", pkg=pkg)
        info.catkin_packages |= set(opts["CATKIN_DEPENDS"])
        info.export_targets |= set(opts["EXPORTED_TARGETS"])
    def on_catkin_metapackage(info, cmd, args):
        info.is_catkin = True
        if not info.manifest.is_metapackage():
            info.report (ERROR, "CATKIN_META_VS_PKG")
        if not "catkin" in info.find_packages:
            info.report (ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="find_package")
    def on_add_msg_files(info, cmd, args):
        info.is_catkin = True
    def on_generate_msg(info, cmd, args):
        info.is_catkin = True
    def on_include(info, cmd, args):
        opts, args = cmake.argparse(args, { "OPTIONAL" : "-", "RESULT_VARIABLE" : "?", "NO_POLICY_SCOPE" : "-"})
        if args:
            mo = re.search(r"\bFind([A-Za-z0-9]+)(\.cmake)?$", args[0])
            if mo:
                pkg = mo.group(1)
                if not os.path.isfile(os.path.join(info.path, args[0])) and not opts["OPTIONAL"]:
                    info.report (ERROR, "FIND_BY_INCLUDE", pkg=pkg)
                info.var["%s_INCLUDE_DIRS" % pkg] = "/%s-includes" % pkg
                info.var["%s_INCLUDE_DIRS" % pkg.upper()] = "/%s-includes" % pkg
                info.var["%s_LIBRARIES" % pkg] = "/%s-libs/library.so" % pkg
                info.var["%s_LIBRARIES" % pkg.upper()] = "/%s-libs/library.so" % pkg
                info.find_packages.add(pkg)
    def on_final(info):
        pkg_buildtool_dep = set([dep.name for dep in info.manifest.buildtool_depends])
        pkg_build_dep = set([dep.name for dep in info.manifest.build_depends])
        pkg_run_dep = set([dep.name for dep in info.manifest.run_depends])
        pkg_test_dep = set([dep.name for dep in info.manifest.test_depends])
        if info.is_catkin and not "catkin" in info.find_packages:
            info.report(ERROR, "MISSING_FIND", pkg="catkin")
        if info.is_catkin and not "catkin" in pkg_buildtool_dep:
            info.report(ERROR, "MISSING_DEPEND", pkg="catkin", type="buildtool")
        if not info.is_catkin and not "catkin" in pkg_buildtool_dep: return
        if not "catkin_package" in info.commands and not info.manifest.is_metapackage():
            info.report(ERROR, "MISSING_CMD", cmd="catkin_package")
        if not "catkin_metapackage" in info.commands and info.manifest.is_metapackage():
            info.report(ERROR, "MISSING_CMD", cmd="catkin_metapackage")
        for pkg in info.required_packages - pkg_build_dep - pkg_buildtool_dep:
            if info.env.is_known_pkg(pkg):
                info.report(ERROR, "MISSING_DEPEND", pkg=pkg, type="build")
        for pkg in info.catkin_packages - pkg_run_dep:
            info.report(ERROR, "MISSING_DEPEND", pkg=pkg, type="run")
        for pkg in pkg_build_dep - info.find_packages:
            if info.env.is_catkin_pkg(pkg):
                info.report(ERROR, "UNCONFIGURED_BUILD_DEPEND", pkg=pkg)
        for pkg in (pkg_build_dep | pkg_run_dep) & pkg_test_dep:
            if info.env.is_known_pkg(pkg):
                info.report (ERROR, "REDUNDANT_TEST_DEPEND", pkg=pkg)
        if info.env.has_rosdep():
            for pkg in pkg_buildtool_dep:
                if not info.env.is_known_pkg(pkg):
                    info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="buildtool")
            for pkg in pkg_build_dep:
                if not info.env.is_known_pkg(pkg):
                    info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="build")
            for pkg in pkg_run_dep:
                if not info.env.is_known_pkg(pkg):
                    info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="run")
            for pkg in pkg_test_dep:
                if not info.env.is_known_pkg(pkg):
                    info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="test")
        for tgt in info.export_targets - info.targets:
            info.report(ERROR, "UNDEFINED_TARGET", target=tgt)
        if info.manifest.is_metapackage() and pkg_build_dep:
            info.report(ERROR, "INVALID_META_DEPEND", type="build")
        if info.manifest.is_metapackage() and pkg_test_dep:
            info.report(ERROR, "INVALID_META_DEPEND", type="test")
        for pkg in (info.find_packages & pkg_build_dep & pkg_run_dep) - info.catkin_packages:
            if re.search(r"_(msg|message)s?(_|$)", pkg) and info.env.is_catkin_pkg(pkg):
                info.report (WARNING, "SUGGEST_CATKIN_DEPEND", pkg=pkg)
    linter.add_init_hook(on_init)
    linter.add_command_hook("find_package", on_find_package)
    linter.add_command_hook("fkie_find_package", on_fkie_find_package)
    linter.add_command_hook("catkin_package", on_catkin_package)
    linter.add_command_hook("catkin_metapackage", on_catkin_metapackage)
    linter.add_command_hook("add_message_files", on_add_msg_files)
    linter.add_command_hook("add_service_files", on_add_msg_files)
    linter.add_command_hook("add_action_files", on_add_msg_files)
    linter.add_command_hook("generate_messages", on_generate_msg)
    linter.add_command_hook("include", on_include)
    linter.add_final_hook(on_final)


def message_generation(linter):
    def on_init(info):
        info.declares_messages = False
        info.msg_deps = set([])
    def on_add_msg_files(info, cmd, args):
        info.is_catkin = True
        if info.manifest.is_metapackage():
            info.report(ERROR, "INVALID_META_COMMAND", cmd=cmd)
        info.declares_messages = True
        if "generate_messages" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="generate_messages", second_cmd=cmd)
        if not "catkin" in info.find_packages:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="find_package")
        if "catkin_package" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="catkin_package", second_cmd=cmd)
    def on_generate_msg(info, cmd, args):
        info.is_catkin = True
        if info.manifest.is_metapackage():
            info.report(ERROR, "INVALID_META_COMMAND", cmd=cmd)
        if not "catkin" in info.find_packages:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="find_package")
        if "catkin_package" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="catkin_package", second_cmd=cmd)
        opts, args = cmake.argparse(args, { "DEPENDENCIES": "*" })
        info.msg_deps |= set(opts["DEPENDENCIES"])
    def on_final(info):
        if info.manifest.is_metapackage(): return
        pkg_build_dep = set([dep.name for dep in info.manifest.build_depends])
        pkg_run_dep = set([dep.name for dep in info.manifest.run_depends])
        for pkg in info.msg_deps - info.catkin_packages:
            info.report(ERROR, "MISSING_MSG_CATKIN", pkg=pkg)
        for pkg in info.msg_deps - pkg_build_dep:
            info.report (ERROR, "MISSING_MSG_DEPEND", pkg=pkg, type="build")
        for pkg in info.msg_deps - pkg_run_dep:
            info.report (ERROR, "MISSING_MSG_DEPEND", pkg=pkg, type="run")
        if info.declares_messages and not "generate_messages" in info.commands:
            info.report(ERROR, "MISSING_GENERATE_MSG")
        if not info.declares_messages and "generate_messages" in info.commands:
            info.report(WARNING, "UNUSED_GENERATE_MSG")
        if info.declares_messages and not "message_generation" in info.find_packages:
            info.report (ERROR, "UNCONFIGURED_BUILD_DEPEND", pkg="message_generation")
        if info.declares_messages and not "message_runtime" in info.catkin_packages:
            info.report(ERROR, "MISSING_CATKIN_DEPEND", pkg="message_runtime")
    linter.add_init_hook(on_init)
    linter.add_command_hook("add_message_files", on_add_msg_files)
    linter.add_command_hook("add_service_files", on_add_msg_files)
    linter.add_command_hook("add_action_files", on_add_msg_files)
    linter.add_command_hook("generate_messages", on_generate_msg)
    linter.add_final_hook(on_final)
