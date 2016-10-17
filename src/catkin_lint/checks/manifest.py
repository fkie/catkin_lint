#!/usr/bin/env python
"""
Copyright (c) 2013-2015 Fraunhofer FKIE

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of the Fraunhofer organization nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

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
import re
from ..linter import ERROR, NOTICE
from ..cmake import argparse as cmake_argparse
from .misc import project


def depends(linter):
    def on_init(info):
        buildtool_depends = [dep.name for dep in info.manifest.buildtool_depends]
        if buildtool_depends != list(sorted(buildtool_depends)):
            info.report(NOTICE, "UNSORTED_LIST", name="<buildtool_depend>")
        build_depends = [dep.name for dep in info.manifest.build_depends]
        if build_depends != list(sorted(build_depends)):
            info.report(NOTICE, "UNSORTED_LIST", name="<build_depend>")
        exec_depends = [dep.name for dep in info.manifest.exec_depends]
        if exec_depends != list(sorted(exec_depends)):
            info.report(NOTICE, "UNSORTED_LIST", name="<exec_depend>")

        info.buildtool_dep = set(buildtool_depends)
        info.build_dep = set(build_depends)
        info.export_dep = set()
        info.exec_dep = set()
        if info.manifest.package_format > 1 and hasattr(info.manifest, "build_export_depends"):
            deps = set([dep.name for dep in info.manifest.build_export_depends])
            for pkg in deps:
                if not info.env.is_known_pkg(pkg):
                    if info.env.ok:
                        info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="build_export")
            info.export_dep.update(deps)
        if info.manifest.package_format > 1 and hasattr(info.manifest, "buildtool_export_depends"):
            deps = set([dep.name for dep in info.manifest.buildtool_export_depends])
            for pkg in deps:
                if not info.env.is_known_pkg(pkg):
                    if info.env.ok:
                        info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="buildtool_export")
            info.export_dep.update(deps)
        if info.manifest.package_format > 1 and hasattr(info.manifest, "exec_depends"):
            deps = set([dep.name for dep in info.manifest.exec_depends])
            for pkg in deps:
                if not info.env.is_known_pkg(pkg):
                    if info.env.ok:
                        info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="exec")
            info.exec_dep.update(deps)
        if info.manifest.package_format < 2 and hasattr(info.manifest, "run_depends"):
            deps = set([dep.name for dep in info.manifest.run_depends])
            info.export_dep.update(deps)
            info.exec_dep.update(deps)
            for pkg in deps:
                if not info.env.is_known_pkg(pkg):
                    if info.env.ok:
                        info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="run")
        info.test_dep = set([dep.name for dep in info.manifest.test_depends])
        for pkg in info.buildtool_dep:
            if not info.env.is_known_pkg(pkg):
                if info.env.ok:
                    info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="buildtool")
        for pkg in info.build_dep:
            if not info.env.is_known_pkg(pkg):
                if info.env.ok:
                    info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="build")
        for pkg in info.test_dep:
            if not info.env.is_known_pkg(pkg):
                if info.env.ok:
                    info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="test")
        if info.manifest.is_metapackage() and info.build_dep:
            info.report(ERROR, "INVALID_META_DEPEND", type="build")
        if info.manifest.is_metapackage() and info.test_dep:
            info.report(ERROR, "INVALID_META_DEPEND", type="test")

    linter.add_init_hook(on_init)


def catkin_build(linter):
    def on_init(info):
        info.uses_catkin = False

    def any_catkin_cmd(info, cmd, args):
        info.uses_catkin = True

    def on_find_package(info, cmd, args):
        if args[0] == "catkin":
            info.uses_catkin = True

    def on_catkin_package(info, cmd, args):
        info.uses_catkin = True
        if info.manifest.is_metapackage():
            info.report(ERROR, "CATKIN_PKG_VS_META")
        if "catkin" not in info.find_packages and not info.is_catkin:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)

    def on_catkin_metapackage(info, cmd, args):
        info.uses_catkin = True
        if not info.manifest.is_metapackage():
            info.report(ERROR, "CATKIN_META_VS_PKG")
        if "catkin" not in info.find_packages and not info.is_catkin:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)

    def on_final(info):
        if "catkin" in info.build_dep:
            info.report(ERROR, "WRONG_DEPEND", pkg="catkin", wrong_type="build", right_type="buildtool")
        if not info.uses_catkin:
            if "catkin" not in info.find_packages and "catkin" in info.buildtool_dep:
                info.report(ERROR, "UNUSED_DEPEND", pkg="catkin", type="buildtool")
            return
        if "catkin" not in info.find_packages and not info.is_catkin:
            info.report(ERROR, "MISSING_FIND", pkg="catkin")
        if "catkin" not in info.buildtool_dep and not info.is_catkin:
            info.report(ERROR, "MISSING_DEPEND", pkg="catkin", type="buildtool")
        if "catkin_package" not in info.commands and "catkin_metapackage" not in info.commands:
            if info.manifest.is_metapackage():
                info.report(ERROR, "MISSING_CMD", cmd="catkin_metapackage")
            else:
                info.report(ERROR, "MISSING_CMD", cmd="catkin_package")

    linter.require(project)
    linter.require(depends)
    linter.add_init_hook(on_init)
    linter.add_command_hook("add_message_files", any_catkin_cmd)
    linter.add_command_hook("add_action_files", any_catkin_cmd)
    linter.add_command_hook("add_service_files", any_catkin_cmd)
    linter.add_command_hook("generate_messages", any_catkin_cmd)
    linter.add_command_hook("catkin_python_setup", any_catkin_cmd)
    linter.add_command_hook("find_package", on_find_package)
    linter.add_command_hook("catkin_package", on_catkin_package)
    linter.add_command_hook("catkin_metapackage", on_catkin_metapackage)
    linter.add_final_hook(on_final)


def export_targets(linter):
    def on_init(info):
        info.export_targets = set()

    def on_catkin_package(info, cmd, args):
        opts, args = cmake_argparse(args, {"INCLUDE_DIRS": "*", "LIBRARIES": "*", "DEPENDS": "*", "CATKIN_DEPENDS": "*", "CFG_EXTRAS": "*", "EXPORTED_TARGETS": "*"})
        info.export_targets |= set(opts["EXPORTED_TARGETS"])

    def on_final(info):
        for tgt in info.export_targets - info.targets:
            info.report(ERROR, "UNDEFINED_TARGET", target=tgt)

    linter.add_init_hook(on_init)
    linter.add_command_hook("catkin_package", on_catkin_package)
    linter.add_final_hook(on_final)


def package_description(linter):
    # Check for meaningless package descriptions
    buzzwords = [
        r"(an)?other",
        r"(python\s+|c(\+\+)?\s+|java\s+)?code(\s+snippets?)?",
        r"(ros\s+)?nodes?",
        r"\d+",
        r"a\s+few",
        r"add(s|ed)?",
        r"all",
        r"an?",
        r"and",
        r"are",
        r"be",
        r"boilerplate",
        r"both",
        r"can",
        r"comprise(s|d)?",
        r"contain(s|ed)?",
        r"describe(s|d)?",
        r"descriptions?",
        r"did",
        r"different",
        r"do(es)?",
        r"done",
        r"examples?",
        r"executables?",
        r"for",
        r"from",
        r"functionalit(y|ies)",
        r"had",
        r"has",
        r"have",
        r"implementations?",
        r"in",
        r"include(s|d)?",
        r"interfaces?",
        r"into",
        r"is",
        r"librar(y|ies)",
        r"meaningless",
        r"miscellaneous",
        r"multiple",
        r"no(thing|ne|t)?",
        r"of",
        r"offer(s|ed)",
        r"one",
        r"or",
        r"packages?",
        r"programs?",
        r"provide(s|d)?",
        r"purpose",
        r"routines?",
        r"runs?",
        r"sets?",
        r"several",
        r"some",
        r"suppl(y|ies|ied)",
        r"that",
        r"the",
        r"these",
        r"this",
        r"those",
        r"three",
        r"to",
        r"todo",
        r"tools?",
        r"two",
        r"use[sd]?",
        r"useful",
        r"various",
        r"versions?",
        r"which",
        r"with",
        r"work\s+in\s+progress",
    ]

    def on_init(info):
        chatter = re.match(r"((\s|[.,!?;()/])*\b(%s|%s)\b)+" % (info.manifest.name, r"|".join(buzzwords)), info.manifest.description, re.IGNORECASE)
        if chatter is not None:
            s = chatter.group(0).replace("\n", " ").strip()
            if info.manifest.description[chatter.end(0):].strip():
                if len(s.split()) > 1:
                    info.report(NOTICE, "DESCRIPTION_BOILERPLATE", text=s)
            else:
                info.report(NOTICE, "DESCRIPTION_MEANINGLESS", text=s)

    linter.add_init_hook(on_init)


def all(linter):
    linter.require(depends)
    linter.require(catkin_build)
    linter.require(export_targets)
    linter.require(package_description)
