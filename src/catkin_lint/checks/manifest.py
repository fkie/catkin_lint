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
import re
from ..linter import ERROR, NOTICE
from ..cmake import argparse as cmake_argparse
from .misc import project


def depends(linter):
    def on_init(info):
        info.buildtool_dep = set([ dep.name for dep in info.manifest.buildtool_depends ])
        info.build_dep = set([ dep.name for dep in info.manifest.build_depends ])
        info.run_dep = set([ dep.name for dep in info.manifest.run_depends ])
        info.test_dep = set([ dep.name for dep in info.manifest.test_depends ])
        for pkg in (info.build_dep | info.run_dep) & info.test_dep:
            if info.env.is_known_pkg(pkg):
                info.report (ERROR, "REDUNDANT_TEST_DEPEND", pkg=pkg)
        if info.env.has_rosdep():
            for pkg in info.buildtool_dep:
                if not info.env.is_known_pkg(pkg):
                    info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="buildtool")
            for pkg in info.build_dep:
                if not info.env.is_known_pkg(pkg):
                    info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="build")
            for pkg in info.run_dep:
                if not info.env.is_known_pkg(pkg):
                    info.report(ERROR, "UNKNOWN_DEPEND", pkg=pkg, type="run")
            for pkg in info.test_dep:
                if not info.env.is_known_pkg(pkg):
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
        if not "catkin" in info.find_packages and not info.is_catkin:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)
    def on_catkin_metapackage(info, cmd, args):
        info.uses_catkin = True
        if not info.manifest.is_metapackage():
            info.report (ERROR, "CATKIN_META_VS_PKG")
        if not "catkin" in info.find_packages and not info.is_catkin:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)
    def on_final(info):
        if "catkin" in info.build_dep:
            info.report(ERROR, "WRONG_DEPEND", pkg="catkin", wrong_type="build", right_type="buildtool")
        if not info.uses_catkin:
            if not "catkin" in info.find_packages and "catkin" in info.buildtool_dep:
                info.report(ERROR, "UNUSED_DEPEND", pkg="catkin", type="buildtool")
            return
        if not "catkin" in info.find_packages and not info.is_catkin:
            info.report(ERROR, "MISSING_FIND", pkg="catkin")
        if not "catkin" in info.buildtool_dep and not info.is_catkin:
            info.report(ERROR, "MISSING_DEPEND", pkg="catkin", type="buildtool")
        if not "catkin_package" in info.commands and not "catkin_metapackage" in info.commands:
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
        info.export_targets = set([])
    def on_catkin_package(info, cmd, args):
        opts, args = cmake_argparse(args, { "INCLUDE_DIRS": "*", "LIBRARIES": "*", "DEPENDS": "*", "CATKIN_DEPENDS": "*", "CFG_EXTRAS": "*", "EXPORTED_TARGETS": "*" })
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
