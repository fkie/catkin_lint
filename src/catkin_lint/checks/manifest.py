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

import re
import os
from lxml import etree as ET
from ..linter import ERROR, WARNING, NOTICE
from ..cmake import argparse as cmake_argparse
from ..util import enumerate_package_files, is_active
from ..environment import PackageType
from .misc import project


def depends(linter):
    def on_init(info):
        info.buildtool_dep = {dep.name for dep in info.manifest.buildtool_depends if is_active(dep)}
        info.build_dep = {dep.name for dep in info.manifest.build_depends if is_active(dep)}
        info.export_dep = set()
        info.exec_dep = set()
        if info.manifest.package_format > 1:
            deps = {dep.name for dep in info.manifest.build_export_depends if is_active(dep)}
            info.export_dep.update(deps)
            deps = {dep.name for dep in info.manifest.buildtool_export_depends if is_active(dep)}
            info.export_dep.update(deps)
            deps = {dep.name for dep in info.manifest.exec_depends if is_active(dep)}
            info.exec_dep.update(deps)
        if info.manifest.package_format < 2:
            deps = {dep.name for dep in info.manifest.run_depends}
            info.export_dep.update(deps)
            info.exec_dep.update(deps)
        info.test_dep = {dep.name for dep in info.manifest.test_depends if is_active(dep)}
        for pkg in info.buildtool_dep | info.build_dep | info.export_dep | info.exec_dep | info.test_dep:
            if info.env.get_package_type(pkg) == PackageType.UNKNOWN:
                info.report(ERROR, "UNKNOWN_PACKAGE", pkg=pkg, file_location=("package.xml", 0))
        if info.manifest.is_metapackage() and info.build_dep:
            info.report(ERROR, "INVALID_META_DEPEND", type="build", file_location=("package.xml", 0))
        if info.manifest.is_metapackage() and info.test_dep:
            info.report(ERROR, "INVALID_META_DEPEND", type="test", file_location=("package.xml", 0))

    linter.add_init_hook(on_init)


def launch_depends(linter):

    # We assume that the following packages are always present in a sane ROS environment, so we do
    # not litter the log with warnings about missing exec_depends if these are used in launch files
    essential_packages = set(["rosbag", "rosnode", "rosservice", "rostopic"])

    def on_init(info):
        info.test_launch_files = set()

    def on_add_rostest(info, cmd, args):
        launch_file = args[0 if cmd == "add_rostest" else 1]
        if not info.is_existing_path(launch_file, check=os.path.isfile):
            info.report(ERROR, "MISSING_FILE", cmd=cmd, file=launch_file)
        info.test_launch_files.add(info.source_relative_path(launch_file))

    def on_final(info):
        for dirpath, filename in enumerate_package_files(info.path):
            if filename.lower().endswith(".launch"):
                full_filename = os.path.join(dirpath, filename)
                src_filename = os.path.relpath(full_filename, info.path)
                exec_dep_type = "exec" if info.manifest.package_format > 1 else "run"
                is_test_launch_file = src_filename in info.test_launch_files
                with open(full_filename, "rb") as f:
                    content = f.read()
                    try:
                        root = ET.fromstring(content)
                        for node in root.getiterator():
                            if node.tag is ET.Comment:
                                args = (node.text or "").split()
                                if args and args[0] == "catkin_lint:" and args[1] in ["ignore_once", "ignore", "report"]:
                                    msg_ids = set([a.upper() for a in args[2:]])
                                    if args[1] == "ignore":
                                        info.ignore_message_ids |= msg_ids
                                    if args[1] == "report":
                                        info.ignore_message_ids -= msg_ids
                                    if args[1] == "ignore_once":
                                        info.ignore_message_ids_once |= msg_ids
                            else:
                                relevant_deps = info.exec_dep
                                dep_type = exec_dep_type
                                if node.tag == "test" or is_test_launch_file:
                                    relevant_deps |= info.test_dep
                                    dep_type = "test"
                                pkg = node.get("pkg")
                                if pkg is not None and pkg != info.manifest.name and info.env.get_package_type(pkg) == PackageType.CATKIN and pkg not in relevant_deps and pkg not in essential_packages:
                                    info.report(WARNING, "LAUNCH_DEPEND", type=dep_type, pkg=pkg, file_location=(src_filename, node.sourceline or 0))
                                for mo in re.finditer(r"\$\(find\s+([^<>)]+)\)", "<>".join(node.values() + [node.text or "", node.tail or ""])):
                                    pkg = mo.group(1)
                                    if pkg is not None and pkg != info.manifest.name and info.env.get_package_type(pkg) == PackageType.CATKIN and pkg not in relevant_deps and pkg not in essential_packages:
                                        info.report(WARNING, "LAUNCH_DEPEND", type=dep_type, pkg=pkg, file_location=(src_filename, node.sourceline or 0))
                                info.ignore_message_ids_once.clear()
                    except (ET.Error, ValueError) as err:
                        info.report(WARNING, "PARSE_ERROR", msg=str(err), file_location=(src_filename, 0))

    linter.require(depends)
    linter.add_init_hook(on_init)
    linter.add_command_hook("add_rostest", on_add_rostest)
    linter.add_command_hook("add_rostest_gtest", on_add_rostest)
    linter.add_final_hook(on_final)


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
            info.report(ERROR, "WRONG_CATKIN_PACKAGE")
        if "catkin" not in info.find_packages and not info.is_catkin:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)

    def on_catkin_metapackage(info, cmd, args):
        info.uses_catkin = True
        if not info.manifest.is_metapackage():
            info.report(ERROR, "WRONG_CATKIN_METAPACKAGE")
        if "catkin" not in info.find_packages and not info.is_catkin:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)

    def on_final(info):
        if "catkin" in info.build_dep:
            info.report(ERROR, "WRONG_DEPEND", pkg="catkin", wrong_type="build", right_type="buildtool", file_location=("package.xml", 0))
        if not info.uses_catkin:
            if "catkin" not in info.find_packages and "catkin" in info.buildtool_dep:
                info.report(ERROR, "UNUSED_DEPEND", pkg="catkin", type="buildtool", file_location=("package.xml", 0))
            return
        if "catkin" not in info.find_packages and not info.is_catkin:
            info.report(ERROR, "MISSING_FIND", pkg="catkin", file_location=("CMakeLists.txt", 0))
        if "catkin" not in info.buildtool_dep and not info.is_catkin:
            info.report(ERROR, "MISSING_DEPEND", pkg="catkin", type="buildtool", file_location=("package.xml", 0))
        if "catkin_package" not in info.commands and "catkin_metapackage" not in info.commands:
            if info.manifest.is_metapackage():
                info.report(ERROR, "MISSING_CMD", cmd="catkin_metapackage", file_location=("CMakeLists.txt", 0))
            else:
                info.report(ERROR, "MISSING_CMD", cmd="catkin_package", file_location=("CMakeLists.txt", 0))

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
            info.report(ERROR, "UNDEFINED_TARGET", target=tgt, file_location=info.location_of("catkin_package"))

    linter.add_init_hook(on_init)
    linter.add_command_hook("catkin_package", on_catkin_package)
    linter.add_final_hook(on_final)


def package_description(linter):
    # Check for meaningless package descriptions
    buzzwords = [
        r"\d+",
        r"(an)?other",
        r"add(s|ed)?",
        r"all",
        r"an?",
        r"and",
        r"are",
        r"be",
        r"boilerplate",
        r"both",
        r"c(\+\+)?",
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
        r"java",
        r"librar(y|ies)",
        r"meaningless",
        r"metapackages?",
        r"meta",
        r"miscellaneous",
        r"multiple",
        r"nodes?",
        r"no(thing|ne|t)?",
        r"of",
        r"offer(s|ed)",
        r"one",
        r"or",
        r"packages?",
        r"programs?",
        r"progress"
        r"provide(s|d)?",
        r"purpose",
        r"python",
        r"ros",
        r"routines?",
        r"runs?",
        r"sets?",
        r"several",
        r"snippets?",
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
        r"wrapper",
        r"work",
        r"yet",
    ]
    buzzwords_re = re.compile(r"^(%s)$" % "|".join(buzzwords))

    def on_init(info):
        words = info.manifest.description.strip().split()
        name = info.manifest.name.lower()
        chatter = []
        for word in words:
            word_lc = word.lower()
            if word_lc == name or buzzwords_re.match(word_lc):
                chatter.append(word)
            else:
                break
        buzzwordiness = len(chatter)
        if buzzwordiness > 1:
            s = " ".join(chatter)
            if buzzwordiness == len(words):
                info.report(NOTICE, "DESCRIPTION_MEANINGLESS", text=s, file_location=("package.xml", 0))
            elif buzzwordiness > 1:
                info.report(NOTICE, "DESCRIPTION_BOILERPLATE", text=s, file_location=("package.xml", 0))

    linter.add_init_hook(on_init)


def all(linter):
    linter.require(depends)
    linter.require(launch_depends)
    linter.require(catkin_build)
    linter.require(export_targets)
    linter.require(package_description)
