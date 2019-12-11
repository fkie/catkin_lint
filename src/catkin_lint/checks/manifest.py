# coding=utf-8
#
# catkin_lint
# Copyright (c) 2013-2018 Fraunhofer FKIE
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

import re
import os
from lxml import etree as ET
from ..linter import ERROR, WARNING, NOTICE
from ..cmake import argparse as cmake_argparse
from .misc import project


def depends(linter):
    def on_init(info):
        info.buildtool_dep = set([dep.name for dep in info.manifest.buildtool_depends])
        info.build_dep = set([dep.name for dep in info.manifest.build_depends])
        info.export_dep = set()
        info.exec_dep = set()
        if info.manifest.package_format > 1:
            deps = set([dep.name for dep in info.manifest.build_export_depends])
            info.export_dep.update(deps)
            deps = set([dep.name for dep in info.manifest.buildtool_export_depends])
            info.export_dep.update(deps)
            deps = set([dep.name for dep in info.manifest.exec_depends])
            info.exec_dep.update(deps)
        if info.manifest.package_format < 2:
            deps = set([dep.name for dep in info.manifest.run_depends])
            info.export_dep.update(deps)
            info.exec_dep.update(deps)
        info.test_dep = set([dep.name for dep in info.manifest.test_depends])
        if info.env.ok:
            for pkg in info.buildtool_dep | info.build_dep | info.export_dep | info.exec_dep | info.test_dep:
                if not info.env.is_known_pkg(pkg):
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

    def on_final(info):
        for dirpath, dirnames, filenames in os.walk(info.path, topdown=True):
            for filename in filenames:
                fl = filename.lower()
                if fl.endswith(".launch") and "test" not in fl and "example" not in fl:
                    full_filename = os.path.join(dirpath, filename)
                    src_filename = os.path.relpath(full_filename, info.path)
                    with open(full_filename, "r") as f:
                        content = f.read()
                        try:
                            root = ET.fromstring(content)
                            for node in root.getiterator():
                                if node.tag is not ET.Comment:
                                    pkg = node.get("pkg")
                                    if pkg is not None and pkg != info.manifest.name and info.env.is_catkin_pkg(pkg) and pkg not in info.exec_dep and pkg not in essential_packages:
                                        info.report(WARNING, "LAUNCH_DEPEND", type="exec" if info.manifest.package_format > 1 else "run", pkg=pkg, file_location=(src_filename, node.sourceline or 0))
                                    for mo in re.finditer(r"\$\(find\s+([^<>)]+)\)", "<>".join(node.values() + [node.text or "", node.tail or ""])):
                                        pkg = mo.group(1)
                                        if pkg is not None and pkg != info.manifest.name and info.env.is_catkin_pkg(pkg) and pkg not in info.exec_dep and pkg not in essential_packages:
                                            info.report(WARNING, "LAUNCH_DEPEND", type="exec" if info.manifest.package_format > 1 else "run", pkg=pkg, file_location=(src_filename, node.sourceline or 0))
                        except ET.Error as err:
                            info.report(WARNING, "PARSE_ERROR", msg=str(err), file_location=(src_filename, 0))
            dirnames[:] = [d for d in dirnames if not d.startswith(".") and "test" not in d and "build" not in d and "example" not in d]

    linter.require(depends)
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
    linter.require(launch_depends)
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
    linter.require(catkin_build)
    linter.require(export_targets)
    linter.require(package_description)
