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
import os
import re
from ..linter import ERROR, WARNING, NOTICE
from ..cmake import argparse as cmake_argparse
from ..util import word_split, iteritems, is_sorted
from .manifest import depends as manifest_depends
from functools import partial


def includes(linter):
    def on_init(info):
        info.build_includes = set()

    def on_include_directories(info, cmd, args):
        _, args = cmake_argparse(args, {"AFTER": "-", "BEFORE": "-", "SYSTEM": "-"})
        includes = set([info.package_path(d) for d in args])
        info.build_includes |= includes

    def on_final(info):
        for incl in info.build_includes:
            if os.path.isabs(incl) or not incl:
                continue
            if not os.path.isdir(info.real_path(incl)):
                info.report(ERROR, "MISSING_BUILD_INCLUDE_PATH", path=incl)

    linter.add_init_hook(on_init)
    linter.add_command_hook("include_directories", on_include_directories)
    linter.add_final_hook(on_final)


def targets(linter):
    def on_init(info):
        info.target_outputs = {}
        info.target_links = {}

    def on_set_target_properties(info, cmd, args):
        opts, args = cmake_argparse(args, {"PROPERTIES": "p"})
        for target in args:
            if "OUTPUT_NAME" in opts["PROPERTIES"]:
                info.target_outputs[target] = opts["PROPERTIES"]["OUTPUT_NAME"]

    def on_add_target(info, cmd, args):
        if info.manifest.is_metapackage():
            info.report(ERROR, "INVALID_META_COMMAND", cmd=cmd)
            return
        if "catkin" not in info.find_packages:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)
        if "catkin_package" not in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="catkin_package")
#        if args[0] in info.targets:
#            TODO
        if not args[0] in info.target_outputs:
            info.target_outputs[args[0]] = args[0]
        if not args[0] in info.target_links:
            info.target_links[args[0]] = set()

    def on_target_link_libraries(info, cmd, args):
        if args[0] not in info.target_links:
            info.target_links[args[0]] = set()
        info.target_links[args[0]] |= set([d for d in args[1:] if not os.path.isabs(d)])

    def on_final(info):
        if (info.executables or info.libraries) and info.catkin_components and not os.path.normpath("/catkin-includes") in info.build_includes:
            info.report(ERROR, "MISSING_CATKIN_INCLUDE")
        if os.path.normpath("/catkin-includes") in info.build_includes:
            for pkg in info.catkin_components:
                if os.path.normpath("/%s-includes" % pkg) in info.build_includes:
                    info.report(WARNING, "DUPLICATE_BUILD_INCLUDE", pkg=pkg)

    linter.require(includes)
    linter.require(depends)
    linter.add_init_hook(on_init)
    linter.add_command_hook("set_target_properties", on_set_target_properties)
    linter.add_command_hook("add_executable", on_add_target)
    linter.add_command_hook("add_library", on_add_target)
    linter.add_command_hook("target_link_libraries", on_target_link_libraries)
    linter.add_final_hook(on_final)


def source_files(linter):
    def on_add_executable(info, cmd, args):
        if "IMPORTED" in args:
            return
        _, args = cmake_argparse(args, {"WIN32": "-", "MACOSX_BUNDLE": "-", "EXCLUDE_FROM_ALL": "-"})
        if not is_sorted(args[1:]):
            info.report(NOTICE, "UNSORTED_LIST", name="of source files")
        for source_file in args[1:]:
            if not source_file:
                continue
            source_file = info.package_path(source_file)
            if os.path.isabs(source_file):
                continue
            if not os.path.isfile(info.real_path(source_file)):
                info.report(ERROR, "MISSING_FILE", cmd=cmd, file=source_file)

    def on_add_library(info, cmd, args):
        if "IMPORTED" in args:
            return
        _, args = cmake_argparse(args, {"STATIC": "-", "SHARED": "-", "MODULE": "-", "EXCLUDE_FROM_ALL": "-"})
        if not is_sorted(args[1:]):
            info.report(NOTICE, "UNSORTED_LIST", name="of source files")
        for source_file in args[1:]:
            if not source_file:
                continue
            source_file = info.package_path(source_file)
            if os.path.isabs(source_file):
                continue
            if not os.path.isfile(info.real_path(source_file)):
                info.report(ERROR, "MISSING_FILE", cmd=cmd, file=source_file)

    linter.add_command_hook("add_executable", on_add_executable)
    linter.add_command_hook("add_library", on_add_library)


def link_directories(linter):
    def on_link_directories(info, cmd, args):
        externals = [p for p in args if not info.is_internal_path(p)]
        if externals:
            info.report(ERROR, "EXTERNAL_LINK_DIRECTORY")
        else:
            info.report(WARNING, "LINK_DIRECTORY")

    linter.add_command_hook("link_directories", on_link_directories)


def depends(linter):
    def on_init(info):
        info.required_packages = set()
        info.test_packages = set()
        info.catkin_components = set()
        info.checked_packages = set()

    def on_find_package(info, cmd, args):
        opts, args = cmake_argparse(args, {"REQUIRED": "-", "COMPONENTS": "*"})
        if "project" not in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="project")
        if args[0] in info.find_packages:
            info.report(ERROR, "DUPLICATE_FIND", pkg=args[0])
        if opts["REQUIRED"]:
            info.required_packages.add(args[0])
        if info.condition_is_true("CATKIN_ENABLE_TESTING"):
            info.test_packages.add(args[0])
        else:
            if args[0] in info.test_dep - info.build_dep - info.buildtool_dep:
                info.report(ERROR, "UNGUARDED_TEST_DEPEND", pkg=args[0])
        if args[0] != "catkin":
            return
        if "catkin_package" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="catkin_package", second_cmd=cmd)
        if not opts["REQUIRED"]:
            info.report(WARNING, "MISSING_REQUIRED", pkg="catkin")
            info.required_packages.add("catkin")
        if not is_sorted(opts["COMPONENTS"]):
            info.report(NOTICE, "UNSORTED_LIST", name="COMPONENTS")
        for pkg in opts["COMPONENTS"]:
            info.var["%s_INCLUDE_DIRS" % pkg] = "/%s-includes" % pkg
            info.var["%s_LIBRARIES" % pkg] = "/%s-libs/library.so" % pkg
            if pkg in info.find_packages:
                info.report(ERROR, "DUPLICATE_FIND", pkg=pkg)
            if not info.env.is_known_pkg(pkg):
                if info.env.ok:
                    info.report(ERROR, "UNKNOWN_PACKAGE", pkg=pkg)
            elif not info.env.is_catkin_pkg(pkg):
                if info.env.ok:
                    info.report(ERROR, "NO_CATKIN_COMPONENT", pkg=pkg)
        for pkg in args[1:]:
            if info.env.is_known_pkg(pkg):
                info.report(ERROR, "MISSING_COMPONENTS", pkg=pkg)
        info.find_packages |= set(opts["COMPONENTS"])
        info.required_packages |= set(opts["COMPONENTS"])
        info.catkin_components |= set(opts["COMPONENTS"])

    def on_if(info, cmd, args):
        for arg in args:
            if arg.endswith("_FOUND"):
                info.checked_packages.add(arg[0:-6])

    def on_final(info):
        for pkg in info.required_packages - info.build_dep - info.buildtool_dep - info.test_dep:
            if info.env.is_known_pkg(pkg):
                info.report(ERROR, "MISSING_DEPEND", pkg=pkg, type="build")
        for pkg in info.find_packages - info.required_packages - info.checked_packages:
            if info.env.is_catkin_pkg(pkg):
                info.report(ERROR, "MISSING_REQUIRED", pkg=pkg)
        for pkg in info.build_dep - (info.find_packages - info.test_packages):
            if info.env.is_catkin_pkg(pkg):
                info.report(ERROR if info.executables or info.libraries else WARNING, "UNCONFIGURED_BUILD_DEPEND", pkg=pkg)

    linter.require(manifest_depends)
    linter.add_init_hook(on_init)
    linter.add_command_hook("find_package", on_find_package)
    linter.add_command_hook("if", on_if)
    linter.add_final_hook(on_final)


def tests(linter):
    def on_test_cmd(info, cmd, args, dep=None):
        if not info.condition_is_true("CATKIN_ENABLE_TESTING"):
            info.report(ERROR, "UNGUARDED_TEST_CMD", cmd=cmd)
        if dep is None:
            return
        if dep not in info.test_dep | info.build_dep | info.exec_dep:
            info.report(ERROR, "MISSING_DEPEND", type="test", pkg=dep)

    linter.require(manifest_depends)
    linter.add_command_hook("catkin_download_test_data", on_test_cmd)
    linter.add_command_hook("catkin_add_gtest", partial(on_test_cmd, dep="rosunit"))
    linter.add_command_hook("catkin_add_nosetests", on_test_cmd)
    linter.add_command_hook("add_rostest", partial(on_test_cmd, dep="rostest"))
    linter.add_command_hook("add_rostest_gtest", partial(on_test_cmd, dep="rostest"))


def exports(linter):
    def on_init(info):
        info.export_packages = set()
        info.export_includes = set()
        info.export_libs = set()
        info.export_targets = set()

    def on_catkin_package(info, cmd, args):
        opts, args = cmake_argparse(args, {"INCLUDE_DIRS": "*", "LIBRARIES": "*", "DEPENDS": "*", "CATKIN_DEPENDS": "*", "CFG_EXTRAS": "*", "EXPORTED_TARGETS": "*"})
        for list_name in ["CATKIN_DEPENDS", "DEPENDS", "CFG_EXTRAS", "EXPORTED_TARGETS"]:
            if not is_sorted(opts[list_name]):
                info.report(NOTICE, "UNSORTED_LIST", name=list_name)
        for pkg in opts["CATKIN_DEPENDS"]:
            if not info.env.is_known_pkg(pkg):
                if info.env.ok:
                    info.report(ERROR, "UNKNOWN_PACKAGE", pkg=pkg)
            elif not info.env.is_catkin_pkg(pkg):
                if info.env.ok:
                    info.report(ERROR, "SYSTEM_AS_CATKIN_DEPEND", pkg=pkg)
        for pkg in opts["DEPENDS"]:
            if info.env.is_catkin_pkg(pkg):
                if info.env.ok:
                    info.report(ERROR, "CATKIN_AS_SYSTEM_DEPEND", pkg=pkg)
            elif pkg in info.pkg_modules:
                info.report(ERROR, "EXPORTED_PKG_CONFIG", pkg=pkg)
            elif pkg not in info.find_packages and not ("%s_INCLUDE_DIRS" % pkg in info.var and "%s_LIBRARIES" % pkg in info.var):
                info.report(ERROR, "UNCONFIGURED_SYSTEM_DEPEND", pkg=pkg)
        includes = [info.package_path(d) for d in opts["INCLUDE_DIRS"]]
        ext_includes = [d for d in includes if not info.is_internal_path(d)]
        if ext_includes:
            info.report(ERROR, "EXTERNAL_INCLUDE_PATH")
        info.export_libs |= set(opts["LIBRARIES"])
        info.export_includes |= set([d for d in includes if not os.path.isabs(d)])
        info.export_packages |= set(opts["CATKIN_DEPENDS"])
        info.export_targets |= set(opts["EXPORTED_TARGETS"])

    def on_final(info):
        for pkg in info.export_packages - info.export_dep:
            if info.env.is_known_pkg(pkg):
                if pkg == "message_runtime":
                    if pkg not in info.exec_dep:
                        info.report(ERROR, "MISSING_DEPEND", pkg=pkg, type="run" if info.manifest.package_format < 2 else "exec")
                else:
                    info.report(ERROR, "MISSING_DEPEND", pkg=pkg, type="run" if info.manifest.package_format < 2 else "build_export")
        for pkg in (info.find_packages & info.build_dep & info.export_dep) - info.export_packages:
            if re.search(r"_(msg|message)s?(_|$)", pkg) and info.env.is_catkin_pkg(pkg):
                info.report(WARNING, "SUGGEST_CATKIN_DEPEND", pkg=pkg)
        if info.export_includes and info.libraries and not info.export_libs:
            info.report(WARNING, "MISSING_EXPORT_LIB")
        if info.executables or info.libraries:
            for incl in info.export_includes - info.build_includes:
                info.report(WARNING, "MISSING_BUILD_INCLUDE", path=incl)
        for incl in info.export_includes:
            if not os.path.isdir(info.real_path(incl)):
                info.report(ERROR, "MISSING_EXPORT_INCLUDE_PATH", path=incl)
        includes = info.build_includes | info.export_includes
        for d1 in includes:
            if not os.path.isabs(d1):
                for d2 in includes:
                    if d1.startswith("%s%s" % (d2, os.path.sep)):
                        info.report(WARNING, "AMBIGUOUS_BUILD_INCLUDE", path=d1, parent_path=d2)
        for lib in info.export_libs:
            if lib not in info.targets:
                continue
            if info.target_outputs[lib] != lib:
                info.report(ERROR, "EXPORT_LIB_RENAMED", target=lib)
            if lib in info.executables:
                info.report(ERROR, "EXPORT_LIB_NOT_LIB", target=lib)

    linter.require(manifest_depends)
    linter.require(pkg_config)
    linter.require(includes)
    linter.require(targets)
    linter.add_init_hook(on_init)
    linter.add_command_hook("catkin_package", on_catkin_package)
    linter.add_final_hook(on_final)


def name_check(linter):
    def on_final(info):
        name_fragments = set(word_split(info.manifest.name))
        for target, output in iteritems(info.target_outputs):
            if "/" in output or "\\" in output:
                info.report(ERROR, "INVALID_TARGET_OUTPUT", target=target)
            tgl = target.lower()
            tnc = True
            for nf in name_fragments:
                if len(nf) < 2:
                    continue
                if nf in tgl:
                    tnc = False
                    break
            if tnc and target not in info.export_libs:
                info.report(NOTICE, "TARGET_NAME_COLLISION", target=target)
            if target in info.libraries and output.startswith("lib"):
                info.report(NOTICE, "REDUNDANT_LIB_PREFIX", output=output)

    linter.require(targets)
    linter.require(exports)
    linter.add_final_hook(on_final)


def pkg_config(linter):
    def on_init(info):
        info.pkg_modules = set()

    def on_pkg_check_modules(info, cmd, args):
        info.pkg_modules.add(args[0])

    linter.add_init_hook(on_init)
    linter.add_command_hook("pkg_check_modules", on_pkg_check_modules)


def installs(linter):
    def on_init(info):
        info.install_targets = set()
        info.install_includes = False
        info.install_files = set()

    def on_install(info, cmd, args):
        install_type = None
        opts, args = cmake_argparse(args, {"PROGRAMS": "*", "FILES": "*", "TARGETS": "*", "DIRECTORY": "?", "DESTINATION": "?", "ARCHIVE DESTINATION": "?", "LIBRARY DESTINATION": "?", "RUNTIME DESTINATION": "?"})
        if opts["PROGRAMS"]:
            install_type = "PROGRAMS"
        if opts["DIRECTORY"]:
            install_type = "DIRECTORY"
        if opts["FILES"]:
            install_type = "FILES"
            info.install_files |= set([os.path.normpath(os.path.join(opts["DESTINATION"], os.path.basename(f))) for f in opts["FILES"]])
        if opts["TARGETS"]:
            install_type = "TARGETS"
            info.install_targets |= set(opts["TARGETS"])
        if install_type is None:
            return
        if install_type != "DIRECTORY" and not is_sorted(opts[install_type]):
            info.report(NOTICE, "UNSORTED_LIST", name=install_type)
        for dest in ["DESTINATION", "ARCHIVE DESTINATION", "LIBRARY DESTINATION", "RUNTIME DESTINATION"]:
            if opts[dest] is None:
                continue
            if not info.is_catkin_target(opts[dest]):
                info.report(WARNING, "INSTALL_DESTINATION", type=install_type, dest=dest)
            if info.is_catkin_target(opts[dest], "include"):
                info.install_includes = True

    def on_final(info):
        for lib in info.export_libs:
            if lib not in info.targets:
                continue
            if lib not in info.install_targets:
                info.report(ERROR if "install" in info.commands else WARNING, "UNINSTALLED_EXPORT_LIB", target=lib)
        for tgt in info.executables - info.install_targets:
            if "test" in tgt.lower() or "example" in tgt.lower():
                continue
            info.report(WARNING, "MISSING_INSTALL_TARGET", target=tgt)
        if info.export_includes and not info.install_includes:
            info.report(ERROR if "install" in info.commands else WARNING, "MISSING_INSTALL_INCLUDE")
        for target, depends in iteritems(info.target_links):
            if target not in info.install_targets:
                continue
            for lib in depends:
                if lib not in info.libraries:
                    continue
                if lib not in info.install_targets:
                    info.report(ERROR, "UNINSTALLED_DEPEND", export_target=target, target=lib)
        for target in info.install_targets:
            if target not in info.libraries and target not in info.executables:
                info.report(ERROR, "UNDEFINED_INSTALL_TARGET", target=target)

    linter.require(targets)
    linter.require(exports)
    linter.add_init_hook(on_init)
    linter.add_command_hook("install", on_install)
    linter.add_final_hook(on_final)


def plugins(linter):
    def on_final(info):
        plugin_dep = set()
        for export in info.manifest.exports:
            if "plugin" in export.attributes:
                plugin = export.attributes["plugin"]
                plugin_dep.add(export.tagname)
                if not plugin.startswith("${prefix}/"):
                    info.report(ERROR, "PLUGIN_EXPORT_PREFIX", export=export.tagname)
                else:
                    if not os.path.isfile(info.real_path(plugin[10:])):
                        info.report(ERROR, "PLUGIN_MISSING_FILE", export=export.tagname, file=plugin)
                    if not os.path.normpath("/catkin-target/share/%s/%s" % (info.manifest.name, plugin[10:])) in info.install_files:
                        info.report(ERROR if "install" in info.commands else WARNING, "PLUGIN_MISSING_INSTALL", export=export.tagname, file=plugin[10:])
        for dep in plugin_dep - info.exec_dep:
            info.report(WARNING, "PLUGIN_DEPEND", export=dep, type="run" if info.manifest.package_format < 2 else "exec", pkg=dep)

    linter.require(manifest_depends)
    linter.require(installs)
    linter.add_final_hook(on_final)


def message_generation(linter):
    def on_init(info):
        info.declares_messages = False
        info.msg_dep = set()

    def on_add_msg_files(info, cmd, args):
        info.uses_catkin = True
        if info.manifest.is_metapackage():
            info.report(ERROR, "INVALID_META_COMMAND", cmd=cmd)
        info.declares_messages = True
        if "generate_messages" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="generate_messages", second_cmd=cmd)
        if "catkin" not in info.find_packages:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)
        if "catkin_package" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="catkin_package", second_cmd=cmd)
        opts, args = cmake_argparse(args, {"DIRECTORY": "?", "FILES": "*"})
        if not is_sorted(opts["FILES"]):
            info.report(NOTICE, "UNSORTED_LIST", name="FILES")

    def on_generate_msg(info, cmd, args):
        info.uses_catkin = True
        if info.manifest.is_metapackage():
            info.report(ERROR, "INVALID_META_COMMAND", cmd=cmd)
        if "catkin" not in info.find_packages:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)
        if "catkin_package" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="catkin_package", second_cmd=cmd)
        opts, args = cmake_argparse(args, {"DEPENDENCIES": "*"})
        if not is_sorted(opts["DEPENDENCIES"]):
            info.report(NOTICE, "UNSORTED_LIST", name="DEPENDENCIES")
        info.msg_dep |= set(opts["DEPENDENCIES"])

    def on_final(info):
        if info.manifest.is_metapackage():
            return
        for pkg in info.msg_dep - info.find_packages:
            info.report(ERROR, "UNCONFIGURED_MSG_DEPEND", pkg=pkg)
        for pkg in info.msg_dep - info.export_packages:
            info.report(ERROR, "MISSING_MSG_CATKIN", pkg=pkg)
        for pkg in info.msg_dep - info.build_dep:
            info.report(ERROR, "MISSING_MSG_DEPEND", pkg=pkg, type="build")
        for pkg in info.msg_dep - info.export_dep:
            info.report(ERROR, "MISSING_MSG_DEPEND", pkg=pkg, type="run" if info.manifest.package_format < 2 else "build_export")
        if info.declares_messages and "generate_messages" not in info.commands:
            info.report(ERROR, "MISSING_GENERATE_MSG")
        if not info.declares_messages and "generate_messages" in info.commands:
            info.report(WARNING, "UNUSED_GENERATE_MSG")
        if info.declares_messages and "message_generation" not in info.find_packages:
            info.report(ERROR, "UNCONFIGURED_BUILD_DEPEND", pkg="message_generation")
        if info.declares_messages and "message_runtime" not in info.export_packages:
            info.report(ERROR, "MISSING_CATKIN_DEPEND", type="run" if info.manifest.package_format < 2 else "build_export", pkg="message_runtime")

    linter.require(manifest_depends)
    linter.require(depends)
    linter.require(exports)
    linter.add_init_hook(on_init)
    linter.add_command_hook("add_message_files", on_add_msg_files)
    linter.add_command_hook("add_service_files", on_add_msg_files)
    linter.add_command_hook("add_action_files", on_add_msg_files)
    linter.add_command_hook("generate_messages", on_generate_msg)
    linter.add_final_hook(on_final)


def all(linter):
    linter.require(includes)
    linter.require(targets)
    linter.require(source_files)
    linter.require(link_directories)
    linter.require(depends)
    linter.require(tests)
    linter.require(exports)
    linter.require(name_check)
    linter.require(pkg_config)
    linter.require(installs)
    linter.require(plugins)
    linter.require(message_generation)
