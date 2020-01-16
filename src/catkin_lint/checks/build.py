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

import os
import posixpath
import re
import stat
from ..linter import ERROR, WARNING, NOTICE, PathConstants
from ..cmake import argparse as cmake_argparse
from ..util import iteritems, is_sorted
from .manifest import depends as manifest_depends
from functools import partial


def includes(linter):
    def on_init(info):
        info.build_includes = set()

    def on_include_directories(info, cmd, args):
        _, args = cmake_argparse(args, {"AFTER": "-", "BEFORE": "-", "SYSTEM": "-"})
        for incl in args:
            if not info.is_valid_path(incl):
                info.report(WARNING, "EXTERNAL_DIRECTORY", cmd=cmd, directory=info.report_path(incl))
            if not info.is_existing_path(incl, check=os.path.isdir):
                info.report(ERROR, "MISSING_DIRECTORY", cmd=cmd, directory=info.report_path(incl))
        includes = set([info.source_relative_path(d) for d in args])
        info.build_includes |= includes

    linter.add_init_hook(on_init)
    linter.add_command_hook("include_directories", on_include_directories)


def targets(linter):
    def on_init(info):
        info.target_outputs = {}
        info.target_links = {}
        info.target_order_violation = set()

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
        if args[0] in info.target_order_violation:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="target_link_libraries", second_cmd=cmd)
        if not args[0] in info.target_outputs:
            info.target_outputs[args[0]] = args[0]
        if not args[0] in info.target_links:
            info.target_links[args[0]] = set()

    def on_target_link_libraries(info, cmd, args):
        if args[0] in info.target_links:
            info.target_links[args[0]] |= set([d for d in args[1:] if not os.path.isabs(d)])
        else:
            info.target_order_violation.add(args[0])

    def on_final(info):
        catkin_include_path = info.find_package_path("catkin", "include")
        if (info.executables or info.libraries) and info.catkin_components and catkin_include_path not in info.build_includes:
            info.report(ERROR, "UNUSED_CATKIN_INCLUDE_DIRS", file_location=("CMakeLists.txt", 0))
        if catkin_include_path in info.build_includes:
            for pkg in info.catkin_components:
                if info.find_package_path(pkg, "include") in info.build_includes:
                    info.report(WARNING, "DUPLICATE_INCLUDE_PATH", pkg=pkg, file_location=("CMakeLists.txt", 0))

    linter.require(includes)
    linter.require(depends)
    linter.add_init_hook(on_init)
    linter.add_command_hook("set_target_properties", on_set_target_properties)
    linter.add_command_hook("add_executable", on_add_target)
    linter.add_command_hook("add_library", on_add_target)
    linter.add_command_hook("target_link_libraries", on_target_link_libraries)
    linter.add_final_hook(on_final)


def generated_files(linter):
    def on_configure_file(info, cmd, args):
        if not info.is_valid_path(args[0]):
            info.report(ERROR, "EXTERNAL_FILE", cmd=cmd, file=info.report_path(args[0]))
        elif not info.is_existing_path(args[0], check=os.path.isfile):
            info.report(ERROR, "MISSING_FILE", cmd=cmd, file=info.report_path(args[0]))
        info.generated_files.add(info.binary_relative_path(args[1]))

    def on_add_custom_command(info, cmd, args):
        opts, args = cmake_argparse(args, {"OUTPUT": "*", "COMMAND": "*", "ARGS": "*", "BYPRODUCTS": "*", "WORKING_DIRECTORY": "?", "MAIN_DEPENDENCY": "?", "DEPENDS": "*"})
        for f in opts["OUTPUT"] + opts["BYPRODUCTS"]:
            info.generated_files.add(info.binary_relative_path(f))

    def on_generate_export_header(info, cmd, args):
        opts, args = cmake_argparse(args, {"BASE_NAME": "?", "EXPORT_FILE_NAME": "?"})
        f = args[0] + "_export.h"
        if opts["BASE_NAME"]:
            f = opts["BASE_NAME"] + "_export.h"
        if opts["EXPORT_FILE_NAME"]:
            f = opts["EXPORT_FILE_NAME"]
        info.generated_files.add(info.binary_relative_path(f))

    def on_xacro_add_xacro_file(info, cmd, args):
        opts, args = cmake_argparse(args, {"INORDER": "-", "LEGACY": "-", "OUTPUT": "?", "REMAP": "*", "DEPENDS": "*"})
        if not info.is_existing_path(args[0], check=os.path.isfile):
            info.report(ERROR, "MISSING_FILE", cmd=cmd, file=info.report_path(args[0]))
        output_var = opts["OUTPUT"] or "XACRO_OUTPUT_FILE"
        if len(args) < 2:
            f = info.source_relative_path(args[0])
            if f.endswith(".xacro"):
                f = f[:-6]
            args.append(f)
        info.var[output_var] = posixpath.join(PathConstants.PACKAGE_BINARY, info.binary_relative_path(args[1]))
        info.generated_files.add(info.binary_relative_path(args[1]))

    def on_xacro_add_files(info, cmd, args):
        opts, args = cmake_argparse(args, {"INORDER": "-", "LEGACY": "-", "INSTALL": "-", "OUTPUT": "?", "TARGET": "?", "DESTINATION": "?", "REMAP": "*", "DEPENDS": "*"})
        for f in args:
            if not info.is_existing_path(f, check=os.path.isfile):
                info.report(ERROR, "MISSING_FILE", cmd=cmd, file=info.report_path(f))
            f_src = info.source_relative_path(f)
            if f_src.endswith(".xacro"):
                f_src = f_src[:-6]
                info.generated_files.add(info.binary_relative_path(f_src))

    linter.add_command_hook("configure_file", on_configure_file)
    linter.add_command_hook("generate_export_header", on_generate_export_header)
    linter.add_command_hook("add_custom_command", on_add_custom_command)
    linter.add_command_hook("xacro_add_xacro_file", on_xacro_add_xacro_file)
    linter.add_command_hook("xacro_add_files", on_xacro_add_files)


def source_files(linter):
    def on_add_executable(info, cmd, args):
        if "IMPORTED" in args or "ALIAS" in args:
            return
        _, args = cmake_argparse(args, {"WIN32": "-", "MACOSX_BUNDLE": "-", "EXCLUDE_FROM_ALL": "-"})
        if not is_sorted(args[1:]):
            info.report(NOTICE, "UNSORTED_LIST", name="of source files")
        for source_file in args[1:]:
            if not info.is_valid_path(source_file):
                info.report(ERROR, "EXTERNAL_FILE", cmd=cmd, file=info.report_path(source_file))
            if not info.is_existing_path(source_file, check=os.path.isfile):
                info.report(ERROR, "MISSING_FILE", cmd=cmd, file=info.report_path(source_file))

    def on_add_library(info, cmd, args):
        if "IMPORTED" in args or "ALIAS" in args or "INTERFACE" in args:
            return
        _, args = cmake_argparse(args, {"GLOBAL": "-", "STATIC": "-", "SHARED": "-", "MODULE": "-", "OBJECT": "-", "UNKNOWN": "-", "EXCLUDE_FROM_ALL": "-"})
        if not is_sorted(args[1:]):
            info.report(NOTICE, "UNSORTED_LIST", name="of source files")
        for source_file in args[1:]:
            if not info.is_valid_path(source_file):
                info.report(ERROR, "EXTERNAL_FILE", cmd=cmd, file=info.report_path(source_file))
            if not info.is_existing_path(source_file, check=os.path.isfile):
                info.report(ERROR, "MISSING_FILE", cmd=cmd, file=info.report_path(source_file))

    linter.require(generated_files)
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
        info.package_components = {}
        info.required_packages = set()
        info.test_packages = set()
        info.catkin_components = set()
        info.checked_packages = set()

    def on_find_package(info, cmd, args):
        opts, args = cmake_argparse(args, {"REQUIRED": "-", "COMPONENTS": "*", "OPTIONAL_COMPONENTS": "*"})
        this_components = opts["COMPONENTS"] if opts["COMPONENTS"] else args[1:]
        if "project" not in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="project")
        if args[0] in info.find_packages:
            previous_components = info.package_components.get(args[0], set())
            if previous_components - set(this_components):
                info.report(ERROR, "SHADOWED_FIND", pkg=args[0])
            elif previous_components == set(this_components):
                info.report(WARNING, "DUPLICATE_FIND", pkg=args[0])
        if opts["REQUIRED"]:
            info.required_packages.add(args[0])
        if info.condition_is_checked("CATKIN_ENABLE_TESTING"):
            info.test_packages.add(args[0])
        else:
            if args[0] in info.test_dep - info.build_dep - info.buildtool_dep:
                info.report(ERROR, "UNGUARDED_TEST_DEPEND", pkg=args[0])
        if args[0] not in info.package_components:
            info.package_components[args[0]] = set()
        info.package_components[args[0]] |= set(this_components)
        info.var["%s_EXTRAS_DIR" % args[0]] = info.find_package_path(args[0], "extras")
        if args[0] == "Qt5":
            for pkg in this_components:
                info.var["Qt5%s_INCLUDE_DIRS" % pkg] = info.find_package_path("Qt5%s" % pkg, "include")
                info.var["Qt5%s_LIBRARIES" % pkg] = posixpath.join(info.find_package_path("Qt5%s" % pkg, "lib"), "library.so")
                info.find_packages.add("Qt5%s" % pkg)
        if args[0] != "catkin":
            return
        if "catkin_package" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="catkin_package", second_cmd=cmd)
        if not opts["REQUIRED"]:
            info.report(WARNING, "MISSING_REQUIRED", pkg="catkin")
            info.required_packages.add("catkin")
        if not is_sorted(this_components):
            info.report(NOTICE, "UNSORTED_LIST", name="COMPONENTS")
        for pkg in this_components:
            info.var["%s_INCLUDE_DIRS" % pkg] = info.find_package_path(pkg, "include")
            info.var["%s_LIBRARIES" % pkg] = posixpath.join(info.find_package_path(pkg, "lib"), "library.so")
            info.var["%s_PACKAGE_PATH" % pkg] = posixpath.normpath(info.find_package_path(pkg, ""))
            info.var["%s_EXTRAS_DIR" % pkg] = info.find_package_path(pkg, "extras")
            if pkg in info.find_packages:
                previous_components = info.package_components.get(pkg, set())
                if previous_components:
                    info.report(ERROR, "SHADOWED_FIND", pkg=pkg)
                else:
                    info.report(WARNING, "DUPLICATE_FIND", pkg=pkg)
            if not info.env.is_known_pkg(pkg):
                if info.env.ok:
                    info.report(ERROR, "UNKNOWN_PACKAGE", pkg=pkg)
            elif not info.env.is_catkin_pkg(pkg):
                if info.env.ok:
                    info.report(ERROR, "NO_CATKIN_COMPONENT", pkg=pkg)
        for pkg in args[1:]:
            if info.env.is_known_pkg(pkg):
                info.report(NOTICE, "MISSING_COMPONENTS", pkg=pkg)
        info.find_packages |= set(this_components)
        info.required_packages |= set(this_components)
        info.catkin_components |= set(this_components)

    def on_if(info, cmd, args):
        for arg in args:
            if arg.endswith("_FOUND"):
                info.checked_packages.add(arg[0:-6])

    def on_final(info):
        for pkg in info.required_packages - info.build_dep - info.buildtool_dep - info.test_dep:
            if info.env.is_known_pkg(pkg):
                info.report(ERROR, "MISSING_DEPEND", pkg=pkg, type="build", file_location=("package.xml", 0))
        for pkg in info.find_packages - info.required_packages - info.checked_packages:
            if info.env.is_catkin_pkg(pkg):
                info.report(ERROR, "MISSING_REQUIRED", pkg=pkg, file_location=("CMakeLists.txt", 0))
        for pkg in info.build_dep - (info.find_packages - info.test_packages):
            if info.env.is_catkin_pkg(pkg):
                # Ignore pure Python packages (or at least packages that look like it), following the
                # Jack O'Quin heuristic from issue fkie/catkin_lint#22
                if pkg not in info.pkg_modules and (info.executables or info.libraries or "catkin_python_setup" not in info.commands):
                    info.report(ERROR, "UNCONFIGURED_BUILD_DEPEND", pkg=pkg, file_location=("CMakeLists.txt", 0))

    linter.require(manifest_depends)
    linter.require(pkg_config)
    linter.add_init_hook(on_init)
    linter.add_command_hook("find_package", on_find_package)
    linter.add_command_hook("if", on_if)
    linter.add_final_hook(on_final)


def tests(linter):
    def on_test_cmd(info, cmd, args, dep=None):
        if not info.condition_is_checked("CATKIN_ENABLE_TESTING"):
            info.report(ERROR, "UNGUARDED_TEST_CMD", cmd=cmd)
        if dep is None:
            return
        if dep not in info.test_dep | info.build_dep | info.exec_dep:
            info.report(ERROR, "MISSING_DEPEND", type="test", pkg=dep)

    linter.require(manifest_depends)
    linter.add_command_hook("catkin_download_test_data", on_test_cmd)
    linter.add_command_hook("catkin_add_gtest", on_test_cmd)
    linter.add_command_hook("catkin_add_nosetests", on_test_cmd)
    linter.add_command_hook("add_rostest", partial(on_test_cmd, dep="rostest"))
    linter.add_command_hook("add_rostest_gtest", partial(on_test_cmd, dep="rostest"))
    linter.add_command_hook("add_rostest_gmock", partial(on_test_cmd, dep="rostest"))


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
            elif pkg in info.pkg_modules_prefix:
                info.report(ERROR, "EXPORTED_PKG_CONFIG", pkg=pkg)
            elif pkg not in info.find_packages and not ("%s_INCLUDE_DIRS" % pkg in info.var and "%s_LIBRARIES" % pkg in info.var):
                info.report(ERROR, "UNCONFIGURED_SYSTEM_DEPEND", pkg=pkg)
        includes = [info.source_relative_path(d) for d in opts["INCLUDE_DIRS"]]
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
                        info.report(ERROR, "MISSING_DEPEND", pkg=pkg, type="run" if info.manifest.package_format < 2 else "exec", file_location=("package.xml", 0))
                else:
                    info.report(ERROR, "MISSING_DEPEND", pkg=pkg, type="run" if info.manifest.package_format < 2 else "build_export", file_location=("package.xml", 0))
        for pkg in (info.find_packages & info.build_dep & info.export_dep) - info.export_packages:
            if re.search(r"_(msg|message)s?(_|$)", pkg) and info.env.is_catkin_pkg(pkg):
                info.report(WARNING, "SUGGEST_CATKIN_DEPEND", pkg=pkg, file_location=info.location_of("catkin_package"))
        if info.export_includes and info.libraries and not info.export_libs:
            info.report(WARNING, "MISSING_EXPORT_LIB", file_location=info.location_of("catkin_package"))
        if info.executables or info.libraries:
            for incl in info.export_includes - info.build_includes:
                info.report(WARNING, "UNUSED_INCLUDE_PATH", path=incl, file_location=info.location_of("catkin_package"))
        for incl in info.export_includes:
            if not info.is_existing_path(incl, check=os.path.isdir, require_source_folder=True):
                info.report(ERROR, "MISSING_INCLUDE_PATH", path=incl, file_location=info.location_of("catkin_package"))
        includes = info.build_includes | info.export_includes
        for d1 in includes:
            if not posixpath.isabs(d1):
                for d2 in includes:
                    if d1.startswith("%s/" % d2):
                        info.report(WARNING, "AMBIGUOUS_INCLUDE_PATH", path=info.report_path(d1), parent_path=info.report_path(d2))
        for lib in info.export_libs:
            if lib in info.targets:
                if info.target_outputs[lib] != lib:
                    info.report(ERROR, "EXPORT_LIB_RENAMED", target=lib, file_location=info.location_of("catkin_package"))
                if lib in info.executables:
                    info.report(ERROR, "EXPORT_LIB_NOT_LIB", target=lib, file_location=info.location_of("catkin_package"))

    linter.require(manifest_depends)
    linter.require(pkg_config)
    linter.require(includes)
    linter.require(targets)
    linter.add_init_hook(on_init)
    linter.add_command_hook("catkin_package", on_catkin_package)
    linter.add_final_hook(on_final)


def name_check(linter):
    def on_final(info):
        for target, output in iteritems(info.target_outputs):
            if "/" in output or "\\" in output:
                info.report(ERROR, "INVALID_TARGET_OUTPUT", target=target)
            if target in info.libraries and output.startswith("lib"):
                info.report(NOTICE, "REDUNDANT_LIB_PREFIX", output=output)

    linter.require(targets)
    linter.require(exports)
    linter.add_final_hook(on_final)


def pkg_config(linter):
    def on_init(info):
        info.pkg_modules = set()
        info.pkg_modules_prefix = set()

    def on_pkg_check_modules(info, cmd, args):
        opts, args = cmake_argparse(args, {"REQUIRED": "-", "QUIET": "-", "NO_CMAKE_PATH": "-", "NO_CMAKE_ENVIRONMENT_PATH": "-", "IMPORTED_TARGET": "-"})
        info.pkg_modules_prefix.add(args[0])
        for pkg in args[1:]:
            if "=" in pkg:
                pkg = pkg.split("=")[0].rstrip("<>=")
            info.pkg_modules.add(pkg)
            if info.env.is_catkin_pkg(pkg):
                info.report(ERROR, "MISCONFIGURED_CATKIN_PACKAGE", pkg=pkg)

    linter.add_init_hook(on_init)
    linter.add_command_hook("pkg_check_modules", on_pkg_check_modules)


def installs(linter):
    def on_init(info):
        info.install_targets = set()
        info.install_programs = set()
        info.install_includes = False
        info.install_files = set()

    def on_catkin_install_python(info, cmd, args):
        opts, args = cmake_argparse(args, {"PROGRAMS": "+", "DESTINATION": "!"})
        for f in opts["PROGRAMS"]:
            if f:
                if not info.is_valid_path(f):
                    info.report(ERROR, "EXTERNAL_FILE", cmd=cmd, file=info.report_path(f))
                if info.is_existing_path(f, check=os.path.isfile):
                    real_f = info.real_path(info.source_relative_path(f))
                    if os.path.isfile(real_f):
                        with open(real_f, "r") as fd:
                            shebang = fd.readline()
                            if not shebang.startswith("#!") or "python" not in shebang:
                                info.report(ERROR, "MISSING_SHEBANG", file=info.report_path(f), interpreter="python")
                else:
                    info.report(ERROR, "MISSING_FILE", cmd=cmd, file=info.report_path(f))
                info.install_programs.add(info.source_relative_path(f))
        if not info.is_catkin_bin_install_destination(opts["DESTINATION"]):
            info.report(WARNING, "WRONG_BIN_INSTALL_DESTINATION")

    def on_install(info, cmd, args):
        install_type = None
        opts, args = cmake_argparse(args, {"PROGRAMS": "*", "FILES": "*", "TARGETS": "*", "DIRECTORY": "*", "DESTINATION": "?", "ARCHIVE DESTINATION": "?", "LIBRARY DESTINATION": "?", "RUNTIME DESTINATION": "?", "USE_SOURCE_PERMISSIONS": "-"})
        if opts["PROGRAMS"]:
            install_type = "PROGRAMS"
            for f in opts["PROGRAMS"]:
                if f:
                    if not info.is_valid_path(f):
                        info.report(WARNING, "EXTERNAL_FILE", cmd=cmd, file=info.report_path(f))
                    if not info.is_existing_path(f, check=os.path.isfile):
                        info.report(ERROR, "MISSING_FILE", cmd=cmd, file=info.report_path(f))
                    info.install_programs.add(info.source_relative_path(f))
        if opts["DIRECTORY"]:
            install_type = "DIRECTORY"
            for d in opts["DIRECTORY"]:
                if d:
                    if not info.is_valid_path(d):
                        info.report(WARNING, "EXTERNAL_DIRECTORY", cmd=cmd, directory=info.report_path(d))
                    if info.is_existing_path(d, check=os.path.isdir):
                        real_d = info.real_path(info.source_relative_path(d))
                        if os.path.isdir(real_d):
                            if opts["USE_SOURCE_PERMISSIONS"]:
                                for dirpath, _, filenames in os.walk(real_d, topdown=True):
                                    for filename in filenames:
                                        real_filename = os.path.join(dirpath, filename)
                                        pkg_filename = info.source_relative_path(real_filename[len(info.path) + 1:])
                                        mode = os.stat(real_filename).st_mode
                                        if mode & stat.S_IXUSR:
                                            info.install_programs.add(pkg_filename)
                    else:
                        info.report(ERROR, "MISSING_DIRECTORY", cmd=cmd, directory=info.report_path(d))
        if opts["FILES"]:
            install_type = "FILES"
            for f in opts["FILES"]:
                if f:
                    if not info.is_valid_path(f):
                        info.report(WARNING, "EXTERNAL_FILE", cmd=cmd, file=info.report_path(f))
                    if not info.is_existing_path(f, check=os.path.isfile):
                        info.report(ERROR, "MISSING_FILE", cmd=cmd, file=info.report_path(f))
            info.install_files |= set([posixpath.normpath(posixpath.join(PathConstants.CATKIN_INSTALL, opts["DESTINATION"], posixpath.basename(f))) for f in opts["FILES"]])
        if opts["TARGETS"]:
            install_type = "TARGETS"
            info.install_targets |= set(opts["TARGETS"])
        if install_type is None:
            return
        if not is_sorted(opts[install_type]):
            info.report(NOTICE, "UNSORTED_LIST", name=install_type)
        for dest in ["DESTINATION", "ARCHIVE DESTINATION", "LIBRARY DESTINATION", "RUNTIME DESTINATION"]:
            if opts[dest] is not None:
                if install_type == "PROGRAMS" and not info.is_catkin_bin_install_destination(opts[dest]):
                    info.report(WARNING, "WRONG_BIN_INSTALL_DESTINATION")
                elif not info.is_catkin_install_destination(opts[dest]):
                    info.report(WARNING, "WRONG_INSTALL_DESTINATION", type=install_type, dest=dest)
                if info.is_catkin_install_destination(opts[dest], "include"):
                    info.install_includes = True

    def on_final(info):
        for lib in info.export_libs:
            if lib in info.targets and lib not in info.install_targets:
                info.report(ERROR if "install" in info.commands else WARNING, "UNINSTALLED_EXPORT_LIB", target=lib, file_location=info.location_of("catkin_package"))
        for tgt in info.executables - info.install_targets:
            if "test" not in tgt.lower() and "example" not in tgt.lower():
                info.report(WARNING, "UNINSTALLED_TARGET", target=tgt, file_location=("CMakeLists.txt", 0))
        if info.export_includes and not info.install_includes:
            info.report(ERROR if "install" in info.commands else WARNING, "UNINSTALLED_INCLUDE_PATH", file_location=info.location_of("catkin_package"))
        for target, depends in iteritems(info.target_links):
            if target in info.install_targets:
                for lib in depends:
                    if lib in info.libraries and lib not in info.install_targets and lib not in info.static_libraries and lib not in info.interface_libraries:
                        info.report(ERROR, "UNINSTALLED_DEPEND", export_target=target, target=lib, file_location=info.location_of("catkin_package"))
        for target in info.install_targets:
            if target not in info.targets:
                info.report(ERROR, "UNDEFINED_TARGET", target=target, file_location=("CMakeLists.txt", 0))

    linter.require(targets)
    linter.require(exports)
    linter.require(generated_files)
    linter.add_init_hook(on_init)
    linter.add_command_hook("catkin_install_python", on_catkin_install_python)
    linter.add_command_hook("install", on_install)
    linter.add_final_hook(on_final)


def plugins(linter):
    PLUGIN_WHITELIST = ["roswtf"]

    def on_final(info):
        plugin_dep = set()
        for export in info.manifest.exports:
            if "plugin" in export.attributes and export.tagname not in PLUGIN_WHITELIST:
                plugin = export.attributes["plugin"]
                if export.tagname != info.manifest.name:
                    plugin_dep.add(export.tagname)
                if not plugin.startswith("${prefix}/"):
                    info.report(ERROR, "PLUGIN_EXPORT_PREFIX", export=export.tagname, file_location=("package.xml", 0))
                elif not os.path.isfile(info.real_path(plugin[10:])):
                    info.report(ERROR, "MISSING_PLUGIN", export=export.tagname, file=plugin, file_location=("package.xml", 0))
                elif posixpath.normpath("%s/share/%s/%s" % (PathConstants.CATKIN_INSTALL, info.manifest.name, plugin[10:])) not in info.install_files:
                    info.report(ERROR if "install" in info.commands else WARNING, "UNINSTALLED_PLUGIN", export=export.tagname, file=plugin[10:], file_location=("CMakeLists.txt", 0))
        for dep in plugin_dep - info.exec_dep:
            info.report(WARNING, "PLUGIN_DEPEND", export=dep, type="run" if info.manifest.package_format < 2 else "exec", pkg=dep, file_location=("package.xml", 0))

    linter.require(manifest_depends)
    linter.require(installs)
    linter.add_final_hook(on_final)


def dynamic_reconfigure(linter):
    def on_init(info):
        info.dynamic_reconfigure_files = set()
        info.dynamic_reconfigure_loc = None

    def on_generate_dynamic_reconfigure_options(info, cmd, args):
        for f in args:
            if f:
                real_f = info.real_path(info.source_relative_path(f))
                if not info.is_valid_path(f):
                    info.report(ERROR, "EXTERNAL_FILE", cmd=cmd, file=info.report_path(f))
                elif os.path.isfile(real_f):
                    mode = os.stat(real_f).st_mode
                    if not mode & stat.S_IXUSR:
                        info.report(ERROR, "SCRIPT_NOT_EXECUTABLE", script=info.report_path(f))
                else:
                    info.report(ERROR, "MISSING_FILE", cmd=cmd, file=info.report_path(f))
                info.dynamic_reconfigure_files.add(info.source_relative_path(f))

    def on_final(info):
        if "generate_dynamic_reconfigure_options" in info.commands and "dynamic_reconfigure" not in info.find_packages:
            info.report(ERROR, "UNCONFIGURED_BUILD_DEPEND", pkg="dynamic_reconfigure", file_location=info.location_of("generate_dynamic_reconfigure_options"))

    linter.require(depends)
    linter.add_init_hook(on_init)
    linter.add_command_hook("generate_dynamic_reconfigure_options", on_generate_dynamic_reconfigure_options)
    linter.add_final_hook(on_final)


def scripts(linter):
    def is_installed(info, pkg_filename):
        return pkg_filename in info.install_programs or pkg_filename in info.dynamic_reconfigure_files

    def on_final(info):
        for dirpath, dirnames, filenames in os.walk(info.path, topdown=True):
            for filename in filenames:
                if "test" not in filename.lower() and "example" not in filename.lower():
                    full_filename = os.path.join(dirpath, filename)
                    pkg_filename = info.source_relative_path(full_filename[len(info.path) + 1:])
                    mode = os.stat(full_filename).st_mode
                    if mode & stat.S_IXUSR and not is_installed(info, pkg_filename):
                        info.report(WARNING, "UNINSTALLED_SCRIPT", script=pkg_filename)
            dirnames[:] = [d for d in dirnames if not d.startswith(".") and "test" not in d and "build" not in d and "example" not in d]

    linter.require(installs)
    linter.require(dynamic_reconfigure)
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
        opts, args = cmake_argparse(args, {"DIRECTORY": "?", "FILES": "*", "PACKAGE": "?", "BASE_DIR": "?", "NOINSTALL": "-"})
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
        opts, args = cmake_argparse(args, {"DEPENDENCIES": "*", "LANGS": "*"})
        if not is_sorted(opts["DEPENDENCIES"]):
            info.report(NOTICE, "UNSORTED_LIST", name="DEPENDENCIES")
        info.msg_dep |= set(opts["DEPENDENCIES"])

    def on_final(info):
        if info.manifest.is_metapackage():
            return
        for pkg in info.msg_dep - info.find_packages:
            info.report(ERROR, "UNCONFIGURED_MSG_DEPEND", pkg=pkg, file_location=("CMakeLists.txt", 0))
        for pkg in info.msg_dep - info.export_packages:
            info.report(ERROR, "MISSING_CATKIN_DEPEND", pkg=pkg, type="run" if info.manifest.package_format < 2 else "build_export", file_location=info.location_of("catkin_package"))
        for pkg in info.msg_dep - info.build_dep:
            info.report(ERROR, "MISSING_DEPEND", pkg=pkg, type="build", file_location=("package.xml", 0))
        for pkg in info.msg_dep - info.export_dep:
            info.report(ERROR, "MISSING_DEPEND", pkg=pkg, type="run" if info.manifest.package_format < 2 else "build_export", file_location=("package.xml", 0))
        if info.declares_messages and "generate_messages" not in info.commands:
            info.report(ERROR, "MISSING_GENERATE_MSG", file_location=("CMakeLists.txt", 0))
        if not info.declares_messages and "generate_messages" in info.commands:
            info.report(WARNING, "UNUSED_GENERATE_MSG", file_location=info.location_of("generate_messages"))
        if info.declares_messages and "message_generation" not in info.find_packages:
            info.report(ERROR, "UNCONFIGURED_BUILD_DEPEND", pkg="message_generation")
        if info.declares_messages and "message_runtime" not in info.export_packages:
            info.report(ERROR, "MISSING_CATKIN_DEPEND", type="run" if info.manifest.package_format < 2 else "build_export", pkg="message_runtime", file_location=info.location_of("catkin_package"))

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
    linter.require(generated_files)
    linter.require(source_files)
    linter.require(link_directories)
    linter.require(depends)
    linter.require(tests)
    linter.require(exports)
    linter.require(name_check)
    linter.require(pkg_config)
    linter.require(installs)
    linter.require(dynamic_reconfigure)
    linter.require(plugins)
    linter.require(scripts)
    linter.require(message_generation)
