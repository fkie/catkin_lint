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
from catkin_lint.linter import ERROR, WARNING, NOTICE
import catkin_lint.cmake as cmake
import catkin_lint.util as util
import catkin_lint.checks.manifest
from catkin_lint.util import iteritems
import re

def includes(linter):
    def on_init(info):
        info.build_includes = set([])
    def on_include_directories(info, cmd, args):
        _, args = cmake.argparse(args, { "AFTER" : "-", "BEFORE" : "-", "SYSTEM" : "-" })
        info.build_includes |= set([ os.path.normpath(os.path.join("/pkg-source", d)) for d in args])
    def on_final(info):
        for incl in info.build_includes:
            if not incl.startswith("/pkg-source"): continue
            if not os.path.isdir(os.path.join(info.path, incl[12:])):
                info.report (ERROR, "MISSING_BUILD_INCLUDE_PATH", path="./%s" % incl[12:])

    linter.add_init_hook(on_init)
    linter.add_command_hook("include_directories", on_include_directories)
    linter.add_final_hook(on_final)


def targets(linter):
    def on_init(info):
        info.target_outputs = {}
        info.target_links = {}
    def on_set_target_properties(info, cmd, args):
        opts, args = cmake.argparse(args, { "PROPERTIES" : "p" })
        for target in args:
            if "OUTPUT_NAME" in opts["PROPERTIES"]:
                info.target_outputs[target] = opts["PROPERTIES"]["OUTPUT_NAME"]
    def on_add_target(info, cmd, args):
        if info.manifest.is_metapackage():
            info.report(ERROR, "INVALID_META_COMMAND", cmd=cmd)
            return
        if not "catkin" in info.find_packages:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)
        if not "catkin_package" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="catkin_package")
#        if args[0] in info.targets:
#            TODO
        if not args[0] in info.target_outputs:
            info.target_outputs[args[0]] = args[0]
        if not args[0] in info.target_links:
            info.target_links[args[0]] = set([])
    def on_target_link_libraries(info, cmd, args):
        if not args[0] in info.target_links: info.target_links[args[0]] = set([])
        info.target_links[args[0]] |= set([ d for d in args[1:] if not d.startswith("/") ])
    def on_final(info):
        if (info.executables or info.libraries) and info.catkin_components and not "/catkin-includes" in info.build_includes:
            info.report(ERROR, "MISSING_CATKIN_INCLUDE")

    linter.require(includes)
    linter.require(depends)
    linter.add_init_hook(on_init)
    linter.add_command_hook("set_target_properties", on_set_target_properties)
    linter.add_command_hook("add_executable", on_add_target)
    linter.add_command_hook("add_library", on_add_target)
    linter.add_command_hook("target_link_libraries", on_target_link_libraries)
    linter.add_final_hook(on_final)


def link_directories(linter):
    def on_link_directories(info, cmd, args):
        externals = [ p for p in args if os.path.isabs(p) and not p.startswith("/pkg-") ]
        if externals:
            info.report (ERROR, "EXTERNAL_LINK_DIRECTORY")
        else:
            info.report (WARNING, "LINK_DIRECTORY")

    linter.add_command_hook("link_directories", on_link_directories)


def depends(linter):
    def on_init(info):
        info.required_packages = set([])
        info.catkin_components = set([])
    def on_find_package(info, cmd, args):
        opts, args = cmake.argparse(args, { "REQUIRED": "-", "COMPONENTS": "*" })
        if not "project" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="project")
        if args[0] in info.find_packages:
            info.report(ERROR, "DUPLICATE_FIND", pkg=args[0])
        if opts["REQUIRED"]: info.required_packages.add(args[0])
        if args[0] != "catkin": return
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
        info.catkin_components |= set(opts["COMPONENTS"])
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
                info.find_packages.add(pkg.upper())
    def on_final(info):
        for pkg in info.required_packages - info.build_dep - info.buildtool_dep:
            if info.env.is_known_pkg(pkg):
                info.report(ERROR, "MISSING_DEPEND", pkg=pkg, type="build")
        for pkg in info.build_dep - info.find_packages:
            if info.env.is_catkin_pkg(pkg):
                info.report(ERROR, "UNCONFIGURED_BUILD_DEPEND", pkg=pkg)

    linter.require(catkin_lint.checks.manifest.depends)
    linter.add_init_hook(on_init)
    linter.add_command_hook("find_package", on_find_package)
    linter.add_command_hook("include", on_include)
    linter.add_final_hook(on_final)


def exports(linter):
    def on_init(info):
        info.export_packages = set([])
        info.export_includes = set([])
        info.export_libs = set([])
        info.export_targets = set([])
    def on_catkin_package(info, cmd, args):
        opts, args = cmake.argparse(args, { "INCLUDE_DIRS": "*", "LIBRARIES": "*", "DEPENDS": "*", "CATKIN_DEPENDS": "*", "CFG_EXTRAS": "*", "EXPORTED_TARGETS": "*" })
        for pkg in opts["CATKIN_DEPENDS"]:
            if not info.env.is_catkin_pkg(pkg):
                info.report(ERROR, "SYSTEM_AS_CATKIN_DEPEND", pkg=pkg)
        for pkg in opts["DEPENDS"]:
            if info.env.is_catkin_pkg(pkg):
                info.report(ERROR, "CATKIN_AS_SYSTEM_DEPEND", pkg=pkg)
            elif not pkg in info.find_packages and not ("%s_INCLUDE_DIRS" % pkg in info.var and "%s_LIBRARIES" % pkg in info.var):
                info.report(ERROR, "UNCONFIGURED_SYSTEM_DEPEND", pkg=pkg)
        includes = [ os.path.join("/pkg-source", d) for d in opts["INCLUDE_DIRS"] ]
        ext_includes = [ d for d in includes if not d.startswith("/pkg-") ]
        if ext_includes:
            info.report(ERROR, "EXTERNAL_INCLUDE_PATH")
        info.export_libs |= set(opts["LIBRARIES"])
        info.export_includes |= set([os.path.normpath(d) for d in includes if d.startswith("/pkg-source") ])
        info.export_packages |= set(opts["CATKIN_DEPENDS"])
        info.export_targets |= set(opts["EXPORTED_TARGETS"])
    def on_final(info):
        for pkg in info.export_packages - info.run_dep:
            info.report(ERROR, "MISSING_DEPEND", pkg=pkg, type="run")
        for pkg in (info.find_packages & info.build_dep & info.run_dep) - info.export_packages:
            if re.search(r"_(msg|message)s?(_|$)", pkg) and info.env.is_catkin_pkg(pkg):
                info.report (WARNING, "SUGGEST_CATKIN_DEPEND", pkg=pkg)
        if info.export_includes and info.libraries and not info.export_libs:
            info.report(WARNING, "MISSING_EXPORT_LIB")
        for lib in info.export_libs:
            if not lib in info.targets: continue
            if info.target_outputs[lib] != lib:
                info.report(ERROR, "EXPORT_LIB_RENAMED", target=lib)
            if lib in info.executables:
                info.report(ERROR, "EXPORT_LIB_NOT_LIB", target=lib)

    linter.require(catkin_lint.checks.manifest.depends)
    linter.require(targets)
    linter.add_init_hook(on_init)
    linter.add_command_hook("catkin_package", on_catkin_package)
    linter.add_final_hook(on_final)


def name_check(linter):
    def on_final(info):
        name_fragments = set(util.word_split(info.manifest.name))
        for target, output in iteritems(info.target_outputs):
            if os.sep in output:
                info.report (ERROR, "INVALID_TARGET_OUTPUT", target=target)
            tgl = target.lower()
            tnc = True
            for nf in name_fragments:
                if len(nf) < 3: continue
                if nf in tgl:
                    tnc = False
                    break
            if tnc and not target in info.export_libs:
                info.report (NOTICE, "TARGET_NAME_COLLISION", target=target)
            if target in info.libraries and output.startswith("lib"):
                info.report (NOTICE, "REDUNDANT_LIB_PREFIX", output=output)

    linter.require(targets)
    linter.require(exports)
    linter.add_final_hook(on_final)


def installs(linter):
    def on_init(info):
        info.install_targets = set([])
        info.install_includes = False
        info.install_files = set([])
    def on_install(info, cmd, args):
        install_type = ""
        opts, args =  cmake.argparse(args, { "PROGRAMS" : "*", "FILES": "*", "TARGETS": "*", "DIRECTORY" : "?", "DESTINATION" : "?", "ARCHIVE DESTINATION": "?", "LIBRARY DESTINATION": "?", "RUNTIME DESTINATION": "?" })
        if opts["PROGRAMS"]:
            install_type = "PROGRAMS"
        if opts["DIRECTORY"]:
            install_type = "DIRECTORY"
        if opts["FILES"]:
            install_type = "FILES"
            info.install_files |= set([os.path.normpath(os.path.join(opts["DESTINATION"], f)) for f in opts["FILES"] ])
        if opts["TARGETS"]:
            install_type = "TARGETS"
            info.install_targets |= set(opts["TARGETS"])
        for dest in [ "DESTINATION", "ARCHIVE DESTINATION", "LIBRARY DESTINATION", "RUNTIME DESTINATION" ]:
            if not opts[dest]: continue
            if not opts[dest].startswith("/catkin-target/"):
                info.report(WARNING, "INSTALL_DESTINATION", type=install_type, dest=dest)
            if opts[dest].startswith("/catkin-target/include"):
                info.install_includes = True
    def on_final(info):
        for lib in info.export_libs:
            if not lib in info.targets: continue
            if not lib in info.install_targets:
                info.report(ERROR if "install" in info.commands else NOTICE, "UNINSTALLED_EXPORT_LIB", target=lib)
        for tgt in info.executables - info.install_targets:
            if "test" in tgt.lower(): continue
            info.report(WARNING if "install" in info.commands else NOTICE, "MISSING_INSTALL_TARGET", target=tgt)
        for incl in info.export_includes:
            if not os.path.isdir(os.path.join(info.path, incl[12:])):
                info.report (ERROR, "MISSING_EXPORT_INCLUDE_PATH", path="./%s" % incl[12:])
        if info.executables or info.libraries:
            for incl in info.export_includes - info.build_includes:
                info.report (WARNING, "MISSING_BUILD_INCLUDE", path="./%s" % incl[12:])
        if info.export_includes and not info.install_includes:
            info.report (ERROR if "install" in info.commands else NOTICE, "MISSING_INSTALL_INCLUDE")
        for target, depends in iteritems(info.target_links):
            if not target in info.install_targets: continue
            for lib in depends:
                if not lib in info.libraries: continue
                if not lib in info.install_targets:
                    info.report (ERROR, "UNINSTALLED_DEPEND", export_target=target, target=lib)

    linter.require(targets)
    linter.require(exports)
    linter.add_init_hook(on_init)
    linter.add_command_hook("install", on_install)
    linter.add_final_hook(on_final)


def plugins(linter):
    def on_final(info):
        plugin_dep = set([])
        for export in info.manifest.exports:
            if "plugin" in export.attributes:
                plugin = export.attributes["plugin"]
                plugin_dep.add(export.tagname)
                if not plugin.startswith("${prefix}/"):
                    info.report (ERROR, "PLUGIN_EXPORT_PREFIX", export=export.tagname)
                else:
                    if not os.path.isfile(os.path.join(info.path, plugin[10:])):
                        info.report (ERROR, "PLUGIN_MISSING_FILE", export=export.tagname, file=plugin)
                    if not os.path.normpath("/catkin-target/share/%s/%s" % (info.manifest.name, plugin[10:])) in info.install_files:
                        info.report (ERROR if "install" in info.commands else NOTICE, "PLUGIN_MISSING_INSTALL", export=export.tagname, file=plugin[10:])
        for dep in plugin_dep - info.run_dep:
            info.report (WARNING, "PLUGIN_DEPEND", export=dep, type="run", pkg=dep)

    linter.require(catkin_lint.checks.manifest.depends)
    linter.require(installs)
    linter.add_final_hook(on_final)


def message_generation(linter):
    def on_init(info):
        info.declares_messages = False
        info.msg_dep = set([])
    def on_add_msg_files(info, cmd, args):
        info.is_catkin = True
        if info.manifest.is_metapackage():
            info.report(ERROR, "INVALID_META_COMMAND", cmd=cmd)
        info.declares_messages = True
        if "generate_messages" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="generate_messages", second_cmd=cmd)
        if not "catkin" in info.find_packages:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)
        if "catkin_package" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="catkin_package", second_cmd=cmd)
    def on_generate_msg(info, cmd, args):
        info.is_catkin = True
        if info.manifest.is_metapackage():
            info.report(ERROR, "INVALID_META_COMMAND", cmd=cmd)
        if not "catkin" in info.find_packages:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)
        if "catkin_package" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="catkin_package", second_cmd=cmd)
        opts, args = cmake.argparse(args, { "DEPENDENCIES": "*" })
        info.msg_dep |= set(opts["DEPENDENCIES"])
    def on_final(info):
        if info.manifest.is_metapackage(): return
        for pkg in info.msg_dep - info.export_packages:
            info.report(ERROR, "MISSING_MSG_CATKIN", pkg=pkg)
        for pkg in info.msg_dep - info.build_dep:
            info.report (ERROR, "MISSING_MSG_DEPEND", pkg=pkg, type="build")
        for pkg in info.msg_dep - info.run_dep:
            info.report (ERROR, "MISSING_MSG_DEPEND", pkg=pkg, type="run")
        if info.declares_messages and not "generate_messages" in info.commands:
            info.report(ERROR, "MISSING_GENERATE_MSG")
        if not info.declares_messages and "generate_messages" in info.commands:
            info.report(WARNING, "UNUSED_GENERATE_MSG")
        if info.declares_messages and not "message_generation" in info.find_packages:
            info.report (ERROR, "UNCONFIGURED_BUILD_DEPEND", pkg="message_generation")
        if info.declares_messages and not "message_runtime" in info.export_packages:
            info.report(ERROR, "MISSING_CATKIN_DEPEND", pkg="message_runtime")

    linter.require(catkin_lint.checks.manifest.depends)
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
    linter.require(link_directories)
    linter.require(depends)
    linter.require(exports)
    linter.require(name_check)
    linter.require(installs)
    linter.require(plugins)
    linter.require(message_generation)
