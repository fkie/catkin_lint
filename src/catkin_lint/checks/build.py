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
import catkin_lint.cmake as cmake
import catkin_lint.util as util
from catkin_lint.main import ERROR, WARNING, NOTICE
from catkin_lint.util import iteritems

def targets(linter):
    def on_init(info):
        info.export_includes = set([])
        info.export_libs = set([])
        info.target_outputs = {}
        info.target_links = {}
        info.install_targets = set([])
        info.install_includes = False
        info.build_includes = set([])
    def on_catkin_package(info, cmd, args):
        opts, args = cmake.argparse(args, { "INCLUDE_DIRS": "*", "LIBRARIES": "*", "DEPENDS": "*", "CATKIN_DEPENDS": "*", "CFG_EXTRAS": "*", "EXPORTED_TARGETS": "*" })
        includes = [ os.path.join("/pkg-source", d) for d in opts["INCLUDE_DIRS"] ]
        ext_includes = [ d for d in includes if not d.startswith("/pkg-") ]
        if ext_includes:
            info.report(ERROR, "EXTERNAL_INCLUDE_PATH")
        info.export_libs |= set(opts["LIBRARIES"])
        info.export_includes |= set([os.path.normpath(d) for d in includes if d.startswith("/pkg-source") ])
    def on_set_target_properties(info, cmd, args):
        opts, args = cmake.argparse(args, { "PROPERTIES" : "p" })
        for target in args:
            if "OUTPUT_NAME" in opts["PROPERTIES"]:
                info.target_outputs[target] = opts["PROPERTIES"]["OUTPUT_NAME"]
    def on_include_directories(info, cmd, args):
        _, args = cmake.argparse(args, { "AFTER" : "-", "BEFORE" : "-", "SYSTEM" : "-" })
        info.build_includes |= set([ os.path.normpath(os.path.join("/pkg-source", d)) for d in args])
    def on_link_directories(info, cmd, args):
        externals = [ p for p in args if os.path.isabs(p) and not p.startswith("/pkg-") ]
        if externals:
            info.report (ERROR, "EXTERNAL_LINK_DIRECTORY")
        else:
            info.report (WARNING, "LINK_DIRECTORY")
    def on_add_target(info, cmd, args):
        if info.manifest.is_metapackage():
            info.report(ERROR, "INVALID_META_COMMAND", cmd=cmd)
            return
        if not "catkin" in info.find_packages:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="find_package")
        if not "catkin_package" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd=cmd, second_cmd="catkin_package")
        if not args[0] in info.target_outputs:
            info.target_outputs[args[0]] = args[0]
        if not args[0] in info.target_links:
            info.target_links[args[0]] = set([])
    def on_target_link_libraries(info, cmd, args):
        if not args[0] in info.target_links: info.target_links[args[0]] = set([])
        info.target_links[args[0]] |= set([ d for d in args[1:] if not d.startswith("/") ])
    def on_install(info, cmd, args):
        install_type = ""
        opts, args =  cmake.argparse(args, { "PROGRAMS" : "*", "FILES": "*", "TARGETS": "*", "DIRECTORY" : "?", "DESTINATION" : "?", "ARCHIVE DESTINATION": "?", "LIBRARY DESTINATION": "?", "RUNTIME DESTINATION": "?" })
        if opts["PROGRAMS"]:
            install_type = "PROGRAMS"
        if opts["DIRECTORY"]:
            install_type = "DIRECTORY"
        if opts["FILES"]:
            install_type = "FILES"
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
        if info.export_includes and info.libraries and not info.export_libs:
            info.report(WARNING, "MISSING_EXPORT_LIB")
        for lib in info.export_libs:
            if not lib in info.targets: continue
            if info.target_outputs[lib] != lib:
                info.report(ERROR, "EXPORT_LIB_RENAMED", target=lib)
            if not lib in info.install_targets:
                info.report(ERROR if "install" in info.commands else NOTICE, "UNINSTALLED_EXPORT_LIB", target=lib)
            if lib in info.executables:
                info.report(ERROR, "EXPORT_LIB_NOT_LIB", target=lib)
        for tgt in info.executables - info.install_targets:
            if "test" in tgt.lower(): continue
            info.report(WARNING if "install" in info.commands else NOTICE, "MISSING_INSTALL_TARGET", target=tgt)
        for incl in info.export_includes:
            if not os.path.isdir(os.path.join(info.path, incl[12:])):
                info.report (ERROR, "MISSING_EXPORT_INCLUDE_PATH", path="./%s" % incl[12:])
        if info.executables or info.libraries:
            for incl in info.export_includes - info.build_includes:
                info.report (WARNING, "MISSING_BUILD_INCLUDE", path="./%s" % incl[12:])
        for incl in info.build_includes:
            if not incl.startswith("/pkg-source"): continue
            if not os.path.isdir(os.path.join(info.path, incl[12:])):
                info.report (ERROR, "MISSING_BUILD_INCLUDE_PATH", path="./%s" % incl[12:])
        name_fragments = set(util.word_split(info.manifest.name))
        if info.export_includes and not info.install_includes:
            info.report (ERROR if "install" in info.commands else NOTICE, "MISSING_INSTALL_INCLUDE")
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
        for target, depends in iteritems(info.target_links):
            if not target in info.install_targets: continue
            for lib in depends:
                if not lib in info.libraries: continue
                if not lib in info.install_targets:
                    info.report (ERROR, "UNINSTALLED_DEPEND", export_target=target, target=lib)

    linter.add_init_hook(on_init)
    linter.add_command_hook("catkin_package", on_catkin_package)
    linter.add_command_hook("set_target_properties", on_set_target_properties)
    linter.add_command_hook("add_executable", on_add_target)
    linter.add_command_hook("add_library", on_add_target)
    linter.add_command_hook("target_link_libraries", on_target_link_libraries)
    linter.add_command_hook("include_directories", on_include_directories)
    linter.add_command_hook("link_directories", on_link_directories)
    linter.add_command_hook("install", on_install)
    linter.add_final_hook(on_final)


def plugins(linter):
    def on_init(info):
        info.install_files = set([])
    def on_install(info, cmd, args):
        opts, args =  cmake.argparse(args, { "PROGRAMS" : "*", "FILES": "*", "TARGETS": "*", "DIRECTORY" : "?", "DESTINATION" : "?", "ARCHIVE DESTINATION": "?", "LIBRARY DESTINATION": "?", "RUNTIME DESTINATION": "?" })
        if opts["FILES"]:
            info.install_files |= set([os.path.normpath(os.path.join(opts["DESTINATION"], f)) for f in opts["FILES"] ])
    def on_final(info):
        plugin_dep = set([])
        pkg_run_dep = set([dep.name for dep in info.manifest.run_depends])
        for export in info.manifest.exports:
            if "plugin" in export.attributes.keys():
                plugin = export.attributes["plugin"]
                plugin_dep.add(export.tagname)
                if not plugin.startswith("${prefix}/"):
                    info.report (ERROR, "PLUGIN_EXPORT_PREFIX", export=export.tagname)
                else:
                    if not os.path.exists(os.path.join(info.path, plugin[10:])):
                        info.report (ERROR, "PLUGIN_MISSING_FILE", export=export.tagname, file=plugin)
                    if not os.path.normpath("/catkin-target/share/%s/%s" % (info.manifest.name, plugin[10:])) in info.install_files:
                        info.report (ERROR if "install" in info.commands else NOTICE, "PLUGIN_MISSING_INSTALL", export=export.tagname, file=plugin[10:])
        for dep in plugin_dep - pkg_run_dep:
            info.report (WARNING, "PLUGIN_DEPEND", export=dep, type="run", pkg=dep)

    linter.add_init_hook(on_init)
    linter.add_command_hook("install", on_install)
    linter.add_final_hook(on_final)
