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
import sys
from functools import total_ordering
from catkin_pkg.packages import find_packages
import catkin_lint.cmake as cmake
import catkin_lint.diagnostics as diagnostics
from .util import iteritems

ERROR = 0
WARNING = 1
NOTICE = 2

@total_ordering
class Message:

    def __init__(self, package, file, line, level, id, text, description):
        self.package = package
        self.file = file
        self.line = line
        self.level = level
        self.id = id
        self.text = text
        self.description = description

    def __eq__(self, other):
        return (self.package, self.level, self.file, self.line, self.id) == (other.package, other.level, other.file, other.line, other.id)

    def __lt__(self, other):
        return (self.package, self.level, self.file, self.line, self.id) < (other.package, other.level, other.file, other.line, other.id)

class LintInfo(object):

    def __init__(self, env):
        self.env = env
        self.path = None
        self.manifest = None
        self.file = ""
        self.line = 0
        self.commands = set([])
        self.find_packages = set([])
        self.targets = set([])
        self.executables = set([])
        self.libraries = set([])
        self.var = {}
        self.messages = []

    def report(self, level, msg_id, **kwargs):
        id, text, description = diagnostics.msg(msg_id, **kwargs)
        self.messages.append(Message(
            package=self.manifest.name, 
            file=self.file, 
            line=self.line,
            level=level,
            id=id,
            text=text,
            description=description
        ))

class CMakeLinter(object):
    def __init__(self, env):
        self.env = env
        self.messages = []
        self._cmd_hooks = {}
        self._init_hooks = []
        self._final_hooks = []
        self._added_checks = set([])
        self._catch_circular_deps = set([])

    def require(self, check):
        if check in self._catch_circular_deps:
            raise RuntimeError("Circular dependency detected")
        if check in self._added_checks: return
        self._added_checks.add(check)
        self._catch_circular_deps.add(check)
        check(self)
        self._catch_circular_deps.remove(check)

    def add_init_hook(self, cb):
        self._init_hooks.append(cb)

    def add_command_hook(self, cmd, cb):
        if cmd not in self._cmd_hooks:
            self._cmd_hooks[cmd] = [ cb ]
        else:
            self._cmd_hooks[cmd].append(cb)

    def add_final_hook(self, cb):
        self._final_hooks.append(cb)

    def _read_file(self, filename):
        f = open(filename, "r")
        content = f.read()
        f.close()
        return content

    def _parse_file(self, info, filename):
        save_file = info.file
        save_line = info.line
        try:
            info.file = os.path.relpath(filename, info.path)
            info.line = 0
            content = self._read_file(filename)
            in_macro = False
            in_function = False
            for cmd, args, line in cmake.parse(content, info.var):
                info.line = line
                if in_macro:
                    if cmd == "endmacro": in_macro = False
                    continue
                else:
                    if cmd == "macro":
                        in_macro = True
                        info.report(NOTICE, "UNSUPPORTED_CMD", cmd="macro")
                        continue
                if in_function:
                    if cmd == "endfunction": in_function = False
                    continue
                else:
                    if cmd == "function":
                        in_function = True
                        info.report(NOTICE, "UNSUPPORTED_CMD", cmd="function")
                        continue
                if cmd in self._cmd_hooks:
                    for cb in self._cmd_hooks[cmd]:
                        cb(info, cmd, args)
                info.commands.add(cmd)
                if cmd == "set":
                    info.var[args[0]] = ';'.join(args[1:])
                if cmd == "unset":
                    info.var[args[0]] = ""
                if cmd == "include":
                    pass
                if cmd == "project":
                    info.var["PROJECT_NAME"] = args[0]
                if cmd == "find_package":
                    info.var["%s_INCLUDE_DIRS" % args[0]] = "/%s-includes" % args[0]
                    info.var["%s_INCLUDE_DIRS" % args[0].upper()] = "/%s-includes" % args[0]
                    info.var["%s_LIBRARIES" % args[0]] = "/%s-libs/library.so" % args[0]
                    info.var["%s_LIBRARIES" % args[0].upper()] = "/%s-libs/library.so" % args[0]
                    info.find_packages.add(args[0])
                if cmd == "add_executable":
                    info.targets.add(args[0])
                    if not "IMPORTED" in args: info.executables.add(args[0])
                if cmd == "add_library":
                    info.targets.add(args[0])
                    if not "IMPORTED" in args: info.libraries.add(args[0])
                if cmd == "add_custom_target":
                    info.targets.add(args[0])
                if cmd == "find_path":
                    info.var[args[0]] = "/find-path"
                if cmd == "find_library":
                    info.var[args[0]] = "/find-libs/library.so"
        except cmake.SyntaxError as err:
            raise cmake.SyntaxError ("%s: %s" % (info.file, str(err)))
        finally:
            info.file = save_file
            info.line = save_line

    def lint(self, path, manifest):
        info = LintInfo(self.env)
        info.path = path
        info.manifest = manifest
        info.var = {
          "CMAKE_CURRENT_SOURCE_DIR" : "/pkg-source",
          "CMAKE_CURRENT_BINARY_DIR" : "/pkg-build",
          "CATKIN_PACKAGE_BIN_DESTINATION" : "/catkin-target/lib/%s" % info.manifest.name,
          "CATKIN_PACKAGE_ETC_DESTINATION" : "/catkin-target/etc/%s" % info.manifest.name,
          "CATKIN_PACKAGE_INCLUDE_DESTINATION" : "/catkin-target/include/%s" % info.manifest.name,
          "CATKIN_PACKAGE_LIB_DESTINATION" : "/catkin-target/lib/%s" % info.manifest.name,
          "CATKIN_PACKAGE_PYTHON_DESTINATION" : "/catkin-target/lib/python/%s" % info.manifest.name,
          "CATKIN_PACKAGE_SHARE_DESTINATION" : "/catkin-target/share/%s" % info.manifest.name,
          "CATKIN_GLOBAL_BIN_DESTINATION" : "/catkin-target/bin",
          "CATKIN_GLOBAL_ETC_DESTINATION" : "/catkin-target/etc",
          "CATKIN_GLOBAL_INCLUDE_DESTINATION" : "/catkin-target/include",
          "CATKIN_GLOBAL_LIB_DESTINATION" : "/catkin-target/lib",
          "CATKIN_GLOBAL_LIBEXEC_DESTINATION" : "/catkin-target/lib",
          "CATKIN_GLOBAL_PYTHON_DESTINATION" : "/catkin-target/lib/python",
          "CATKIN_GLOBAL_SHARE_DESTINATION" : "/catkin-target/share",
        }
        try:
            for cb in self._init_hooks:
                cb(info)
            self._parse_file(info, os.path.join(path, "CMakeLists.txt"))
            for cb in self._final_hooks:
                cb(info)
        except IOError as err:
            info.report(ERROR, "OS_ERROR", msg=str(err))
        self.messages += info.messages

class CatkinEnvironment(object):
    def __init__(self, rosdep_view=None):
        self.manifests = {}
        self.known_catkin_pkgs = set([])
        self.known_other_pkgs = set([])
        if rosdep_view is None:
            try:
                from rosdep2.lookup import RosdepLookup
                from rosdep2.rospkg_loader import DEFAULT_VIEW_KEY
                from rosdep2.sources_list import SourcesListLoader
                sources_loader = SourcesListLoader.create_default()
                lookup = RosdepLookup.create_from_rospkg(sources_loader=sources_loader)
                self.rosdep_view = lookup.get_rosdep_view(DEFAULT_VIEW_KEY)
            except Exception as err:
                sys.stderr.write("catkin_lint: cannot load rosdep database: %s\n" % str(err))
                sys.stderr.write("catkin_lint: unknown dependencies will be ignored\n")
                self.rosdep_view = {}
        else:
            self.rosdep_view = rosdep_view
        self.cache = {}

    def add_path(self, path):
        if not os.path.isdir(path):
            return []
        realpath = os.path.realpath(path)
        if realpath in self.cache:
            return self.cache[realpath]
        pkgs = find_packages(path)
        found = []
        for p, m in iteritems(pkgs):
            is_catkin = True
            for e in m.exports:
                if e.tagname == "build_type" and e.content != "catkin":
                    is_catkin = False
                break
            if is_catkin:
                self.known_catkin_pkgs.add(m.name)
                pm = ( os.path.join(path, p), m )
                self.manifests[m.name] = pm
                found.append(pm)
            else:
                self.known_other_pkgs.add(m.name)
        self.cache[realpath] = found
        return found

    def is_catkin_pkg(self, name):
        return name in self.known_catkin_pkgs

    def is_system_pkg(self, name):
        return name in self.rosdep_view.keys() or name in self.known_other_pkgs

    def is_known_pkg(self, name):
        return self.is_catkin_pkg(name) or self.is_system_pkg(name)

    def has_rosdep(self):
        return len(self.rosdep_view.keys()) > 0
