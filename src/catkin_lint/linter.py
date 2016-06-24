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
from functools import total_ordering
from fnmatch import fnmatch
from copy import copy
from .cmake import ParserContext, argparse as cmake_argparse, CMakeSyntaxError
from .diagnostics import msg

ERROR = 0
WARNING = 1
NOTICE = 2


@total_ordering
class Message(object):

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
        self.subdir = ""
        self.subdirs = set([])
        self.manifest = None
        self.file = ""
        self.line = 0
        self.ignore_messages = set([])
        self.ignored_messages = 0
        self.commands = set([])
        self.find_packages = set([])
        self.targets = set([])
        self.executables = set([])
        self.libraries = set([])
        self.conditionals = []
        self.var = {}
        self.parent_var = {}
        self.messages = []
        self._pkg_source = os.path.normpath("/pkg-source")
        self._pkg_build = os.path.normpath("/pkg-build")

    def report(self, level, msg_id, **kwargs):
        if msg_id in self.ignore_messages:
            self.ignored_messages += 1
            return
        msg_id, text, description = msg(msg_id, **kwargs)
        self.messages.append(Message(
            package=self.manifest.name,
            file=self.file,
            line=self.line,
            level=level,
            id=msg_id,
            text=text,
            description=description
        ))

    def package_path(self, path):
        if not path:
            return ""
        new_path = os.path.normpath(os.path.join(self.var["CMAKE_CURRENT_SOURCE_DIR"], path))
        if new_path.startswith(self._pkg_source):
            new_path = new_path[len(self._pkg_source) + 1:]
        return new_path

    def real_path(self, path):
        return os.path.normpath(os.path.join(self.path, path))

    def is_internal_path(self, path):
        tmp = os.path.normpath(os.path.join(self.var["CMAKE_CURRENT_SOURCE_DIR"], path))
        return tmp.startswith(self._pkg_source) or tmp.startswith(self._pkg_build)

    def is_catkin_target(self, path, subdir=None):
        catkin_dir = "/catkin-target"
        if subdir is not None:
            catkin_dir = os.path.join(catkin_dir, subdir)
        return os.path.normpath(path).startswith(os.path.normpath(catkin_dir))

    def condition_is_true(self, expr):
        ret = False
        for c in self.conditionals:
            if c.expr == expr and not c.value:
                return False
            if c.expr == expr and c.value:
                ret = True
        return ret


class IfCondition(object):
    def __init__(self, expr, value):
        self.expr = expr
        self.value = value


class CMakeLinter(object):
    def __init__(self, env):
        self.env = env
        self.messages = []
        self.ignore_messages = set([])
        self.ignored_messages = 0
        self._cmd_hooks = {}
        self._running_hooks = set([])
        self._init_hooks = []
        self._final_hooks = []
        self._added_checks = set([])
        self._catch_circular_deps = set([])
        self._include_blacklist = {"catkin": ["*"]}
        self._ctx = ParserContext()

    def require(self, check):
        if check in self._catch_circular_deps:
            raise RuntimeError("Circular dependency detected")
        if check in self._added_checks:
            return
        self._added_checks.add(check)
        self._catch_circular_deps.add(check)
        check(self)
        self._catch_circular_deps.remove(check)

    def add_init_hook(self, cb):
        self._init_hooks.append(cb)

    def add_command_hook(self, cmd, cb):
        if cmd not in self._cmd_hooks:
            self._cmd_hooks[cmd] = [cb]
        else:
            self._cmd_hooks[cmd].append(cb)

    def add_final_hook(self, cb):
        self._final_hooks.append(cb)

    def _read_file(self, filename):  # pragma: no cover
        with open(filename, "r") as f:
            content = f.read()
        return content

    def _include_file(self, info, args):
        opts, args = cmake_argparse(args, {"OPTIONAL": "-", "RESULT_VARIABLE": "?", "NO_POLICY_SCOPE": "-"})
        if not args:
            return
        if "/" not in args[0] and "." not in args[0]:
            incl_file = "NOTFOUND"
        else:
            incl_file = info.package_path(args[0])
            if incl_file.startswith(os.path.normpath("/find-path")):
                return
            skip_parsing = False
            if info.manifest.name in self._include_blacklist:
                for glob_pattern in self._include_blacklist[info.manifest.name]:
                    if fnmatch(incl_file, glob_pattern):
                        skip_parsing = True
                        break
            real_file = os.path.join(info.path, incl_file)
            if os.path.isfile(real_file):
                if not skip_parsing:
                    self._parse_file(info, real_file)
            else:
                if not opts["OPTIONAL"]:
                    info.report(ERROR, "MISSING_FILE", cmd="include", file=incl_file)
                incl_file = "NOTFOUND"
        if opts["RESULT_VARIABLE"]:
            info.var[opts["RESULT_VARIABLE"]] = incl_file

    def _subdirectory(self, info, args):
        _, args = cmake_argparse(args, {"EXCLUDE_FROM_ALL": "-"})
        subdir = info.package_path(args[0])
        real_subdir = info.real_path(subdir)
        if os.path.isabs(subdir):
            info.report(ERROR, "EXTERNAL_SUBDIR", subdir=subdir)
            return
        if not os.path.isdir(real_subdir):
            info.report(ERROR, "MISSING_SUBDIR", subdir=subdir)
            return
        if subdir in info.subdirs:
            info.report(ERROR, "DUPLICATE_SUBDIR", subdir=subdir)
            return
        info.subdirs.add(subdir)
        old_subdir = info.subdir
        old_parent_var = info.parent_var
        info.parent_var = info.var
        info.var = copy(info.var)
        try:
            info.var["CMAKE_CURRENT_SOURCE_DIR"] = os.path.join(info._pkg_source, subdir)
            info.subdir = subdir
            self._parse_file(info, os.path.join(real_subdir, "CMakeLists.txt"))
        finally:
            info.var = info.parent_var
            info.parent_var = old_parent_var
            info.subdir = old_subdir

    def _handle_list(self, info, args):
        try:
            op = args.pop(0)
            name = args.pop(0)
            items = info.var[name].split(';') if name in info.var and info.var[name] != "" else []
            if op == "APPEND":
                items += args
            elif op == "INSERT":
                pos = int(args.pop(0))
                items[pos:pos] = args
            elif op == "REVERSE":
                items.reverse()
            elif op == "SORT":
                items.sort()
            elif op == "REMOVE_ITEM":
                for a in args:
                    while a in items:
                        items.remove(a)
            elif op == "REMOVE_AT":
                args = [int(a) for a in args]
                args.sort()
                args.reverse()
                for a in args:
                    try:
                        del items[a]
                    except IndexError:
                        pass
            elif op == "REMOVE_DUPLICATES":
                new_items = []
                for i in items:
                    if i not in new_items:
                        new_items.append(i)
                items = new_items
            elif op == "LENGTH":
                output = args.pop(-1)
                info.var[output] = str(len(items))
                return
            elif op == "GET":
                output = args.pop(-1)
                sub_list = []
                for a in args:
                    try:
                        sub_list.append(items[int(a)])
                    except IndexError:
                        pass
                info.var[output] = ';'.join(sub_list)
                return
            elif op == "FIND":
                output = args.pop(-1)
                a = args.pop(0)
                info.var[output] = str(items.index(a)) if a in items else "-1"
                return
            else:
                return
            info.var[name] = ';'.join(items)
        except:
            pass

    def _handle_pragma(self, info, args):
        pragma = args.pop(0)
        if pragma == "ignore":
            info.ignore_messages |= set([a.upper() for a in args])

    def _handle_if(self, info, cmd, args, arg_tokens):
        if cmd == "if":
            info.conditionals.append(IfCondition(" ".join(args), True))
            if len(arg_tokens) == 1 and re.match("\${[a-z_0-9]+}$", arg_tokens[0][1]):
                info.report(WARNING, "AMBIGUOUS_CONDITION", cond=arg_tokens[0][1])
            for i, tok in enumerate(arg_tokens):
                if tok[0] != "WORD":
                    continue
                if tok[1][:3] == "STR" or tok[1][:8] == "VERSION_" or tok[1] in ["MATCHES", "IS_NEWER_THAN"]:
                    if i == 0 or i == len(arg_tokens) - 1:
                        raise CMakeSyntaxError("%s(%d): missing argument for binary operator %s" % (info.file, info.line, tok))
                    if arg_tokens[i - 1][0] != "STRING" or arg_tokens[i + 1][0] != "STRING":
                        info.report(NOTICE, "UNQUOTED_STRING_OP", op=tok[1])
                if tok[1] in ["EXISTS", "IS_DIRECTORY", "IS_SYMLINK", "IS_ABSOLUTE"]:
                    if i == len(arg_tokens) - 1:
                        raise CMakeSyntaxError("%s(%d): missing argument for unary operator %s" % (info.file, info.line, tok))
                    if arg_tokens[i + 1][0] != "STRING":
                        info.report(NOTICE, "UNQUOTED_STRING_OP", op=tok[1])
        if cmd == "else":
            if len(info.conditionals) > 0:
                info.conditionals[-1].value = False
        if cmd == "endif":
            if len(info.conditionals) > 0:
                info.conditionals.pop()

    def execute_hook(self, info, cmd, args):
        if cmd in self._running_hooks:
            return
        if cmd == "project":
            info.var["PROJECT_NAME"] = args[0]
            info.var["PROJECT_SOURCE_DIR"] = info.var["CMAKE_CURRENT_SOURCE_DIR"]
            info.var["PROJECT_BINARY_DIR"] = info.var["CMAKE_CURRENT_BINARY_DIR"]
        self._running_hooks.add(cmd)
        if cmd in self._cmd_hooks:
            for cb in self._cmd_hooks[cmd]:
                cb(info, cmd, args)
        if cmd == "set":
            opts, args = cmake_argparse(args, {"PARENT_SCOPE": "-", "FORCE": "-", "CACHE": "*"})
            if opts["PARENT_SCOPE"]:
                info.parent_var[args[0]] = ';'.join(args[1:])
            else:
                info.var[args[0]] = ';'.join(args[1:])
        if cmd == "unset":
            opts, args = cmake_argparse(args, {"CACHE": "-"})
            info.var[args[0]] = ""
        if cmd == "list":
            self._handle_list(info, args)
        if cmd == "include":
            self._include_file(info, args)
        if cmd == "add_subdirectory":
            saved_hooks = self._running_hooks
            self._running_hooks = set()
            self._subdirectory(info, args)
            self._running_hooks = saved_hooks
        if cmd == "find_package":
            info.var["%s_INCLUDE_DIRS" % args[0]] = "/%s-includes" % args[0]
            info.var["%s_INCLUDE_DIRS" % args[0].upper()] = "/%s-includes" % args[0]
            info.var["%s_LIBRARIES" % args[0]] = "/%s-libs/library.so" % args[0]
            info.var["%s_LIBRARIES" % args[0].upper()] = "/%s-libs/library.so" % args[0]
            info.find_packages.add(args[0])
        if cmd == "add_executable":
            info.targets.add(args[0])
            if "IMPORTED" not in args:
                info.executables.add(args[0])
        if cmd == "add_library":
            info.targets.add(args[0])
            if "IMPORTED" not in args:
                info.libraries.add(args[0])
        if cmd == "add_custom_target":
            info.targets.add(args[0])
        if cmd == "find_path":
            info.var[args[0]] = "/find-path"
        if cmd == "find_library":
            info.var[args[0]] = "/find-libs/library.so"
        if cmd == "find_file":
            info.var[args[0]] = "/find-file/filename.ext"
        self._running_hooks.discard(cmd)

    def _parse_file(self, info, filename):
        save_file = info.file
        save_line = info.line
        try:
            cur_file = os.path.relpath(filename, info.path)
            info.var["CMAKE_CURRENT_LIST_FILE"] = cur_file
            info.file = cur_file
            info.line = 0
            content = self._read_file(filename)
            # cur_col is a len(call_stack) list of indentation lists
            # i.e. each function/macro invocation has its own indentation list
            cur_col = [[None]]
            cur_depth = 0
            for cmd, args, arg_tokens, (fname, line, column) in self._ctx.parse(content, var=info.var, env_var=self.env.os_env, filename=cur_file):
                info.file = fname
                info.line = line
                if cmd == "#catkin_lint":
                    self._handle_pragma(info, args)
                    continue
                if "$ENV{" in ";".join(args):
                    info.report(WARNING, "ENV_VAR")
                if cmd != cmd.lower():
                    info.report(NOTICE, "CMD_CASE", cmd=cmd)
                    cmd = cmd.lower()
                if cmd != "project" and "PROJECT_NAME" in info.var:
                    for _, val in arg_tokens:
                        if info.var["PROJECT_NAME"] in val:
                            info.report(NOTICE, "LITERAL_PROJECT_NAME", name=info.var["PROJECT_NAME"])
                            break
                depth = self._ctx.call_depth()
                if depth > cur_depth:
                    cur_col += [[None]] * (depth - cur_depth)
                    cur_depth = depth
                if depth < cur_depth:
                    del cur_col[depth + 1:]
                    cur_depth = depth
                if cmd == "else":
                    if len(cur_col[-1]) < 2:
                        raise CMakeSyntaxError("%s(%d): else() without if()" % (info.file, info.line))
                    if column != cur_col[-1][-2]:
                        info.report(NOTICE, "INDENTATION")
                    cur_col[-1][-1] = None
                elif cmd in ["endif", "endforeach"]:
                    cur_col[-1].pop(-1)
                    if len(cur_col[-1]) == 0:
                        raise CMakeSyntaxError("%s(%d): %s() without %s()" % (info.file, info.line, cmd, cmd[3:]))
                    if column != cur_col[-1][-1]:
                        info.report(NOTICE, "INDENTATION")
                else:
                    if cur_col[-1][-1] is None:
                        cur_col[-1][-1] = column
                        if len(cur_col[-1]) >= 2 and cur_col[-1][-2] >= cur_col[-1][-1]:
                            info.report(NOTICE, "INDENTATION")
                    if column != cur_col[-1][-1]:
                        info.report(NOTICE, "INDENTATION")
                if cmd in ["if", "foreach"]:
                    cur_col[-1].append(None)
                if cmd in ["if", "else", "endif"]:
                    self._handle_if(info, cmd, args, arg_tokens)
                if cmd == "project" and info.subdir:
                    info.report(WARNING, "SUBPROJECT", subdir=info.subdir)
                    return
                self.execute_hook(info, cmd, args)
                info.commands.add(cmd)
        finally:
            info.file = save_file
            info.line = save_line

    def lint(self, path, manifest, info=None):
        if info is None:
            info = LintInfo(self.env)
        info.ignore_messages = self.ignore_messages
        info.path = path
        info.manifest = manifest
        info.conditionals = []
        info.var = {
            "CMAKE_CURRENT_SOURCE_DIR": "/pkg-source",
            "CMAKE_CURRENT_BINARY_DIR": "/pkg-build",
            "CATKIN_PACKAGE_BIN_DESTINATION": "/catkin-target/lib/%s" % info.manifest.name,
            "CATKIN_PACKAGE_ETC_DESTINATION": "/catkin-target/etc/%s" % info.manifest.name,
            "CATKIN_PACKAGE_INCLUDE_DESTINATION": "/catkin-target/include/%s" % info.manifest.name,
            "CATKIN_PACKAGE_LIB_DESTINATION": "/catkin-target/lib/%s" % info.manifest.name,
            "CATKIN_PACKAGE_PYTHON_DESTINATION": "/catkin-target/lib/python/%s" % info.manifest.name,
            "CATKIN_PACKAGE_SHARE_DESTINATION": "/catkin-target/share/%s" % info.manifest.name,
            "CATKIN_GLOBAL_BIN_DESTINATION": "/catkin-target/bin",
            "CATKIN_GLOBAL_ETC_DESTINATION": "/catkin-target/etc",
            "CATKIN_GLOBAL_INCLUDE_DESTINATION": "/catkin-target/include",
            "CATKIN_GLOBAL_LIB_DESTINATION": "/catkin-target/lib",
            "CATKIN_GLOBAL_LIBEXEC_DESTINATION": "/catkin-target/lib",
            "CATKIN_GLOBAL_PYTHON_DESTINATION": "/catkin-target/lib/python",
            "CATKIN_GLOBAL_SHARE_DESTINATION": "/catkin-target/share",
        }
        self.ctx = ParserContext()
        try:
            for cb in self._init_hooks:
                cb(info)
            self._parse_file(info, os.path.join(path, "CMakeLists.txt"))
            for cb in self._final_hooks:
                cb(info)
        except IOError as err:  # pragma: no cover
            info.report(ERROR, "OS_ERROR", msg=str(err))
        self.messages += info.messages
        self.ignored_messages += info.ignored_messages
