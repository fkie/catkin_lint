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
import random
import string
from fnmatch import fnmatch
from copy import copy
from .cmake import ParserContext, argparse as cmake_argparse, CMakeSyntaxError
from .diagnostics import msg

ERROR = 0
WARNING = 1
NOTICE = 2


class Message(object):

    def __init__(self, package, file_name, line, level, msg_id, text, description):
        self.package = package
        self.file = file_name
        self.line = line
        self.level = level
        self.id = msg_id
        self.text = text
        self.description = description

    def __lt__(self, other):
        return (self.package, self.level, self.file, self.line, self.id) < (other.package, other.level, other.file, other.line, other.id)


def generate_random_id(L=16):
    return "".join(random.choice(string.ascii_letters + string.digits) for _ in range(L))


class PathClass(object):
    SOURCE = 0
    BINARY = 1
    DISCOVERED = 3
    CATKIN = 4
    OTHER = 5


class PathConstants(object):
    PACKAGE_SOURCE = "/%s" % generate_random_id()
    PACKAGE_BINARY = "/%s" % generate_random_id()
    CATKIN_DEVEL = "/%s" % generate_random_id()
    CATKIN_INSTALL = "/%s" % generate_random_id()
    DISCOVERED_PATH = "/%s" % generate_random_id()


class LintInfo(object):

    def __init__(self, env):
        self.env = env
        self.path = None
        self.subdir = ""
        self.subdirs = set()
        self.manifest = None
        self.file = ""
        self.line = 0
        self.ignore_message_ids = set()
        self.ignore_message_ids_once = set()
        self.ignored_messages = []
        self.command_loc = {}
        self.commands = set()
        self.find_packages = set()
        self.targets = set()
        self.executables = set()
        self.libraries = set()
        self.static_libraries = set()
        self.interface_libraries = set()
        self.conditionals = []
        self.var = {}
        self.parent_var = {}
        self.messages = []
        self.generated_files = set([""])

    def report(self, level, msg_id, **kwargs):
        file_name, line = self.file, self.line
        loc = kwargs.get("file_location", None)
        if loc:
            file_name, line = loc
            del kwargs["file_location"]
        msg_id, text, description = msg(msg_id, **kwargs)
        if msg_id in self.ignore_message_ids or msg_id in self.ignore_message_ids_once:
            self.ignored_messages.append(Message(
                package=self.manifest.name,
                file_name=file_name,
                line=line,
                level=level,
                msg_id=msg_id,
                text=text,
                description=description
            ))
            return
        self.messages.append(Message(
            package=self.manifest.name,
            file_name=file_name,
            line=line,
            level=level,
            msg_id=msg_id,
            text=text,
            description=description
        ))

    def current_location(self):
        return (self.file, self.line) if self.file and self.line else None

    def location_of(self, command):
        return self.command_loc.get(command, None)

    def source_relative_path(self, path):
        new_path = posixpath.normpath(posixpath.join(self.var["CMAKE_CURRENT_SOURCE_DIR"], path.replace(os.path.sep, "/")))
        if new_path.startswith(PathConstants.PACKAGE_SOURCE):
            new_path = new_path[len(PathConstants.PACKAGE_SOURCE) + 1:]
        return new_path

    def binary_relative_path(self, path):
        new_path = posixpath.normpath(posixpath.join(self.var["CMAKE_CURRENT_BINARY_DIR"], path.replace(os.path.sep, "/")))
        if new_path.startswith(PathConstants.PACKAGE_BINARY):
            new_path = new_path[len(PathConstants.PACKAGE_BINARY) + 1:]
        return new_path

    def report_path(self, path):
        new_path = path.replace(PathConstants.PACKAGE_BINARY, "${PROJECT_BUILD_DIR}")
        new_path = new_path.replace(PathConstants.CATKIN_DEVEL, "${CATKIN_DEVEL_PREFIX}")
        new_path = new_path.replace(PathConstants.CATKIN_INSTALL, "${CATKIN_INSTALL_PREFIX}")
        if new_path.startswith(PathConstants.PACKAGE_SOURCE):
            return posixpath.normpath(path[len(PathConstants.PACKAGE_SOURCE) + 1:])
        return posixpath.normpath(new_path)

    def real_path(self, path):
        return os.path.normpath(os.path.join(self.path, path))

    def find_package_path(self, pkg, path):
        return posixpath.join(PathConstants.DISCOVERED_PATH, pkg, path)

    def path_class(self, path):
        tmp = posixpath.normpath(posixpath.join(self.var["CMAKE_CURRENT_SOURCE_DIR"], path.replace(os.path.sep, "/")))
        if tmp.startswith(PathConstants.PACKAGE_SOURCE):
            return PathClass.SOURCE
        if tmp.startswith(PathConstants.PACKAGE_BINARY):
            return PathClass.BINARY
        if tmp.startswith(PathConstants.DISCOVERED_PATH):
            return PathClass.DISCOVERED
        if tmp.startswith(PathConstants.CATKIN_DEVEL) or tmp.startswith(PathConstants.CATKIN_INSTALL):
            return PathClass.CATKIN
        return PathClass.OTHER

    def is_valid_path(self, path, valid=[PathClass.SOURCE, PathClass.BINARY, PathClass.DISCOVERED]):
        return self.path_class(path) in valid

    def is_existing_path(self, path, check=os.path.exists, require_source_folder=False, discovered_path_ok=True):
        if self.condition_is_checked("EXISTS %s" % path) or (check == os.path.isdir and self.condition_is_checked("IS_DIRECTORY %s" % path)):
            return True
        tmp = path.replace(os.path.sep, "/")
        if tmp.startswith(PathConstants.PACKAGE_SOURCE):
            tmp = path[len(PathConstants.PACKAGE_SOURCE) + 1:]
        if check(os.path.normpath(os.path.join(self.path, self.subdir, tmp))):
            return True
        tmp = posixpath.normpath(posixpath.join(self.var["CMAKE_CURRENT_SOURCE_DIR"], path.replace(os.path.sep, "/")))
        if tmp.startswith(PathConstants.PACKAGE_SOURCE):
            if not require_source_folder and not posixpath.isabs(path) and tmp[len(PathConstants.PACKAGE_SOURCE) + 1:] in self.generated_files:
                return True
            if not require_source_folder and tmp in self.generated_files:
                return True
            return check(os.path.join(self.path, os.path.normpath(tmp[len(PathConstants.PACKAGE_SOURCE) + 1:])))
        if not require_source_folder and tmp.startswith(PathConstants.PACKAGE_BINARY):
            return tmp[len(PathConstants.PACKAGE_BINARY) + 1:] in self.generated_files
        if not require_source_folder and tmp in self.generated_files:
            return True
        if not require_source_folder and tmp.startswith(PathConstants.CATKIN_DEVEL):
            s = tmp[len(PathConstants.CATKIN_DEVEL) + 1:]
            for t in ["include", "lib", "share", "bin"]:
                if s.startswith(t):
                    return True
        if not require_source_folder and tmp.startswith(PathConstants.CATKIN_INSTALL):
            s = tmp[len(PathConstants.CATKIN_INSTALL) + 1:]
            for t in ["include", "lib", "share", "bin"]:
                if s.startswith(t):
                    return True
        return tmp.startswith(PathConstants.DISCOVERED_PATH) and discovered_path_ok

    def is_internal_path(self, path):
        tmp = posixpath.normpath(posixpath.join(self.var["CMAKE_CURRENT_SOURCE_DIR"], path))
        return tmp.startswith(PathConstants.PACKAGE_SOURCE) or tmp.startswith(PathConstants.PACKAGE_BINARY)

    def is_catkin_install_destination(self, path, subdir=None):
        full_path = posixpath.normpath(posixpath.join(PathConstants.CATKIN_INSTALL, path))
        catkin_dir = posixpath.join(PathConstants.CATKIN_INSTALL, subdir or "")
        return full_path.startswith(catkin_dir)

    def is_catkin_bin_install_destination(self, path):
        return self.is_catkin_install_destination(path, "bin") or self.is_catkin_install_destination(path, "lib/%s" % self.manifest.name)

    def condition_is_checked(self, expr):
        for c in self.conditionals:
            if expr in c.expr and c.value:
                return True
        return False


class IfCondition(object):
    def __init__(self, expr, value):
        self.expr = expr
        self.value = value


class CMakeLinter(object):
    def __init__(self, env):
        self.env = env
        self.messages = []
        self.ignore_message_ids = set()
        self.ignored_messages = []
        self._cmd_hooks = {}
        self._running_hooks = set([])
        self._init_hooks = []
        self._final_hooks = []
        self._added_checks = set([])
        self._catch_circular_deps = set([])
        self._include_blacklist = {"catkin": ["*"]}
        self._ctx = None

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

    def _read_file(self, filename):
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
            incl_file = info.source_relative_path(args[0])
            if incl_file.startswith(PathConstants.DISCOVERED_PATH):
                return
            skip_parsing = False
            if info.manifest.name in self._include_blacklist:
                for glob_pattern in self._include_blacklist[info.manifest.name]:
                    if fnmatch(incl_file, glob_pattern):
                        skip_parsing = True
                        break
            real_file = os.path.join(info.path, os.path.normpath(incl_file))
            if os.path.isfile(real_file):
                if not skip_parsing:
                    self._parse_file(info, real_file)
            else:
                if not opts["OPTIONAL"]:
                    if posixpath.isabs(incl_file):
                        info.report(ERROR, "EXTERNAL_FILE", cmd="include", file=args[0])
                    else:
                        info.report(ERROR, "MISSING_FILE", cmd="include", file=incl_file)
                incl_file = "NOTFOUND"
        if opts["RESULT_VARIABLE"]:
            info.var[opts["RESULT_VARIABLE"]] = incl_file

    def _subdirectory(self, info, args):
        _, args = cmake_argparse(args, {"EXCLUDE_FROM_ALL": "-"})
        subdir = info.source_relative_path(args[0])
        real_subdir = info.real_path(subdir)
        if posixpath.isabs(subdir):
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
        old_find_packages = info.find_packages
        info.parent_var = info.var
        info.var = copy(info.var)
        info.find_packages = copy(info.find_packages)
        try:
            info.var["CMAKE_CURRENT_SOURCE_DIR"] = posixpath.join(PathConstants.PACKAGE_SOURCE, subdir)
            info.var["CMAKE_CURRENT_BINARY_DIR"] = posixpath.join(PathConstants.PACKAGE_BINARY, subdir)
            info.generated_files.add(subdir)
            info.subdir = subdir
            self._parse_file(info, os.path.join(real_subdir, "CMakeLists.txt"))
        finally:
            info.var = info.parent_var
            info.parent_var = old_parent_var
            info.subdir = old_subdir
            info.find_packages = old_find_packages

    def _handle_list(self, info, args):
        try:
            op = args.pop(0)
            name = args.pop(0)
            items = info.var[name].split(';') if name in info.var and info.var[name] != "" else []
            if op == "APPEND":
                items += args
            elif op == "PREPEND":
                items = args + items
            elif op == "INSERT":
                pos = int(args.pop(0))
                items[pos:pos] = args
            elif op == "REVERSE":
                items.reverse()
            elif op == "SORT":
                items.sort()
            elif op == "POP_FRONT":
                for a in args:
                    info.var[a] = items.pop(0)
                if not args:
                    items.pop(0)
            elif op == "POP_BACK":
                for a in args:
                    info.var[a] = items.pop(-1)
                if not args:
                    items.pop(-1)
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
            elif op == "JOIN":
                glue, output = args
                info.var[output] = glue.join(items)
                return
            elif op == "SUBLIST":
                first, length, output = int(args[0]), int(args[1]), args[2]
                if first < 0:
                    first = max(0, len(items) + first)
                if length == -1 or first + length > len(items):
                    length = len(items) - first
                info.var[output] = ";".join(items[first:first + length])
                return
            else:
                return
            info.var[name] = ';'.join(items)
        except Exception:
            pass

    def _handle_pragma(self, info, args):
        pragma = args.pop(0)
        if pragma == "ignore":
            info.ignore_message_ids |= set([a.upper() for a in args])
        if pragma == "report":
            info.ignore_message_ids -= set([a.upper() for a in args])
        if pragma == "ignore_once":
            info.ignore_message_ids_once |= set([a.upper() for a in args])
        if pragma == "skip":
            self._ctx.skip_block()

    def _handle_if(self, info, cmd, args, arg_tokens):
        def is_string_comparison_op(x):
            return x in ["MATCHES", "IS_NEWER_THAN", "STRLESS", "STRGREATER", "STREQUAL", "STRLESS_EQUAL", "STRGREATER_EQUAL", "VERSION_LESS", "VERSION_GREATER", "VERSION_EQUAL", "VERSION_LESS_EQUAL", "VERSION_GREATER_EQUAL"]
        if cmd == "if":
            info.conditionals.append(IfCondition(" ".join(args), True))
            if len(arg_tokens) == 1 and re.match(r"\${[a-z_0-9]+}$", arg_tokens[0][1]):
                info.report(WARNING, "AMBIGUOUS_CONDITION", cond=arg_tokens[0][1])
            for i, tok in enumerate(arg_tokens):
                if tok[0] != "WORD":
                    continue
                if is_string_comparison_op(tok[1]):
                    if i == 0 or i == len(arg_tokens) - 1:
                        raise CMakeSyntaxError("%s(%d): missing argument for binary operator %s" % (info.file, info.line, tok[1]))
                    if (arg_tokens[i - 1][0] != "STRING" and "${" in arg_tokens[i - 1][1]) or (arg_tokens[i + 1][0] != "STRING" and "${" in arg_tokens[i + 1][1]):
                        info.report(NOTICE, "UNQUOTED_STRING_OP", op=tok[1])
                if tok[1] in ["EXISTS", "IS_DIRECTORY", "IS_SYMLINK", "IS_ABSOLUTE"]:
                    if i == len(arg_tokens) - 1:
                        raise CMakeSyntaxError("%s(%d): missing argument for unary operator %s" % (info.file, info.line, tok[1]))
                    if arg_tokens[i + 1][0] != "STRING":
                        info.report(NOTICE, "UNQUOTED_STRING_OP", op=tok[1])
        if cmd == "else":
            if len(info.conditionals) > 0:
                info.conditionals[-1].value = False
        if cmd == "endif":
            if len(info.conditionals) > 0:
                info.conditionals.pop()

    def execute_hook(self, info, other_cmd, args):
        cmd = other_cmd.lower()
        if cmd not in self._running_hooks:
            self._running_hooks.add(cmd)
            try:
                if cmd == "project":
                    opts, args = cmake_argparse(args, {"VERSION": "?"})
                    version = opts["VERSION"] or ""
                    version_parts = version.split(".")
                    while len(version_parts) < 4:
                        version_parts.append("0")
                    info.var["PROJECT_NAME"] = args[0]
                    info.var["PROJECT_VERSION"] = info.var["%s_VERSION" % args[0]] = version
                    info.var["PROJECT_VERSION_MAJOR"] = info.var["%s_VERSION_MAJOR" % args[0]] = version_parts[0]
                    info.var["PROJECT_VERSION_MINOR"] = info.var["%s_VERSION_MINOR" % args[0]] = version_parts[1]
                    info.var["PROJECT_VERSION_PATCH"] = info.var["%s_VERSION_PATCH" % args[0]] = version_parts[2]
                    info.var["PROJECT_VERSION_TWEAK"] = info.var["%s_VERSION_TWEAK" % args[0]] = version_parts[3]
                    info.var["PROJECT_SOURCE_DIR"] = info.var["CMAKE_CURRENT_SOURCE_DIR"]
                    info.var["PROJECT_BINARY_DIR"] = info.var["CMAKE_CURRENT_BINARY_DIR"]
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
                    info.var["%s_INCLUDE_DIRS" % args[0]] = info.find_package_path(args[0], "include")
                    info.var["%s_INCLUDE_DIRS" % args[0].upper()] = info.find_package_path(args[0], "include")
                    info.var["%s_LIBRARIES" % args[0]] = posixpath.join(info.find_package_path(args[0], "lib"), "library.so")
                    info.var["%s_LIBRARIES" % args[0].upper()] = posixpath.join(info.find_package_path(args[0], "lib"), "library.so")
                    info.find_packages.add(args[0])
                if cmd == "add_executable":
                    info.targets.add(args[0])
                    if "IMPORTED" not in args and "ALIAS" not in args:
                        info.executables.add(args[0])
                if cmd == "add_library":
                    info.targets.add(args[0])
                    if "IMPORTED" not in args and "ALIAS" not in args and "OBJECT" not in args:
                        info.libraries.add(args[0])
                        if "STATIC" in args:
                            info.static_libraries.add(args[0])
                        if "INTERFACE" in args:
                            info.interface_libraries.add(args[0])
                if cmd == "add_custom_target":
                    info.targets.add(args[0])
                if cmd == "find_path":
                    info.var[args[0]] = info.find_package_path(args[0], "folder")
                if cmd == "find_library":
                    info.var[args[0]] = info.find_package_path(args[0], "library.so")
                if cmd == "find_file":
                    info.var[args[0]] = info.find_package_path(args[0], "filename.ext")
            except CMakeSyntaxError as e:
                info.report(WARNING, "ARGUMENT_ERROR", msg=str(e))
            finally:
                self._running_hooks.discard(cmd)

    def _parse_file(self, info, filename):
        save_file = info.file
        save_line = info.line
        save_ctx = self._ctx
        try:
            self._ctx = ParserContext()
            cur_file = os.path.relpath(filename, info.path)
            info.var["CMAKE_CURRENT_LIST_FILE"] = cur_file
            info.var["CMAKE_CURRENT_LIST_DIR"] = os.path.dirname(cur_file) or "."
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
                        # We do not complain about the project name in source file names
                        if cmd in ["add_executable", "add_library"]:
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
                info.command_loc[cmd] = info.current_location()
                info.ignore_message_ids_once.clear()
        finally:
            info.file = save_file
            info.line = save_line
            info.ignore_message_ids_once.clear()
            self._ctx = save_ctx

    def lint(self, path, manifest, info=None):
        if info is None:
            info = LintInfo(self.env)
        info.ignore_message_ids = copy(self.ignore_message_ids)
        info.path = os.path.abspath(path)
        info.manifest = manifest
        info.conditionals = []
        info.var = {
            "CMAKE_CURRENT_SOURCE_DIR": PathConstants.PACKAGE_SOURCE,
            "CMAKE_CURRENT_BINARY_DIR": PathConstants.PACKAGE_BINARY,
            "CMAKE_ARCHIVE_OUTPUT_DIRECTORY": "%s/lib" % PathConstants.CATKIN_DEVEL,
            "CMAKE_LIBRARY_OUTPUT_DIRECTORY": "%s/lib" % PathConstants.CATKIN_DEVEL,
            "CMAKE_RUNTIME_OUTPUT_DIRECTORY": "%s/lib/%s" % (PathConstants.CATKIN_DEVEL, info.manifest.name),
            "CATKIN_INSTALL_PREFIX": PathConstants.CATKIN_INSTALL,
            "CMAKE_INSTALL_PREFIX": PathConstants.CATKIN_INSTALL,
            "CATKIN_DEVEL_PREFIX": PathConstants.CATKIN_DEVEL,
            "CATKIN_PACKAGE_BIN_DESTINATION": "lib/%s" % info.manifest.name,
            "CATKIN_PACKAGE_ETC_DESTINATION": "etc/%s" % info.manifest.name,
            "CATKIN_PACKAGE_INCLUDE_DESTINATION": "include/%s" % info.manifest.name,
            "CATKIN_PACKAGE_LIB_DESTINATION": "lib",
            "CATKIN_PACKAGE_PYTHON_DESTINATION": "lib/python/%s" % info.manifest.name,
            "CATKIN_PACKAGE_SHARE_DESTINATION": "share/%s" % info.manifest.name,
            "CATKIN_GLOBAL_BIN_DESTINATION": "bin",
            "CATKIN_GLOBAL_ETC_DESTINATION": "etc",
            "CATKIN_GLOBAL_INCLUDE_DESTINATION": "include",
            "CATKIN_GLOBAL_LIB_DESTINATION": "lib",
            "CATKIN_GLOBAL_LIBEXEC_DESTINATION": "lib",
            "CATKIN_GLOBAL_PYTHON_DESTINATION": "lib/python/packages",
            "CATKIN_GLOBAL_SHARE_DESTINATION": "share",
            "PYTHON_INSTALL_DIR": "lib/python/packages",
        }
        try:
            if os.path.basename(info.path) != manifest.name:
                info.report(NOTICE, "PACKAGE_PATH_NAME", path=info.path)
            for cb in self._init_hooks:
                cb(info)
            self._parse_file(info, os.path.join(path, "CMakeLists.txt"))
            for cb in self._final_hooks:
                cb(info)
        except IOError as err:
            info.report(ERROR, "OS_ERROR", msg=str(err))
        self.messages += info.messages
        self.ignored_messages += info.ignored_messages
