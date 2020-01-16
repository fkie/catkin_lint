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
from .util import iteritems, zip_longest
from copy import copy


class CMakeSyntaxError(RuntimeError):
    pass


def _escape(s):
    return re.sub(r'([\\$"])', r"\\\1", s)


def _unescape(s):
    return s if "\\" not in s else re.sub(r'\\(.)', r"\1", s)


_find_var = re.compile(r'(?<!\\)\$\{([a-z_0-9]+)\}', re.IGNORECASE).search
_find_env_var = re.compile(r'(?<!\\)\$ENV\{([A-Za-z_0-9]+)\}').search


def _resolve_vars(s, var, env_var):
    if var is not None:
        mo = _find_var(s)
        while mo is not None:
            key = _unescape(mo.group(1))
            value = _escape(var.get(key, ""))
            s = s[:mo.start(0)] + value + s[mo.end(0):]
            mo = _find_var(s)
    if env_var is not None:
        mo = _find_env_var(s)
        while mo is not None:
            key = _unescape(mo.group(1))
            value = _escape(env_var.get(key, "$ENV{%s}" % key))
            s = s[:mo.start(0)] + value + s[mo.end(0):]
            mo = _find_env_var(s)
    return s


_find_genexp = re.compile(r'(?<!\\)\$<([a-z_0-9]+)(?::([^<>]+))>', re.IGNORECASE).search


# TODO We just replace all generator expressions by empty strings.
#      This may or may not be a smart thing to do in this context
def _resolve_generator_expressions(s):
    mo = _find_genexp(s)
    while mo is not None:
        s = s[:mo.start(0)] + s[mo.end(0):]
        mo = _find_genexp(s)
    return s


_token_spec = [
    ('NL', r'\r\n|\r|\n'),
    ('SKIP', r'[ \t]+'),
    ('LPAREN', r'\('),
    ('RPAREN', r'\)'),
    ('STRING', r'"(?:\\.|[^\\"])*"'),
    ('SEMICOLON', r';'),
    ('WORD', r'(?:\\.|[^\\\(\)"# \t\r\n;])+'),
    ('PRAGMA', r'#catkin_lint:.*?$'),
    ('COMMENT', r'#.*?$'),
]
_next_token = re.compile('|'.join('(?P<%s>%s)' % pair for pair in _token_spec), re.MULTILINE | re.IGNORECASE).match


def _lexer(s):
    line = 1
    col = 1
    mo = _next_token(s)
    pos = 0
    while mo is not None:
        typ = mo.lastgroup
        if typ == 'NL':
            line += 1
            col = 1
        else:
            if typ != 'SKIP':
                val = mo.group(typ)
                if typ == "STRING":
                    val = val[1:-1]
                yield (typ, val, line, col)
            col += mo.end() - mo.start()
        pos = mo.end()
        mo = _next_token(s, pos)
    if pos != len(s):
        raise CMakeSyntaxError("Unexpected character %r on line %d" % (s[pos], line))


_arg_spec = [
    ('SKIP', r';'),
    ('ARG', r'(?:\\.|[^;])+'),
]
_next_arg = re.compile('|'.join('(?P<%s>%s)' % pair for pair in _arg_spec)).match


def _resolve_args(arg_tokens, var, env_var):
    args = []
    for typ, val in arg_tokens:
        if typ == "STRING":
            val = _resolve_vars(val, var, env_var)
            # Treat quoted strings as a single word
            args.append(_unescape(val))
        elif typ == "WORD":
            val = _resolve_vars(val, var, env_var)
            # Split unquoted text into list items
            mo = _next_arg(val)
            while mo is not None:
                typ = mo.lastgroup
                if typ != "SKIP":
                    arg = mo.group(typ)
                    args.append(_unescape(arg))
                pos = mo.end()
                mo = _next_arg(val, pos)
        elif typ != "SEMICOLON":
            args.append(val)
    return args


class Command(object):
    def __init__(self, name, args, filename, line, column):
        self.name = name
        self.args = args
        self.filename = filename
        self.line = line
        self.column = column


class BasicBlock(object):
    def __init__(self):
        self.commands = []


class Callable(BasicBlock):
    def __init__(self, params, new_context):
        BasicBlock.__init__(self)
        self.name = params[0]
        self.params = params[1:]
        self.new_context = new_context


def _parse_commands(s, filename):
    commands = []
    state = 0
    line = 0
    s = _resolve_generator_expressions(s)
    for typ, val, line, col in _lexer(s):
        if typ == "COMMENT":
            continue
        if typ == "PRAGMA":
            args = re.split(r'\s+', val[13:])
            commands.append(Command("#catkin_lint", [("LITERAL", arg) for arg in args if len(arg) > 0], filename, line, col))
            continue
        if state == 0:
            if typ != "WORD":
                raise CMakeSyntaxError("%s(%d): expected command identifier and got '%s'" % (filename, line, val))
            cmdname = val
            cmdargs = []
            cmdline = line
            cmdcol = col
            state = 1
        elif state == 1:
            if typ != "LPAREN":
                raise CMakeSyntaxError("%s(%d): expected '(' and got '%s'" % (filename, line, val))
            paren = 1
            state = 2
        elif state == 2:
            if typ == "LPAREN":
                paren += 1
            elif typ == "RPAREN":
                paren -= 1
                if paren == 0:
                    commands.append(Command(cmdname, cmdargs, filename, cmdline, cmdcol))
                    state = 0
                    continue
            cmdargs.append((typ, val))
    if state == 1:
        raise CMakeSyntaxError("%s(%d): expected '(' and got end of file" % (filename, line))
    if state == 2:
        raise CMakeSyntaxError("%s(%d): expected ')' and got end of file" % (filename, line))
    return commands


def _parse_block(filename, cmds, block_name, result_type, *args):
    result = result_type(*args)
    nesting = 1
    while cmds:
        if cmds[0].name.lower() == block_name.lower():
            nesting += 1
        if cmds[0].name.lower() == "end%s" % block_name.lower():
            nesting -= 1
            if nesting == 0:
                return result
        cmd = cmds.pop(0)
        result.commands.append(cmd)
    raise CMakeSyntaxError("%s: expected 'end%s()' and got end of file" % (filename, block_name))


class ParserContext(object):
    def __init__(self, parent=None):
        self.parent = parent
        self.callable = copy(parent.callable) if parent is not None else {}
        self._call_stack = set([])
        self._skip_block = False
        self._block_level = -1

    def call_depth(self):
        return len(self._call_stack)

    def _call(self, name, args, var, env_var=None, skip_callable=False):
        lname = name.lower()
        if lname in self._call_stack:
            return
        f = self.callable[lname]
        argn = []
        save_vars = {}
        try:
            for key, value in zip_longest(f.params, args):
                if key is None:
                    argn.append(value)
                elif key:
                    save_vars[key] = var[key] if key in var else None
                    var[key] = value if value is not None else ""
            var["ARGN"] = ';'.join(argn)
            cmds = copy(f.commands)
            self._call_stack.add(lname)
            for cmd, args, arg_tokens, loc in self._yield(cmds, var, env_var, skip_callable):
                yield (cmd, args, arg_tokens, loc)
        finally:
            self._call_stack.remove(lname)
            for key, value in iteritems(save_vars):
                if value is not None:
                    var[key] = value
                else:
                    del var[key]
            if "ARGN" in var:
                del var["ARGN"]

    def _yield(self, cmds, var, env_var, skip_callable):
        if var is None:
            var = {}
        self._block_level = self._block_level + 1
        while cmds:
            if self._block_level == 0:
                self._skip_block = False
            cmd = cmds.pop(0)
            cmdname = _resolve_vars(cmd.name, var, env_var)
            cmdname_lower = cmdname.lower()
            if not re.match(r'^#?[a-z_][a-z_0-9]*$', cmdname_lower):
                raise CMakeSyntaxError("%s(%d): invalid command identifier '%s'" % (cmd.filename, cmd.line, cmdname))
            args = _resolve_args(cmd.args, var, env_var)
            if cmd.name.lower() == "macro":
                if not args:
                    raise CMakeSyntaxError("%s(%d): malformed macro() definition" % (cmd.filename, cmd.line))
                f = _parse_block(cmd.filename, cmds, cmdname, Callable, args, False)
                self.callable[f.name.lower()] = f
                yield (cmdname, args, cmd.args, (cmd.filename, cmd.line, cmd.column))
            elif cmd.name.lower() == "function":
                if not args:
                    raise CMakeSyntaxError("%s(%d): malformed function() definition" % (cmd.filename, cmd.line))
                f = _parse_block(cmd.filename, cmds, cmdname, Callable, args, True)
                self.callable[f.name.lower()] = f
                yield (cmdname, args, cmd.args, (cmd.filename, cmd.line, cmd.column))
            elif cmd.name.lower() == "if":
                f = _parse_block(cmd.filename, cmds, cmdname, BasicBlock)
                if not self._skip_block:
                    yield (cmdname, args, cmd.args, (cmd.filename, cmd.line, cmd.column))
                    for cmd, args, arg_tokens, loc in self._yield(f.commands, var, env_var, skip_callable):
                        if cmd.lower() == "else":
                            self._skip_block = False
                        if not self._skip_block:
                            yield (cmd, args, arg_tokens, loc)
                    self._skip_block = False
            elif cmd.name.lower() == "foreach":
                if not args:
                    raise CMakeSyntaxError("%s(%d): malformed foreach() loop" % (cmd.filename, cmd.line))
                f = _parse_block(cmd.filename, cmds, cmdname, BasicBlock)
                if not self._skip_block:
                    yield (cmdname, args, cmd.args, (cmd.filename, cmd.line, cmd.column))
                    loop_var = args[0]
                    if len(args) == 1:
                        continue
                    if args[1] == "RANGE":
                        try:
                            if len(args) == 3:
                                loop_args = range(int(args[2]) + 1)
                            elif len(args) == 4:
                                loop_args = range(int(args[2]), int(args[3]) + 1)
                            elif len(args) == 5:
                                loop_args = range(int(args[2]), int(args[3]) + 1, int(args[4]))
                            else:
                                raise CMakeSyntaxError("%s(%d): RANGE expects one, two, or three integers" % (cmd.filename, cmd.line))
                        except ValueError:
                            raise CMakeSyntaxError("%s(%d): invalid RANGE parameters" % (cmd.filename, cmd.line))
                    elif args[1:3] == ["IN", "LISTS"]:
                        loop_args = []
                        for l in args[3:]:
                            if l in var:
                                loop_args += var[l].split(";")
                    elif args[1:3] == ["IN", "ITEMS"]:
                        loop_args = args[3:]
                    else:
                        loop_args = args[1:]
                    for loop_value in loop_args:
                        var[loop_var] = str(loop_value)
                        loop_cmds = copy(f.commands)
                        for cmd, args, arg_tokens, loc in self._yield(loop_cmds, var, env_var, skip_callable):
                            yield (cmd, args, arg_tokens, loc)
                            if self._skip_block:
                                break
                        self._skip_block = False
            elif cmdname_lower in self.callable:
                f = self.callable[cmdname_lower]
                if skip_callable or f.new_context:
                    yield (cmdname, args, cmd.args, (cmd.filename, cmd.line, cmd.column))
                elif not self._skip_block:
                    for cmd, args, arg_tokens, loc in self._call(cmdname, args, var, env_var, skip_callable):
                        yield (cmd, args, arg_tokens, loc)
                        if self._skip_block:
                            break
                    self._skip_block = False
            else:
                yield (cmdname, args, cmd.args, (cmd.filename, cmd.line, cmd.column))
        self._block_level = self._block_level - 1

    def parse(self, s, var=None, env_var=None, filename=None, skip_callable=False):
        if filename is None:
            filename = "<inline>"
        cmds = _parse_commands(s, filename)
        self._block_level = -1
        for cmd, args, arg_tokens, loc in self._yield(cmds, var, env_var, skip_callable):
            yield (cmd, args, arg_tokens, loc)

    def skip_block(self):
        self._skip_block = True


def argparse(args, opts):
    result = {}
    remaining = []
    for optname, opttype in iteritems(opts):
        if opttype == "*" or opttype == "+":
            result[optname] = []
        elif opttype == "?" or opttype == "!":
            result[optname] = None
        elif opttype == "-":
            result[optname] = False
        elif opttype == "p":
            result[optname] = {}
        else:
            raise RuntimeError("invalid option '%s': %s" % (optname, opttype))
    curname = None
    curtype = None
    t_args = args[:]
    while t_args:
        L = 0
        for k, v in iteritems(opts):
            kl = k.split()
            ll = len(kl)
            if kl == t_args[:ll]:
                if L < ll:
                    L = ll
                    curname = k
                    curtype = v
        if L > 0:
            del t_args[:L]
            if curtype == "-":
                result[curname] = True
                curname = None
                curtype = None
        elif curname is not None:
            if curtype == "?" or curtype == "!":
                result[curname] = t_args[0]
                curname = None
                curtype = None
                del t_args[0]
            elif curtype == "p":
                if len(t_args) < 2:
                    t_args.append("")
                result[curname][t_args[0]] = t_args[1]
                del t_args[:2]
            else:
                result[curname].append(t_args[0])
                del t_args[0]
        else:
            remaining.append(t_args[0])
            del t_args[0]
    for optname, opttype in iteritems(opts):
        if opttype == "+" and not result[optname]:
            raise CMakeSyntaxError("option '%s' has empty, unquoted argument" % optname)
        if opttype == "!" and not result[optname]:
            raise CMakeSyntaxError("option '%s' has empty, unquoted argument" % optname)
    return result, remaining
