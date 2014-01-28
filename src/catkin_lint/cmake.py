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
import re
from .util import iteritems, zip_longest
from copy import copy

class SyntaxError(RuntimeError):
    pass


def _escape(s):
    return re.sub(r'([\\$"])',r"\\\1", s)


def _unescape(s):
    return re.sub(r'\\(.)', r"\1", s)


_find_var = re.compile(r'(?<!\\)\$\{([a-z_0-9]+)\}', re.IGNORECASE).search
def _resolve_vars(s, var):
    mo = _find_var(s)
    while mo is not None:
        key = _unescape(mo.group(1))
        value = _escape(var[key]) if key in var else ""
        s = s[:mo.start(0)] + value + s[mo.end(0):]
        mo = _find_var(s)
    return s


_token_spec = [
    ( 'NL', r'\r\n|\r|\n' ),
    ( 'SKIP', r'[ \t]+' ),
    ( 'LPAREN', r'\(' ),
    ( 'RPAREN', r'\)' ),
    ( 'STRING', r'"[^\\"]*(?:\\.[^\\"]*)*"' ),
    ( 'SEMICOLON', r';'),
    ( 'WORD', r'[^\(\)"# \t\r\n;]+' ),
    ( 'COMMENT', r'#.*?$' ),
]
_next_token = re.compile('|'.join('(?P<%s>%s)' % pair for pair in _token_spec), re.MULTILINE | re.IGNORECASE).match
def _lexer(s):
    keywords = set([])
    line = 1
    mo = _next_token(s)
    pos = 0
    while mo is not None:
        typ = mo.lastgroup
        if typ == 'NL':
            line += 1
        elif typ != 'SKIP':
            val = mo.group(typ)
            if val.upper() in keywords: typ = val.upper()
            yield ( typ, val, line )
        pos = mo.end()
        mo = _next_token(s, pos)
    if pos != len(s):
        raise SyntaxError("Unexpected character %r on line %d" % (s[pos], line))


def _resolve_args(arg_tokens, var):
    args = []
    for typ, val in arg_tokens:
        if typ == "STRING":
            val = val[1:-1]
            val = _resolve_vars(val, var)
            args.append(_unescape(val))
        elif typ == "WORD":
            val = _resolve_vars(val, var)
            if val:
                args += re.split(r";|[ \t]+", _unescape(val))
        elif typ != "SEMICOLON":
            args.append(val)
    return args


class Command(object):
    def __init__(self, name, args, filename, line):
        self.name = name
        self.args = args
        self.filename = filename
        self.line = line


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
    for typ, val, line in _lexer(s):
        if typ == "COMMENT": continue
        if state == 0:
            if typ != "WORD":
                raise SyntaxError("%s(%d): expected command identifier and got '%s'" % (filename, line, val))
            cmdname = val.lower()
            cmdargs = []
            cmdline = line
            state = 1
        elif state == 1:
            if typ != "LPAREN":
                raise SyntaxError("%s(%d): expected '(' and got '%s'" % (filename, line, val))
            paren = 1
            state = 2
        elif state == 2:
            if typ == "LPAREN":
                paren += 1
            elif typ == "RPAREN":
                paren -= 1
                if paren == 0:
                    commands.append(Command(cmdname, cmdargs, filename, cmdline))
                    state = 0
                    continue
            cmdargs.append((typ, val))
    if state == 1:
        raise SyntaxError("%s(%d): expected '(' and got end of file" % (filename, line))
    if state == 2:
        raise SyntaxError("%s(%d): expected ')' and got end of file" % (filename, line))
    return commands


def _parse_block(filename, cmds, block_name, result_type, *args):
    result = result_type(*args)
    while cmds:
        cmd = cmds.pop(0)
        if cmd.name == "end%s" % block_name: return result
        result.commands.append(cmd)
    raise SyntaxError("%s: expected 'end%s()' and got end of file" % (filename, block_name))


class ParserContext(object):
    def __init__(self, parent=None):
        self.parent = parent
        self.callable = copy(parent.callable) if parent is not None else {}
        self._call_stack = set([])

    def call(self, name, args, var=None, skip_callable=False):
        if name in self._call_stack: return
        if var is None: var = {}
        f = self.callable[name]
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
            self._call_stack.add(name)
            for cmd, args, fname, line in self._yield(cmds, var, skip_callable):
                yield (cmd, args, fname, line)
        finally:
            self._call_stack.remove(name)
            for key, value in iteritems(save_vars):
                if value is not None:
                    var[key] = value
                else:
                    del var[key]
            if "ARGN" in var: del var["ARGN"]

    def _yield(self, cmds, var, skip_callable):
        if var is None: var = {}
        while cmds:
            cmd = cmds.pop(0)
            cmdname = _resolve_vars(cmd.name, var)
            if not re.match(r'^[a-z_][a-z_0-9]*$', cmdname):
                raise SyntaxError("%s(%d): invalid command identifier '%s'" % (cmd.filename, cmd.line, cmdname))
            args = _resolve_args(cmd.args, var)
            if cmd.name == "macro":
                if not args:
                    raise SyntaxError("%s(%d): malformed macro() definition" % (cmd.filename, cmd.line))
                f = _parse_block(cmd.filename, cmds, cmdname, Callable, args, False)
                self.callable[f.name] = f
            elif cmd.name == "function":
                if not args:
                    raise SyntaxError("%s(%d): malformed function() definition" % (cmd.filename, cmd.line))
                f = _parse_block(cmd.filename, cmds, cmdname, Callable, args, True)
                self.callable[f.name] = f
            elif cmd.name == "foreach":
                if not args:
                    raise SyntaxError("%s(%d): malformed foreach() loop" % (cmd.filename, cmd.line))
                f = _parse_block(cmd.filename, cmds, cmdname, BasicBlock)
                loop_var = args.pop(0)
                if not args: continue
                if args[0] == "RANGE":
                    try:
                        if len(args) == 2:
                            loop_args = range(int(args[1])+1)
                        elif len(args) == 3:
                            loop_args = range(int(args[1]), int(args[2])+1)
                        elif len(args) == 4:
                            loop_args = range(int(args[1]), int(args[2])+1, int(args[3]))
                        else:
                            raise SyntaxError("%s(%d): RANGE expects one, two, or three integers" % (cmd.filename, cmd.line))
                    except ValueError:
                        raise SyntaxError("%s(%d): invalid RANGE parameters" % (cmd.filename, cmd.line))
                elif args[:2] == ["IN","LISTS"]:
                    loop_args = []
                    for l in args[2:]:
                        if l in var:
                            loop_args += var[l].split(";")
                elif args[:2] == ["IN","ITEMS"]:
                    loop_args = args[2:]
                else:
                    loop_args = args
                for loop_value in loop_args:
                    var[loop_var] = str(loop_value)
                    loop_cmds = copy(f.commands)
                    for cmd, args, fname, line in self._yield(loop_cmds, var, skip_callable):
                        yield (cmd, args, fname, line)
            elif cmdname in self.callable:
                f = self.callable[cmdname]
                if skip_callable or f.new_context:
                    yield (cmdname, args, cmd.filename, cmd.line)
                else:
                    for cmd, args, fname, line in self.call(cmdname, args, var, skip_callable):
                        yield (cmd, args, fname, line)
            else:
                yield (cmdname, args, cmd.filename, cmd.line)

    def parse(self, s, var=None, filename=None, skip_callable=False):
        if filename is None: filename = "<inline>"
        cmds = _parse_commands(s, filename)
        for cmd, args, fname, line in self._yield(cmds, var, skip_callable):
            yield (cmd, args, fname, line)


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
            raise RuntimeError("Invalid option '%s': %s" % (optname, opttype))
    curname = None
    curtype = None
    t_args = args[:]
    while t_args:
        l = 0
        for k,v in iteritems(opts):
            kl = k.split()
            ll = len(kl)
            if kl == t_args[:ll]:
                if l < ll:
                    l = ll
                    curname = k
                    curtype = v
        if l > 0:
            del t_args[:l]
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
                    raise SyntaxError("Option '%s' has truncated key-value pair" % curname)
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
            raise SyntaxError("Option '%s' requires at least one argument" % optname)
        if opttype == "!" and not result[optname]:
            raise SyntaxError("Option '%s' requires exactly one argument" % optname)
    return result, remaining

