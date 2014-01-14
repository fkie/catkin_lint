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
from catkin_lint.util import iteritems

_find_var = re.compile(r'\$\{([a-z_0-9]+)\}', re.IGNORECASE).search

_token_spec = [
    ( 'NL', r'\r|\n|\r\n' ),
    ( 'LPAREN', r'\(' ),
    ( 'RPAREN', r'\)' ),
    ( 'ID', r'[a-z_][a-z_0-9]*(?=[ \t\r\n\(\)#])' ),
    ( 'STRING', r'"[^\r\n]*?"' ),
    ( 'WORD', r'[^\(\)"#; \t\r\n]+' ),
    ( 'COMMENT', r'#.*?$' ),
    ( 'SKIP', r'[ \t;]+' ),
]
_next_token = re.compile('|'.join('(?P<%s>%s)' % pair for pair in _token_spec), re.MULTILINE | re.IGNORECASE).match

class SyntaxError(RuntimeError):
    pass

def _resolve(s, var):
    mo = _find_var(s)
    while mo is not None:
        key = mo.group(1)
        value = var[key] if key in var else ""
        s = s[:mo.start(0)] + value + s[mo.end(0):]
        mo = _find_var(s)
    return s

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

def _expect(etyp, typ, val, line):
    if not typ in etyp:
        raise SyntaxError("Unexpected token '%s' on line %d" % ( val, line ))
    return val

def parse(s, var=None):
    state = 0
    cmd = None
    cmdline = 0
    for typ, val, line in _lexer(s):
        if typ == "COMMENT": continue
        if state == 0:
            cmd = _expect(["ID"], typ, val, line).lower()
            args = []
            state = 1
            cmdline = line
        elif state == 1:
            _expect(["LPAREN"], typ, val, line)
            paren = 1
            state = 2
        elif state == 2:
            _expect([ "LPAREN", "RPAREN","ID","WORD","STRING"], typ, val, line)
            if var is not None: val = _resolve(val, var)
            if typ == "LPAREN":
                paren += 1
                args.append("(")
            elif typ == "RPAREN":
                paren -= 1
                if paren == 0:
                    yield ( cmd, args, cmdline )
                    state = 0
                else:
                    args.append(")")
            elif typ == "STRING":
                args.append(val[1:-1])
            else:
                args += re.split(";|[ \t]+", val)
    if state != 0:
        raise SyntaxError("Unexpected end of file")

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

