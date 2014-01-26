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
import sys
import textwrap
from .linter import ERROR, WARNING, NOTICE
from . import __version__

class TextOutput(object):

    diagnostic_label = { ERROR : "error", WARNING : "warning", NOTICE : "notice" }

    def prolog(self, file=sys.stdout):
        pass

    def message(self, msg, file=sys.stdout):
        loc = msg.package
        explained = set([])
        if msg.file:
            if msg.line:
                fn = "%s(%d)" % ( msg.file, msg.line )
            else:
                fn = msg.file
            loc = "%s: %s" % (msg.package, fn)
        file.write("%s: %s: %s\n" % (loc, self.diagnostic_label[msg.level], msg.text))

    def epilog(self, file=sys.stdout):
        pass


class ExplainedTextOutput(TextOutput):

    def __init__(self):
        self.explained = set([])

    def message(self, msg, file=sys.stdout):
        TextOutput.message(self, msg, file)
        if not msg.id in self.explained:
            self.explained.add(msg.id)
            file.write("%s\n" % textwrap.fill(msg.description, initial_indent="     * ", subsequent_indent="     * "))


class XmlOutput(object):

    tag_label = { ERROR : "error", WARNING : "warning", NOTICE : "notice" }

    def _quote(self, s):
        return s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;').replace('"', '&quote;')

    def prolog(self, file=sys.stdout):
        file.write ('<catkin_lint version="%s">' % __version__)

    def message(self, msg, file=sys.stdout):
        file.write('<%s><location><package>%s</package>' % (self.tag_label[msg.level], self._quote(msg.package)))
        if msg.file:
            file.write('<file>%s</file>' % self._quote(msg.file))
            if msg.line:
                file.write('<line>%s</line>' % msg.line)
        file.write('</location><id>%s</id><text>%s</text>' % (msg.id, self._quote(msg.text)))
        file.write('</%s>' % self.tag_label[msg.level])

    def epilog(self, file=sys.stdout):
        file.write ('</catkin_lint>\n')
