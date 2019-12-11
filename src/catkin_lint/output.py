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

import sys
import textwrap
import json
from .linter import ERROR, WARNING, NOTICE
from . import __version__


def isatty(fd):
    return hasattr(fd, "isatty") and fd.isatty()


class Color(object):
    Never = 0
    Always = 1
    Auto = 2
    switch_on = {False: {ERROR: "", WARNING: "", NOTICE: ""},
                 True: {ERROR: "\033[1;31m", WARNING: "\033[1;33m", NOTICE: "\033[36m"}}
    switch_off = {False: "", True: "\033[0m"}


class TextOutput(object):

    diagnostic_label = {ERROR: "error", WARNING: "warning", NOTICE: "notice"}

    def __init__(self, color):
        self.color = color

    def prolog(self, fd=sys.stdout):
        pass

    def message(self, msg, fd=sys.stdout):
        use_color = self.color == Color.Always or (self.color == Color.Auto and isatty(fd))
        loc = msg.package
        if msg.file:
            if msg.line:
                fn = "%s(%d)" % (msg.file, msg.line)
            else:
                fn = msg.file
            loc = "%s: %s" % (msg.package, fn)
        fd.write("%s: %s%s%s: %s\n" % (loc, Color.switch_on[use_color][msg.level], self.diagnostic_label[msg.level], Color.switch_off[use_color], msg.text))

    def epilog(self, fd=sys.stdout):
        pass


class ExplainedTextOutput(TextOutput):

    def __init__(self, color):
        TextOutput.__init__(self, color)
        self.explained = set()

    def message(self, msg, fd=sys.stdout):
        TextOutput.message(self, msg, fd)
        if msg.id not in self.explained:
            self.explained.add(msg.id)
            fd.write("%s\n" % textwrap.fill(msg.description, initial_indent="     * ", subsequent_indent="     * "))
            fd.write("     * You can ignore this problem with --ignore %s\n" % msg.id.lower())


class XmlOutput(object):

    tag_label = {ERROR: "error", WARNING: "warning", NOTICE: "notice"}

    def _quote(self, s):
        return s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;').replace('"', '&quote;')

    def prolog(self, fd=sys.stdout):
        fd.write('<catkin_lint xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="https://raw.githubusercontent.com/fkie/catkin_lint/%(version)s/catkin_lint.xsd" version="%(version)s">' % {"version": __version__})

    def message(self, msg, fd=sys.stdout):
        fd.write('<%s><location><package>%s</package>' % (self.tag_label[msg.level], self._quote(msg.package)))
        if msg.file:
            fd.write('<file>%s</file>' % self._quote(msg.file))
            if msg.line:
                fd.write('<line>%s</line>' % msg.line)
        fd.write('</location><id>%s</id><text>%s</text>' % (msg.id, self._quote(msg.text)))
        fd.write('</%s>' % self.tag_label[msg.level])

    def epilog(self, fd=sys.stdout):
        fd.write('</catkin_lint>\n')


class JsonOutput(object):

    def __init__(self):
        self._json = {"errors": [], "warnings": [], "notices": [], "version": __version__}

    def prolog(self, fd=sys.stdout):
        pass

    def message(self, msg, fd=sys.stdout):
        location = {"package": msg.package}
        if msg.file:
            location["file"] = msg.file
            if msg.line:
                location["line"] = msg.line
        entry = {"id": msg.id, "text": msg.text, "location": location}
        if msg.level == ERROR:
            self._json["errors"].append(entry)
        elif msg.level == WARNING:
            self._json["warnings"].append(entry)
        elif msg.level == NOTICE:
            self._json["notices"].append(entry)

    def epilog(self, fd=sys.stdout):
        json.dump(self._json, fd, ensure_ascii=False, indent=None, sort_keys=True)
        fd.write("\n")
