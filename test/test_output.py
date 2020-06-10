# coding=utf-8
#
# catkin_lint
# Copyright (c) 2013-2020 Fraunhofer FKIE
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

import unittest

from catkin_lint.linter import Message, ERROR, WARNING, NOTICE
import catkin_lint.output as o
from catkin_lint import __version__ as catkin_lint_version

try:
    import StringIO as io
except ImportError:
    import io


class OutputTest(unittest.TestCase):

    _demo_msgs = [
        Message(package="mock", file_name="mock.cmake", line=1, level=ERROR, msg_id="MOCK_MSG", text="short text", description="long text"),
        Message(package="mock", file_name="mock.cmake", line=2, level=WARNING, msg_id="MOCK_MSG", text="short text", description="long text"),
        Message(package="mock", file_name="mock.cmake", line=3, level=NOTICE, msg_id="MOCK_MSG", text="short text", description="long text"),
        Message(package="mock", file_name="", line=0, level=ERROR, msg_id="MOCK_MSG", text="short text", description="long text"),
        Message(package="mock", file_name="mock.cmake", line=0, level=ERROR, msg_id="MOCK_MSG", text="short text", description="long text"),
    ]

    def _do_output(self, formatter, msgs):
        output = io.StringIO()
        formatter.prolog(fd=output)
        for msg in msgs:
            formatter.message(msg, fd=output)
        formatter.epilog(fd=output)
        return output.getvalue()

    def test_text(self):
        """Test output format for catkin_lint text output"""
        result = self._do_output(o.TextOutput(o.Color.Never), self._demo_msgs)
        self.assertEqual(result,
                         "mock: mock.cmake(1): error: short text\n"
                         "mock: mock.cmake(2): warning: short text\n"
                         "mock: mock.cmake(3): notice: short text\n"
                         "mock: error: short text\n"
                         "mock: mock.cmake: error: short text\n"
                         )

    def test_explained_text(self):
        """Test output format for catkin_lint text output with explanations"""
        result = self._do_output(o.ExplainedTextOutput(o.Color.Never), self._demo_msgs)
        self.assertEqual(result,
                         "mock: mock.cmake(1): error: short text\n"
                         "     * long text\n"
                         "     * You can ignore this problem with --ignore mock_msg\n"
                         "mock: mock.cmake(2): warning: short text\n"
                         "mock: mock.cmake(3): notice: short text\n"
                         "mock: error: short text\n"
                         "mock: mock.cmake: error: short text\n"
                         )

    def test_json(self):
        """Test output format for catkin_lint JSON output"""
        result = self._do_output(o.JsonOutput(), self._demo_msgs)
        self.assertEqual(result,
                         '{"errors": ['
                         '{"id": "MOCK_MSG", "location": {"file": "mock.cmake", "line": 1, "package": "mock"}, "text": "short text"}, '
                         '{"id": "MOCK_MSG", "location": {"package": "mock"}, "text": "short text"}, '
                         '{"id": "MOCK_MSG", "location": {"file": "mock.cmake", "package": "mock"}, "text": "short text"}'
                         '], "notices": ['
                         '{"id": "MOCK_MSG", "location": {"file": "mock.cmake", "line": 3, "package": "mock"}, "text": "short text"}'
                         '], "version": "%(version)s", "warnings": ['
                         '{"id": "MOCK_MSG", "location": {"file": "mock.cmake", "line": 2, "package": "mock"}, "text": "short text"}'
                         ']}\n' % {"version": catkin_lint_version}
                         )

    def test_xml(self):
        """Test output format for catkin_lint XML output"""
        result = self._do_output(o.XmlOutput(), self._demo_msgs)
        self.assertEqual(result,
                         '<catkin_lint xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="https://raw.githubusercontent.com/fkie/catkin_lint/%(version)s/catkin_lint.xsd" version="%(version)s">'
                         '<error>'
                         '<location><package>mock</package><file>mock.cmake</file><line>1</line></location>'
                         '<id>MOCK_MSG</id><text>short text</text>'
                         '</error>'
                         '<warning>'
                         '<location><package>mock</package><file>mock.cmake</file><line>2</line></location>'
                         '<id>MOCK_MSG</id><text>short text</text>'
                         '</warning>'
                         '<notice>'
                         '<location><package>mock</package><file>mock.cmake</file><line>3</line></location>'
                         '<id>MOCK_MSG</id><text>short text</text>'
                         '</notice>'
                         '<error>'
                         '<location><package>mock</package></location>'
                         '<id>MOCK_MSG</id><text>short text</text>'
                         '</error>'
                         '<error>'
                         '<location><package>mock</package><file>mock.cmake</file></location>'
                         '<id>MOCK_MSG</id><text>short text</text>'
                         '</error>'
                         '</catkin_lint>\n' % {"version": catkin_lint_version}
                         )
