import unittest

import sys
sys.stderr = sys.stdout

from catkin_lint.linter import Message, ERROR, WARNING, NOTICE
import catkin_lint.output as o
from catkin_lint import __version__ as catkin_lint_version

try:
    import StringIO as io
except ImportError:
    import io

class OutputTest(unittest.TestCase):

    _demo_msgs = [
        Message(package="mock", file="mock.cmake", line=1, level=ERROR, id="MOCK_MSG", text="short text", description="long text"),
        Message(package="mock", file="mock.cmake", line=2, level=WARNING, id="MOCK_MSG", text="short text", description="long text"),
        Message(package="mock", file="mock.cmake", line=3, level=NOTICE, id="MOCK_MSG", text="short text", description="long text"),
        Message(package="mock", file="", line=0, level=ERROR, id="MOCK_MSG", text="short text", description="long text"),
        Message(package="mock", file="mock.cmake", line=0, level=ERROR, id="MOCK_MSG", text="short text", description="long text"),
    ]

    def _do_output(self, formatter, msgs):
        output = io.StringIO()
        formatter.prolog(file=output)
        for msg in msgs:
            formatter.message(msg, file=output)
        formatter.epilog(file=output)
        return output.getvalue()

    def test_text(self):
        result = self._do_output(o.TextOutput(), self._demo_msgs)
        self.assertEqual(result,
          "mock: mock.cmake(1): error: short text\n"
          "mock: mock.cmake(2): warning: short text\n"
          "mock: mock.cmake(3): notice: short text\n"
          "mock: error: short text\n"
          "mock: mock.cmake: error: short text\n"
        )

    def test_explained_text(self):
        result = self._do_output(o.ExplainedTextOutput(), self._demo_msgs)
        self.assertEqual(result,
          "mock: mock.cmake(1): error: short text\n"
          "     * long text\n"
          "mock: mock.cmake(2): warning: short text\n"
          "mock: mock.cmake(3): notice: short text\n"
          "mock: error: short text\n"
          "mock: mock.cmake: error: short text\n"
        )

    def test_xml(self):
        result = self._do_output(o.XmlOutput(), self._demo_msgs)
        self.assertEqual(result,
          '<catkin_lint version="%(version)s">'
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
          '</catkin_lint>\n' % { "version" : catkin_lint_version }
        )
