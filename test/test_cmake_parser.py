import unittest

import sys
sys.stderr = sys.stdout

import catkin_lint.cmake

class CMakeParserTest(unittest.TestCase):

    def parse_all(self, s, var=None):
        result = []
        for cmd, args, line in catkin_lint.cmake.parse(s, var):
            result.append( ( cmd, args, line) )
        return result

    def test_command(self):
        self.assertEqual(
            self.parse_all("command()"),
            [ ("command", [], 1)]
        )
        self.assertEqual(
            self.parse_all("MiXeDCaSe()"),
            [ ("mixedcase", [], 1)]
        )
        self.assertRaises(RuntimeError, self.parse_all, "unbalanced(")
        self.assertRaises(RuntimeError, self.parse_all, "invalid%=characters$()")
        self.assertRaises(RuntimeError, self.parse_all, "()")
        self.assertRaises(RuntimeError, self.parse_all, "missing_braces")

    def test_arguments(self):
        self.assertEqual(
            self.parse_all("cmd(one two three)"),
            [ ("cmd", [ "one", "two", "three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all("cmd(one two;three)"),
            [ ("cmd", [ "one", "two", "three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all("cmd(one;two;three)"),
            [ ("cmd", [ "one", "two", "three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all("cmd(one;two three)"),
            [ ("cmd", [ "one", "two", "three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd("one;two" three)'),
            [ ("cmd", [ "one;two", "three" ], 1) ]
        )

    def test_substitution(self):
        self.assertEqual(
            self.parse_all("cmd(${args})", { "args" : "one;two;three"}),
            [ ("cmd", [ "one", "two", "three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all("cmd(${args})", { "args" : "one two three"}),
            [ ("cmd", [ "one", "two", "three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd("${args}")', { "args" : "one;two;three"}),
            [ ("cmd", [ "one;two;three" ], 1) ]
        )

    def test_comments(self):
        self.assertEqual(
            self.parse_all("""\
            # initial comment
            cmd(one # first argument comment
            two # second argument comment
            three# third argument comment without space
            )## closing comment
            # commented-out command
            # cmd()
            """),
            [ ("cmd", [ "one", "two", "three" ], 2) ]
        )

    def test_line_numbering(self):
        self.assertEqual(
            self.parse_all("""\
            cmd1()
            cmd2(
            )
            # Comment
            cmd3()
            """),
            [ ("cmd1", [], 1), ("cmd2", [], 2), ("cmd3", [], 5) ]
        )
