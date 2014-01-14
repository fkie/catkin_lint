import unittest

import sys
sys.stderr = sys.stdout

import catkin_lint.cmake as cmake

class CMakeParserTest(unittest.TestCase):

    def parse_all(self, s, var=None):
        result = []
        for cmd, args, line in cmake.parse(s, var):
            result.append( ( cmd, args, line) )
        return result

    def test_empty(self):
        self.assertEqual(
            self.parse_all(""),
            []
        )

    def test_command(self):
        self.assertEqual(
            self.parse_all("command()"),
            [ ("command", [], 1)]
        )
        self.assertEqual(
            self.parse_all("MiXeDCaSe()"),
            [ ("mixedcase", [], 1)]
        )
        self.assertRaises(cmake.SyntaxError, self.parse_all, "unbalanced(")
        self.assertRaises(cmake.SyntaxError, self.parse_all, "invalid%=characters$()")
        self.assertRaises(cmake.SyntaxError, self.parse_all, "()")
        self.assertRaises(cmake.SyntaxError, self.parse_all, "missing_braces")

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
        self.assertEqual(
            self.parse_all('cmd("one;two";three)'),
            [ ("cmd", [ "one;two", "three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd(one;"two;three")'),
            [ ("cmd", [ "one", "two;three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('if(NOT (A OR B)) endif()'),
            [ ("if", [ "NOT", "(", "A", "OR", "B", ")" ], 1), ( "endif", [], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd("(")'),
            [ ("cmd", [ "(" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd(")")'),
            [ ("cmd", [ ")" ], 1) ]
        )
        self.assertRaises(cmake.SyntaxError, self.parse_all, 'cmd("unclosed string)')

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

    def test_argparse(self):
        self.assertRaises(RuntimeError, cmake.argparse, [], { "TEST" : "xxx"})

        opts, args = cmake.argparse([], {})
        self.assertEqual({}, opts)
        self.assertEqual([], args)

        opts, args = cmake.argparse([], { "TEST" : "-"})
        self.assertEqual({ "TEST": False }, opts)
        self.assertEqual([], args)

        opts, args = cmake.argparse([], { "TEST" : "?"})
        self.assertEqual({ "TEST": None }, opts)
        self.assertEqual([], args)

        self.assertRaises(cmake.SyntaxError, cmake.argparse, [], { "TEST" : "!"})

        opts, args = cmake.argparse([], { "TEST" : "*"})
        self.assertEqual({ "TEST": [] }, opts)
        self.assertEqual([], args)

        self.assertRaises(cmake.SyntaxError, cmake.argparse, [], { "TEST" : "+"})

        opts, args = cmake.argparse([], { "TEST" : "p"})
        self.assertEqual({ "TEST": {} }, opts)
        self.assertEqual([], args)

        opts, args = cmake.argparse([ "argument", "BOOL"], {"BOOL" : "-"})
        self.assertEqual({ "BOOL" : True }, opts)
        self.assertEqual([ "argument" ], args)

        opts, args = cmake.argparse([ "argument", "KEY", "value" ], {"KEY" : "?"})
        self.assertEqual({ "KEY" : "value" }, opts)
        self.assertEqual([ "argument" ], args)

        opts, args = cmake.argparse([ "argument", "KEY", "value" ], {"KEY" : "!"})
        self.assertEqual({ "KEY" : "value" }, opts)
        self.assertEqual([ "argument" ], args)

        opts, args = cmake.argparse([ "argument", "LIST", "value1", "value2" ], {"LIST" : "*"})
        self.assertEqual({ "LIST" : [ "value1", "value2" ] }, opts)
        self.assertEqual([ "argument" ], args)

        opts, args = cmake.argparse([ "argument", "LIST", "value1", "value2" ], {"LIST" : "+"})
        self.assertEqual({ "LIST" : [ "value1", "value2" ] }, opts)
        self.assertEqual([ "argument" ], args)

        opts, args = cmake.argparse([ "argument", "PROPERTIES", "key1", "value1", "key2", "value2" ], { "PROPERTIES" : "p"})
        self.assertEqual({ "PROPERTIES" : { "key1" : "value1", "key2" : "value2" } }, opts)
        self.assertEqual([ "argument" ], args)

        self.assertRaises(cmake.SyntaxError, cmake.argparse, ["PROPERTIES", "key1", "value1", "key2" ], { "PROPERTIES" : "p"})

        opts, args = cmake.argparse([ "DOUBLE", "DOUBLE", "ARGUMENT", "ARGUMENT" ], {"DOUBLE ARGUMENT" : "?"})
        self.assertEqual({ "DOUBLE ARGUMENT" : "ARGUMENT" }, opts)
        self.assertEqual([ "DOUBLE" ], args)

        opts, args = cmake.argparse([ "BOOL", "argument" ], {"BOOL" : "-"})
        self.assertEqual({ "BOOL" : True }, opts)
        self.assertEqual([ "argument" ], args)
