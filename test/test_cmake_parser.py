import unittest

import sys
sys.stderr = sys.stdout

import catkin_lint.cmake as cmake

class CMakeParserTest(unittest.TestCase):

    def parse_all(self, s, var=None):
        result = []
        ctxt = cmake.ParserContext()
        for cmd, args, fname, line in ctxt.parse(s, var=var):
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
        self.assertRaises(cmake.SyntaxError, self.parse_all, "cmd();")
        self.assertRaises(cmake.SyntaxError, self.parse_all, "cmd cmd()")

    def test_string(self):
        self.assertEqual(
            self.parse_all('cmd("simple string")'),
            [ ("cmd", [ "simple string" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd("string with \\"quote\\"")'),
            [ ("cmd", [ 'string with "quote"' ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd("string that spans\nmultiple lines")'),
            [ ("cmd", [ 'string that spans\nmultiple lines' ], 1) ]
        )

    def test_macro(self):
        self.assertEqual(
            self.parse_all("macro(test) cmd() endmacro() test()"),
            [ ("cmd", [], 1) ]
        )
        self.assertEqual(
            self.parse_all("macro(test) cmd() test() endmacro() test()"),
            [ ("cmd", [], 1) ]
        )
        self.assertEqual(
            self.parse_all("macro(test) cmd(${global}) test() endmacro() test()", { "global": "value"}),
            [ ("cmd", [ "value" ], 1) ]
        )
        self.assertEqual(
            self.parse_all("macro(test arg) cmd(${arg}) endmacro() test(fun)"),
            [ ("cmd", [ "fun" ], 1) ]
        )
        self.assertEqual(
            self.parse_all("macro(test arg) cmd(${arg}) endmacro() test(local) cmd(${arg})", { "arg": "global" }),
            [ ("cmd", [ "local" ], 1), ("cmd", [ "global" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('macro(test arg) cmd(${arg}) endmacro() test("one;two;three")'),
            [ ("cmd", [ "one", "two", "three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('macro(test arg) cmd(${arg}) cmd(${ARGN}) endmacro() test(one;two;three)'),
            [ ("cmd", [ "one" ], 1), ("cmd", [ "two", "three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('macro(test arg1 arg2) cmd("${arg2}") cmd(${ARGN}) endmacro() test(one)'),
            [ ("cmd", [ "" ], 1), ("cmd", [], 1) ]
        )
        self.assertEqual(
            self.parse_all('macro(test arg) cmd("${arg}") endmacro() test("one;two;three")'),
            [ ("cmd", [ "one;two;three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('macro(test arg) cmd(${arg} ${ARGN}) endmacro() test(arg extra stuff)'),
            [ ("cmd", [ "arg", "extra", "stuff" ], 1) ]
        )
        self.assertRaises(cmake.SyntaxError, self.parse_all, "macro() endmacro()")
        self.assertRaises(cmake.SyntaxError, self.parse_all, "macro(fun)")

    def test_function(self):
        self.assertEqual(
            self.parse_all("function(test) cmd() endfunction() test()"),
            [ ("test", [], 1) ]
        )
        self.assertRaises(cmake.SyntaxError, self.parse_all, "function() endfunction()")
        self.assertRaises(cmake.SyntaxError, self.parse_all, "function(fun)")

    def test_foreach(self):
        self.assertEqual(
            self.parse_all('foreach(arg RANGE 2) cmd(${arg}) endforeach()'),
            [ ("cmd", [ "0" ], 1), ("cmd", [ "1" ], 1), ("cmd", [ "2" ], 1), ]
        )
        self.assertEqual(
            self.parse_all('foreach(arg RANGE 1 3) cmd(${arg}) endforeach()'),
            [ ("cmd", [ "1" ], 1), ("cmd", [ "2" ], 1), ("cmd", [ "3" ], 1), ]
        )
        self.assertEqual(
            self.parse_all('foreach(arg RANGE 1 5 2) cmd(${arg}) endforeach()'),
            [ ("cmd", [ "1" ], 1), ("cmd", [ "3" ], 1), ("cmd", [ "5" ], 1), ]
        )
        self.assertEqual(
            self.parse_all('foreach(arg 1 2 3 4 5) endforeach()'),
            []
        )
        self.assertEqual(
            self.parse_all('foreach(arg one) cmd(${global}) endforeach()', { "global": "value" }),
            [ ("cmd", ["value"], 1) ]
        )
        self.assertEqual(
            self.parse_all('foreach(arg IN LISTS dummy) cmd(${arg}) endforeach()', { "dummy": "one;two;three" }),
            [ ("cmd", [ "one" ], 1), ("cmd", [ "two" ], 1), ("cmd", [ "three" ], 1), ]
        )
        self.assertEqual(
            self.parse_all('foreach(arg IN ITEMS ${dummy}) cmd(${arg}) endforeach()', { "dummy": "one;two;three" }),
            [ ("cmd", [ "one" ], 1), ("cmd", [ "two" ], 1), ("cmd", [ "three" ], 1), ]
        )
        self.assertEqual(
            self.parse_all('foreach(arg ${dummy}) cmd(${arg}) endforeach()', { "dummy": "one;two;three" }),
            [ ("cmd", [ "one" ], 1), ("cmd", [ "two" ], 1), ("cmd", [ "three" ], 1), ]
        )
        self.assertEqual(
            self.parse_all('foreach(arg) cmd(${arg}) endforeach()'),
            []
        )
        self.assertRaises(cmake.SyntaxError, self.parse_all, "foreach(arg)")
        self.assertRaises(cmake.SyntaxError, self.parse_all, "foreach(arg RANGE bla) endforeach()")
        self.assertRaises(cmake.SyntaxError, self.parse_all, "foreach(arg RANGE 1 5 2 0) endforeach()")
        self.assertRaises(cmake.SyntaxError, self.parse_all, "foreach() endforeach()")

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
        self.assertEqual(
            self.parse_all('cmd("\\"")'),
            [ ("cmd", [ '"' ], 1) ]
        )
        self.assertEqual(
            self.parse_all("cmd(ENV{PATH})"),
            [ ("cmd", [ "ENV{PATH}" ], 1) ]
        )
        self.assertRaises(cmake.SyntaxError, self.parse_all, 'cmd("unclosed string)')

    def test_substitution(self):
        self.assertEqual(
            self.parse_all("cmd(${args})", { "args" : "one;two;three"}),
            [ ("cmd", [ "one", "two", "three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all("cmd(${missing})"),
            [ ("cmd", [], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd("${missing}")'),
            [ ("cmd", [ "" ], 1) ]
        )
        self.assertEqual(
            self.parse_all("${fun}()", { "fun" : "cmd"}),
            [ ("cmd", [], 1) ]
        )
        self.assertEqual(
            self.parse_all("cmd(${args})", { "args" : "one two three"}),
            [ ("cmd", [ "one", "two", "three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd("${args}")', { "args" : "one;two;three"}),
            [ ("cmd", [ "one;two;three" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd("\\${args}")', { "args" : "fail"}),
            [ ("cmd", [ "${args}" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd(\\${args})', { "args" : "fail"}),
            [ ("cmd", [ "${args}" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd(${args})', { "args" : "\\\\"}),
            [ ("cmd", [ "\\\\" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd(${args})', { "args" : "${looks_like_a_variable}"}),
            [ ("cmd", [ "${looks_like_a_variable}" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd(${args})', { "args" : ")"}),
            [ ("cmd", [ ")" ], 1) ]
        )
        self.assertEqual(
            self.parse_all('cmd(fun ${args})', { "args" : "stuff"}),
            [ ("cmd", [ "fun", "stuff" ], 1) ]
        )
        self.assertEqual(
            self.parse_all("cmd($ENV{PATH})"),
            [ ("cmd", [ "$ENV{PATH}" ], 1) ]
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
        self.assertEqual(
            self.parse_all("cmd1()\rcmd2()\rcmd3()\r"),
            [ ("cmd1", [], 1), ("cmd2", [], 2), ("cmd3", [], 3) ]
        )
        self.assertEqual(
            self.parse_all("cmd1()\ncmd2()\ncmd3()\n"),
            [ ("cmd1", [], 1), ("cmd2", [], 2), ("cmd3", [], 3) ]
        )
        self.assertEqual(
            self.parse_all("cmd1()\r\ncmd2()\r\ncmd3()\r\n"),
            [ ("cmd1", [], 1), ("cmd2", [], 2), ("cmd3", [], 3) ]
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
