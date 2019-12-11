import unittest
import catkin_lint.cmake as cmake


class CMakeParserTest(unittest.TestCase):

    def parse_all(self, s, var=None, env_var=None, location=None):
        result = []
        ctxt = cmake.ParserContext()
        for cmd, args, arg_tokens, (fname, line, column) in ctxt.parse(s, var=var, env_var=env_var):
            if cmd == "#catkin_lint" and args and args[0] == "skip":
                ctxt.skip_block()
                continue
            if location is None:
                result.append((cmd, args))
            elif location == 1:
                result.append((cmd, args, line))
            elif location == 2:
                result.append((cmd, args, line, column))
        return result

    def test_empty(self):
        """Test CMake parser with empty file"""
        self.assertEqual(
            self.parse_all(""),
            []
        )

    def test_generator_expressions(self):
        """Test CMake parser generator expressions"""
        self.assertEqual(
            self.parse_all("command($<0:ignore_me>)"),
            [("command", [])]
        )

    def test_command(self):
        """Test CMake parser command parsing"""
        self.assertEqual(
            self.parse_all("command()"),
            [("command", [])]
        )
        self.assertEqual(
            self.parse_all("MiXeDCaSe()"),
            [("MiXeDCaSe", [])]
        )
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "unbalanced(")
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "invalid%=characters$()")
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "()")
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "missing_braces")
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "cmd();")
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "cmd cmd()")

    def test_string(self):
        """Test CMake parser string parsing"""
        self.assertEqual(
            self.parse_all('cmd("simple string")'),
            [("cmd", ["simple string"])]
        )
        self.assertEqual(
            self.parse_all('cmd("string with \\"quote\\"")'),
            [("cmd", ['string with "quote"'])]
        )
        self.assertEqual(
            self.parse_all('cmd("string that spans\nmultiple lines")'),
            [("cmd", ['string that spans\nmultiple lines'])]
        )
        self.assertEqual(
            self.parse_all('cmd("\\\\"\\")'),
            [("cmd", ['\\', '"'])]
        )

    def test_macro(self):
        """Test CMake parser macro expansion"""
        self.assertEqual(
            self.parse_all("macro(test) cmd() endmacro() test()"),
            [("macro", ["test"]), ("endmacro", []), ("cmd", [])]
        )
        self.assertEqual(
            self.parse_all("macro(test) cmd() test() endmacro() test()"),
            [("macro", ["test"]), ("endmacro", []), ("cmd", [])]
        )
        self.assertEqual(
            self.parse_all("macro(test) cmd(${global}) test() endmacro() test()", {"global": "value"}),
            [("macro", ["test"]), ("endmacro", []), ("cmd", ["value"])]
        )
        self.assertEqual(
            self.parse_all("macro(test arg) cmd(${arg}) endmacro() test(fun)"),
            [("macro", ["test", "arg"]), ("endmacro", []), ("cmd", ["fun"])]
        )
        self.assertEqual(
            self.parse_all("macro(test arg) cmd(${arg}) endmacro() test(local) cmd(${arg})", {"arg": "global"}),
            [("macro", ["test", "arg"]), ("endmacro", []), ("cmd", ["local"]), ("cmd", ["global"])]
        )
        self.assertEqual(
            self.parse_all('macro(test arg) cmd(${arg}) endmacro() test("one;two;three")'),
            [("macro", ["test", "arg"]), ("endmacro", []), ("cmd", ["one", "two", "three"])]
        )
        self.assertEqual(
            self.parse_all('macro(test arg) cmd(${arg}) cmd(${ARGN}) endmacro() test(one;two;three)'),
            [("macro", ["test", "arg"]), ("endmacro", []), ("cmd", ["one"]), ("cmd", ["two", "three"])]
        )
        self.assertEqual(
            self.parse_all('macro(test arg1 arg2) cmd("${arg2}") cmd(${ARGN}) endmacro() test(one)'),
            [("macro", ["test", "arg1", "arg2"]), ("endmacro", []), ("cmd", [""]), ("cmd", [])]
        )
        self.assertEqual(
            self.parse_all('macro(test arg) cmd("${arg}") endmacro() test("one;two;three")'),
            [("macro", ["test", "arg"]), ("endmacro", []), ("cmd", ["one;two;three"])]
        )
        self.assertEqual(
            self.parse_all('macro(test arg) cmd(${arg} ${ARGN}) endmacro() test(arg extra stuff)'),
            [("macro", ["test", "arg"]), ("endmacro", []), ("cmd", ["arg", "extra", "stuff"])]
        )
        self.assertEqual(
            self.parse_all('macro(TEST arg) cmd(${arg}) endmacro() test(value)'),
            [("macro", ["TEST", "arg"]), ("endmacro", []), ("cmd", ["value"])]
        )
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "macro() endmacro()")
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "macro(fun)")

    def test_function(self):
        """Test CMake parser function definitions"""
        self.assertEqual(
            self.parse_all("function(test) cmd() endfunction() test()"),
            [("function", ["test"]), ("endfunction", []), ("test", [])]
        )
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "function() endfunction()")
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "function(fun)")

    def test_foreach(self):
        """Test CMake parser foreach() loop handling"""
        self.assertEqual(
            self.parse_all('foreach(arg RANGE 2) cmd(${arg}) endforeach()'),
            [("foreach", ["arg", "RANGE", "2"]), ("cmd", ["0"]), ("cmd", ["1"]), ("cmd", ["2"]), ("endforeach", [])]
        )
        self.assertEqual(
            self.parse_all('foreach(arg RANGE 1 3) cmd(${arg}) endforeach()'),
            [("foreach", ["arg", "RANGE", "1", "3"]), ("cmd", ["1"]), ("cmd", ["2"]), ("cmd", ["3"]), ("endforeach", [])]
        )
        self.assertEqual(
            self.parse_all('foreach(arg RANGE 1 5 2) cmd(${arg}) endforeach()'),
            [("foreach", ["arg", "RANGE", "1", "5", "2"]), ("cmd", ["1"]), ("cmd", ["3"]), ("cmd", ["5"]), ("endforeach", [])]
        )
        self.assertEqual(
            self.parse_all('foreach(arg 1 2 3 4 5) endforeach()'),
            [("foreach", ["arg", "1", "2", "3", "4", "5"]), ("endforeach", [])]
        )
        self.assertEqual(
            self.parse_all('foreach(arg one) cmd(${global}) endforeach()', {"global": "value"}),
            [("foreach", ["arg", "one"]), ("cmd", ["value"]), ("endforeach", [])]
        )
        self.assertEqual(
            self.parse_all('foreach(arg IN LISTS dummy) cmd(${arg}) endforeach()', {"dummy": "one;two;three"}),
            [("foreach", ["arg", "IN", "LISTS", "dummy"]), ("cmd", ["one"]), ("cmd", ["two"]), ("cmd", ["three"]), ("endforeach", [])]
        )
        self.assertEqual(
            self.parse_all('foreach(arg IN ITEMS ${dummy}) cmd(${arg}) endforeach()', {"dummy": "one;two;three"}),
            [("foreach", ["arg", "IN", "ITEMS", "one", "two", "three"]), ("cmd", ["one"]), ("cmd", ["two"]), ("cmd", ["three"]), ("endforeach", [])]
        )
        self.assertEqual(
            self.parse_all('foreach(arg ${dummy}) cmd(${arg}) endforeach()', {"dummy": "one;two;three"}),
            [("foreach", ["arg", "one", "two", "three"]), ("cmd", ["one"]), ("cmd", ["two"]), ("cmd", ["three"]), ("endforeach", [])]
        )
        self.assertEqual(
            self.parse_all('foreach(arg) cmd(${arg}) endforeach()'),
            [("foreach", ["arg"]), ("endforeach", [])]
        )
        self.assertEqual(
            self.parse_all('foreach(a 1 2) foreach(b 3 4) cmd(${a} ${b}) endforeach() endforeach()'),
            [("foreach", ["a", "1", "2"]),
             ("foreach", ["b", "3", "4"]),
             ("cmd", ["1", "3"]),
             ("cmd", ["1", "4"]),
             ("endforeach", []),
             ("foreach", ["b", "3", "4"]),
             ("cmd", ["2", "3"]),
             ("cmd", ["2", "4"]),
             ("endforeach", []),
             ("endforeach", [])]
        )
        self.assertEqual(
            self.parse_all('FOREACH(a 1 2) FOREACH(b 3 4) cmd(${a} ${b}) ENDFOREACH() ENDFOREACH()'),
            [("FOREACH", ["a", "1", "2"]),
             ("FOREACH", ["b", "3", "4"]),
             ("cmd", ["1", "3"]),
             ("cmd", ["1", "4"]),
             ("ENDFOREACH", []),
             ("FOREACH", ["b", "3", "4"]),
             ("cmd", ["2", "3"]),
             ("cmd", ["2", "4"]),
             ("ENDFOREACH", []),
             ("ENDFOREACH", [])]
        )
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "foreach(arg)")
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "foreach(arg RANGE bla) endforeach()")
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "foreach(arg RANGE 1 5 2 0) endforeach()")
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, "foreach() endforeach()")

    def test_arguments(self):
        """Test CMake parser argument parsing"""
        self.assertEqual(
            self.parse_all("cmd(one two three)"),
            [("cmd", ["one", "two", "three"])]
        )
        self.assertEqual(
            self.parse_all("cmd(one two;three)"),
            [("cmd", ["one", "two", "three"])]
        )
        self.assertEqual(
            self.parse_all("cmd(one;two;three)"),
            [("cmd", ["one", "two", "three"])]
        )
        self.assertEqual(
            self.parse_all("cmd(one;two three)"),
            [("cmd", ["one", "two", "three"])]
        )
        self.assertEqual(
            self.parse_all('cmd("one;two" three)'),
            [("cmd", ["one;two", "three"])]
        )
        self.assertEqual(
            self.parse_all('cmd("one;two";three)'),
            [("cmd", ["one;two", "three"])]
        )
        self.assertEqual(
            self.parse_all('cmd(one;"two;three")'),
            [("cmd", ["one", "two;three"])]
        )
        self.assertEqual(
            self.parse_all('if(NOT (A OR B)) endif()'),
            [("if", ["NOT", "(", "A", "OR", "B", ")"]), ("endif", [])]
        )
        self.assertEqual(
            self.parse_all('cmd("(")'),
            [("cmd", ["("])]
        )
        self.assertEqual(
            self.parse_all('cmd(")")'),
            [("cmd", [")"])]
        )
        self.assertEqual(
            self.parse_all('cmd("\\"")'),
            [("cmd", ['"'])]
        )
        self.assertEqual(
            self.parse_all('cmd(\\")'),
            [("cmd", ['"'])]
        )
        self.assertEqual(
            self.parse_all('cmd(a\\ b)'),
            [("cmd", ['a b'])]
        )
        self.assertEqual(
            self.parse_all("cmd(ENV{PATH})"),
            [("cmd", ["ENV{PATH}"])]
        )
        self.assertRaises(cmake.CMakeSyntaxError, self.parse_all, 'cmd("unclosed string)')

    def test_substitution(self):
        """Test CMake parser variable substitution semantics"""
        self.assertEqual(
            self.parse_all("cmd(${args})", var={"args": "one;two;three"}),
            [("cmd", ["one", "two", "three"])]
        )
        self.assertEqual(
            self.parse_all("cmd(${missing})"),
            [("cmd", [])]
        )
        self.assertEqual(
            self.parse_all('cmd("${missing}")'),
            [("cmd", [""])]
        )
        self.assertEqual(
            self.parse_all("${fun}()", var={"fun": "cmd"}),
            [("cmd", [])]
        )
        self.assertEqual(
            self.parse_all("cmd(${args})", var={"args": "one two three"}),
            [("cmd", ["one two three"])]
        )
        self.assertEqual(
            self.parse_all('cmd("${args}")', var={"args": "one;two;three"}),
            [("cmd", ["one;two;three"])]
        )
        self.assertEqual(
            self.parse_all('cmd("\\${args}")', var={"args": "fail"}),
            [("cmd", ["${args}"])]
        )
        self.assertEqual(
            self.parse_all('cmd(\\${args})', var={"args": "fail"}),
            [("cmd", ["${args}"])]
        )
        self.assertEqual(
            self.parse_all('cmd(${args})', var={"args": "\\\\"}),
            [("cmd", ["\\\\"])]
        )
        self.assertEqual(
            self.parse_all('cmd(${args})', var={"args": "${looks_like_a_variable}"}),
            [("cmd", ["${looks_like_a_variable}"])]
        )
        self.assertEqual(
            self.parse_all('cmd(${args})', var={"args": ")"}),
            [("cmd", [")"])]
        )
        self.assertEqual(
            self.parse_all('cmd(fun ${args})', var={"args": "stuff"}),
            [("cmd", ["fun", "stuff"])]
        )
        self.assertEqual(
            self.parse_all("cmd($ENV{PATH})"),
            [("cmd", ["$ENV{PATH}"])]
        )
        self.assertEqual(
            self.parse_all("cmd($env{test})", env_var={"test": "foo"}),
            [("cmd", ["$env{test}"])]
        )
        self.assertEqual(
            self.parse_all("cmd($ENV{test})", env_var={"test": "foo"}),
            [("cmd", ["foo"])]
        )
        self.assertEqual(
            self.parse_all("cmd($ENV{Test})", env_var={"test": "foo"}),
            [("cmd", ["$ENV{Test}"])]
        )

    def test_pragma(self):
        """Test CMake parser catkin_lint pragmas"""
        self.assertEqual(
            self.parse_all("# catkin_lint: extra space\n#catkin_lint:\n#catkin_lint:   \n#catkin_lint:   one   two   three   \n#catkin_lint :\n"),
            [("#catkin_lint", []), ("#catkin_lint", []), ("#catkin_lint", ["one", "two", "three"])]
        )

    def test_skip_block(self):
        """Test CMaker parser skip block"""
        self.assertEqual(
            self.parse_all("""\
            #catkin_lint: skip
            cmd()
            """),
            [("cmd", [])]
        )
        self.assertEqual(
            self.parse_all("""\
            if()
            endif()
            #catkin_lint: skip
            cmd()
            """),
            [("if", []), ("endif", []), ("cmd", [])]
        )
        self.assertEqual(
            self.parse_all("""\
            #catkin_lint: skip
            if(test)
            endif()
            """),
            [("if", ["test"]), ("endif", [])]
        )
        self.assertEqual(
            self.parse_all("""\
            if(condition) #catkin_lint: skip
                cmd()
            endif()
            """),
            [("if", ["condition"]), ("endif", [])]
        )
        self.assertEqual(
            self.parse_all("""\
            if(condition) #catkin_lint: skip
                cmd1()
            else()
                cmd2()
            endif()
            """),
            [("if", ["condition"]), ("else", []), ("cmd2", []), ("endif", [])]
        )
        self.assertEqual(
            self.parse_all("""\
            if(condition)
                cmd1()
            else() #catkin_lint: skip
                cmd2()
            endif()
            """),
            [("if", ["condition"]), ("cmd1", []), ("else", []), ("endif", [])]
        )
        self.assertEqual(
            self.parse_all("""\
            foreach(arg 1 2)
                cmd(${arg})
                #catkin_lint: skip
                do_not_parse_this()
            endforeach()
            """),
            [("foreach", ["arg", "1", "2"]), ("cmd", ["1"]), ("cmd", ["2"]), ("endforeach", [])]
        )
        self.assertEqual(
            self.parse_all("""\
            macro(test)
                cmd()
                #catkin_lint: skip
                do_not_parse_this()
            endmacro()
            test()
            """),
            [("macro", ["test"]), ("endmacro", []), ("cmd", [])]
        )

        self.assertEqual(
            self.parse_all("""\
            if(first) #catkin_lint: skip
                if(second)
                endif(second)
                do_not_parse_this()
            endif()
            """),
            [("if", ["first"]), ("endif", [])]
        )
        self.assertEqual(
            self.parse_all("""\
            if(first) #catkin_lint: skip
                foreach(second 1 2)
                endforeach()
                do_not_parse_this()
            endif()
            """),
            [("if", ["first"]), ("endif", [])]
        )
        self.assertEqual(
            self.parse_all("""\
            macro(test)
                do_not_parse_this()
            endmacro()
            if(first) #catkin_lint: skip
                test()
                do_not_parse_this()
            endif()
            """),
            [("macro", ["test"]), ("endmacro", []), ("if", ["first"]), ("endif", [])]
        )

    def test_comments(self):
        """Test CMake parser comment handling"""
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
            [("cmd", ["one", "two", "three"])]
        )

    def test_line_numbering(self):
        """Test CMake parser line numbering"""
        self.assertEqual(
            self.parse_all("""\
            cmd1()
            cmd2(
            )
            # Comment
            cmd3()
            """, location=1),
            [("cmd1", [], 1), ("cmd2", [], 2), ("cmd3", [], 5)]
        )
        self.assertEqual(
            self.parse_all("cmd1()\rcmd2()\rcmd3()\r", location=1),
            [("cmd1", [], 1), ("cmd2", [], 2), ("cmd3", [], 3)]
        )
        self.assertEqual(
            self.parse_all("cmd1()\ncmd2()\ncmd3()\n", location=1),
            [("cmd1", [], 1), ("cmd2", [], 2), ("cmd3", [], 3)]
        )
        self.assertEqual(
            self.parse_all("cmd1()\r\ncmd2()\r\ncmd3()\r\n", location=1),
            [("cmd1", [], 1), ("cmd2", [], 2), ("cmd3", [], 3)]
        )

    def test_line_columns(self):
        """Test CMake parser column numbering"""
        self.assertEqual(
            self.parse_all("cmd1()\n cmd2()\n  cmd3()\n", location=2),
            [("cmd1", [], 1, 1), ("cmd2", [], 2, 2), ("cmd3", [], 3, 3)]
        )

    def test_argparse(self):
        """Test CMake parser argparse utility function"""
        self.assertRaises(RuntimeError, cmake.argparse, [], {"TEST": "xxx"})

        opts, args = cmake.argparse([], {})
        self.assertEqual({}, opts)
        self.assertEqual([], args)

        opts, args = cmake.argparse([], {"TEST": "-"})
        self.assertEqual({"TEST": False}, opts)
        self.assertEqual([], args)

        opts, args = cmake.argparse([], {"TEST": "?"})
        self.assertEqual({"TEST": None}, opts)
        self.assertEqual([], args)

        self.assertRaises(cmake.CMakeSyntaxError, cmake.argparse, [], {"TEST": "!"})

        opts, args = cmake.argparse([], {"TEST": "*"})
        self.assertEqual({"TEST": []}, opts)
        self.assertEqual([], args)

        self.assertRaises(cmake.CMakeSyntaxError, cmake.argparse, [], {"TEST": "+"})

        opts, args = cmake.argparse([], {"TEST": "p"})
        self.assertEqual({"TEST": {}}, opts)
        self.assertEqual([], args)

        opts, args = cmake.argparse(["argument", "BOOL"], {"BOOL": "-"})
        self.assertEqual({"BOOL": True}, opts)
        self.assertEqual(["argument"], args)

        opts, args = cmake.argparse(["argument", "KEY", "value"], {"KEY": "?"})
        self.assertEqual({"KEY": "value"}, opts)
        self.assertEqual(["argument"], args)

        opts, args = cmake.argparse(["argument", "KEY", "value"], {"KEY": "!"})
        self.assertEqual({"KEY": "value"}, opts)
        self.assertEqual(["argument"], args)

        opts, args = cmake.argparse(["argument", "LIST", "value1", "value2"], {"LIST": "*"})
        self.assertEqual({"LIST": ["value1", "value2"]}, opts)
        self.assertEqual(["argument"], args)

        opts, args = cmake.argparse(["argument", "LIST", "value1", "value2"], {"LIST": "+"})
        self.assertEqual({"LIST": ["value1", "value2"]}, opts)
        self.assertEqual(["argument"], args)

        opts, args = cmake.argparse(["argument", "PROPERTIES", "key1", "value1", "key2", "value2"], {"PROPERTIES": "p"})
        self.assertEqual({"PROPERTIES": {"key1": "value1", "key2": "value2"}}, opts)
        self.assertEqual(["argument"], args)

        opts, args = cmake.argparse(["PROPERTIES", "key1", "value1", "key2"], {"PROPERTIES": "p"})
        self.assertEqual({"PROPERTIES": {"key1": "value1", "key2": ""}}, opts)

        opts, args = cmake.argparse(["DOUBLE", "DOUBLE", "ARGUMENT", "ARGUMENT"], {"DOUBLE ARGUMENT": "?"})
        self.assertEqual({"DOUBLE ARGUMENT": "ARGUMENT"}, opts)
        self.assertEqual(["DOUBLE"], args)

        opts, args = cmake.argparse(["BOOL", "argument"], {"BOOL": "-"})
        self.assertEqual({"BOOL": True}, opts)
        self.assertEqual(["argument"], args)
