import unittest
from .helper import create_env, create_manifest, mock_lint

import sys
sys.stderr = sys.stdout
import os

class IndentationTest(unittest.TestCase):

    def test_regular(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
                cmd1()
                cmd2()
                cmd3()
            """, checks=None, indentation=True)
        self.assertEqual([], result)
        result = mock_lint(env, pkg,
            """
                cmd1()
                  cmd2()
                cmd3()
            """, checks=None, indentation=True)
        self.assertEqual(["INDENTATION"], result)
        result = mock_lint(env, pkg,
            """
                cmd1() cmd2()
            """, checks=None, indentation=True)
        self.assertEqual(["INDENTATION"], result)

    def test_macro(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
                macro(test)
                           cmd2()
                endmacro()
                cmd1()
                test()
                cmd3()
            """, checks=None, indentation=True)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
                macro(test)
                           if()
                        cmd()
                           endif()
                endmacro()
                cmd1()
                test()
                cmd3()
            """, checks=None, indentation=True)
        self.assertEqual(["INDENTATION"], result)

        result = mock_lint(env, pkg,
            """
                macro(test2)
                    cmd()
                endmacro()
                macro(test)
                           if()
                               cmd()
                               test2()
                               cmd()
                           endif()
                endmacro()
                cmd1()
                test()
                cmd3()
            """, checks=None, indentation=True)
        self.assertEqual([], result)
        result = mock_lint(env, pkg,
            """
                macro(test3)
                    cmd()
                    if()
                        cmd()
                    endif()
                endmacro()
                macro(test2)
                    test3()
                    if()
                        cmd()
                    endif()
                endmacro()
                macro(test)
                    test2()
                    if()
                        cmd()
                    endif()
                endmacro()
                cmd1()
                test()
                cmd3()
            """, checks=None, indentation=True)

    def test_if(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
                cmd()
                if()
                    cmd()
                    cmd()
                else()
                    cmd()
                    cmd()
                endif()
                cmd()
            """, checks=None, indentation=True)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
                if()
                else()
                endif()
            """, checks=None, indentation=True)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
                if()
                    if()
                    endif()
                else()
                    if()
                    endif()
                endif()
            """, checks=None, indentation=True)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
                if()
                cmd()
                cmd()
                endif()
            """, checks=None, indentation=True)
        self.assertEqual(["INDENTATION"], result)

        result = mock_lint(env, pkg,
            """
                if()
                    cmd()
                    cmd()
                    endif()
            """, checks=None, indentation=True)
        self.assertEqual(["INDENTATION"], result)

        result = mock_lint(env, pkg,
            """
                if()
                    cmd()
                    else()
                    cmd()
                endif()
            """, checks=None, indentation=True)
        self.assertEqual(["INDENTATION"], result)

        result = mock_lint(env, pkg,
            """
                if()
                    cmd()
                  else()
                    cmd()
                endif()
            """, checks=None, indentation=True)
        self.assertEqual(["INDENTATION"], result)

        result = mock_lint(env, pkg,
            """
                if()
            cmd()
                else()
                    cmd()
                endif()
            """, checks=None, indentation=True)
        self.assertEqual(["INDENTATION"], result)

    def test_foreach(self):
        env = create_env()
        pkg = create_manifest("mock")
        result = mock_lint(env, pkg,
            """
                cmd()
                foreach(a 1)
                    cmd()
                    cmd()
                endforeach()
            """, checks=None, indentation=True)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
                foreach(a 1)
                    cmd()
                cmd()
                endforeach()
            """, checks=None, indentation=True)
        self.assertEqual(["INDENTATION"], result)

        result = mock_lint(env, pkg,
            """
                foreach(a 1)
                endforeach()
            """, checks=None, indentation=True)
        self.assertEqual([], result)

        result = mock_lint(env, pkg,
            """
                foreach(a 1)
                cmd()
                endforeach()
            """, checks=None, indentation=True)
        self.assertEqual(["INDENTATION"], result)

