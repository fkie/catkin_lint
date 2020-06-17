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
from .helper import create_env, create_manifest, mock_lint


class IndentationTest(unittest.TestCase):

    def test_regular(self):
        """Test indentation check for regular command sequences"""
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
        """Test indentation check for sequences with macro calls"""
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
                macro(test4)
                    cmd()
                    if()
                        cmd()
                    endif()
                endmacro()
                macro(test3)
                    test4()
                endmacro()
                macro(test2)
                    test3()
                    if()
                        if()
                            if()
                                cmd()
                                test3()
                            endif()
                        endif()
                    endif()
                endmacro()
                macro(test)
                    test2()
                    if()
                        cmd()
                        test2()
                    else()
                        foreach(a b c d e)
                            test2()
                        endforeach()
                    endif()
                endmacro()
                cmd1()
                test()
                cmd3()
            """, checks=None, indentation=True)

    def test_if(self):
        """Test indentation check for if()/else()/endif() blocks"""
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
        """Test indentation check for foreach()/endforeach() blocks"""
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
