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
import os
from ..linter import ERROR
from .misc import project


def setup(linter):
    def on_catkin_python_setup(info, cmd, args):
        if not "catkin" in info.find_packages and not info.is_catkin:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)
        if "generate_messages" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="generate_messages", second_cmd=cmd)
        if not os.path.isfile(os.path.join(info.path, "setup.py")):
            info.report(ERROR, "MISSING_FILE", cmd=cmd, file="setup.py")
    def on_final(info):
        if not "catkin_python_setup" in info.commands and os.path.isfile(os.path.join(info.path, "setup.py")):
            info.report(ERROR, "MISSING_PYTHON_SETUP")

    linter.require(project)
    linter.add_command_hook("catkin_python_setup", on_catkin_python_setup)
    linter.add_final_hook(on_final)


def all(linter):
    linter.require(setup)
