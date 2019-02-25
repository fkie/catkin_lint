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

import os
from ..linter import ERROR
from .misc import project


def setup(linter):
    def on_catkin_python_setup(info, cmd, args):
        if "catkin" not in info.find_packages and not info.is_catkin:
            info.report(ERROR, "CATKIN_ORDER_VIOLATION", cmd=cmd)
        if "generate_messages" in info.commands:
            info.report(ERROR, "ORDER_VIOLATION", first_cmd="generate_messages", second_cmd=cmd)
        if not info.is_existing_path("setup.py", check=os.path.isfile, require_source_folder=True):
            info.report(ERROR, "MISSING_FILE", cmd=cmd, file="setup.py")

    def on_final(info):
        if "catkin_python_setup" not in info.commands and info.is_existing_path("setup.py", check=os.path.isfile, require_source_folder=True):
            info.report(ERROR, "MISSING_PYTHON_SETUP", file_location=("CMakeLists.txt", 0))

    linter.require(project)
    linter.add_command_hook("catkin_python_setup", on_catkin_python_setup)
    linter.add_final_hook(on_final)


def all(linter):
    linter.require(setup)
