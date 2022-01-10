#!/usr/bin/python3
# coding=utf-8
#
# catkin_lint
# Copyright 2013-2022 Fraunhofer FKIE
#
# SPDX-License-Identifier: BSD-3-Clause
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
#  * Neither the name of the copyright holder nor the names of its
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

import sys
import os
import re

srcpath = os.path.normpath(os.path.join(os.path.dirname(__file__), os.pardir, "src"))
sys.path.insert(0, srcpath)

if __name__ == "__main__":
    from catkin_lint.diagnostics import message_list
    severity = {}
    for curdir, _, files in os.walk(os.path.join(srcpath, "catkin_lint")):
        for fn in files:
            if fn.endswith(".py"):
                with open(os.path.join(curdir, fn), "r") as f:
                    for line in f.readlines():
                        m = re.search(r'info.report\((.*?), "(.*?)"', line)
                        if m:
                            if m.group(2) not in severity:
                                severity[m.group(2)] = set()
                            for s in ["ERROR", "WARNING", "NOTICE"]:
                                if s in m.group(1):
                                    severity[m.group(2)].add(s.lower())
    with open(os.path.join(os.path.dirname(__file__), "docs", "messages.md"), "w") as f:
        f.write("""\
# catkin_lint diagnostic messages

This is a list of all messages which might be shown by **catkin_lint**.
Each problem has a unique ID (such as *catkin_order_violation*),
which you can use to disable certain messages, either with the command line option
`--ignore ID`, or by adding a pragma line `#catkin_lint: ignore ID` at the beginning
of the CMakeLists.txt file. As a third option, you can add a pragma line `#catkin_lint: ignore_once ID`
right before the offending statement. Use this if you want to ignore a particular instance
of a problem but still be notified if the same problem occurs someplace else. You may
also use `#catkin_lint: report ID` at any point to override a previous `ignore`.

Since version 1.5.4, you may also customize the severity with the command line options
`--error ID`, `--warning ID`, or `--notice ID`. You can also add the pragma line
`#catkin_lint: skip` in any `if()`, `foreach()`, or `macro()` block, which will instruct
the parser to ignore all remaining commands in the block until the `else()`, `endif()`,
`endforeach()`, or `endmacro()` statement.

""")

        messages = {}
        for key in sorted(message_list.keys()):
            if key not in severity:
                print("Warning: unused message '%s'" % key)
                continue
            short_text, long_text = message_list[key]
            long_text = long_text.replace("\n", " ")
            long_text = long_text.replace("catkin_lint", "**catkin_lint**")
            short_text = re.sub(r"%\((.*?)\)s", r"<i>\1</i>", short_text)
            long_text = re.sub(r"%\((.*?)\)s", r"<i>\1</i>", long_text)
            long_text = re.sub(r"([a-z_]+\(.*?\))", r"<code>\1</code>", long_text)
            long_text = re.sub(r" +", " ", long_text)
            long_text = long_text.strip()
            short_text = short_text.strip()
            messages[(short_text, key.lower())] = (long_text, ", ".join(sorted(severity[key], key=lambda x: {"error": 0, "warning": 1, "notice": 2}.get(x))))
        for msg, key in sorted(messages.keys()):
            long_text, severities = messages[(msg, key)]
            f.write("## %s\n\n" % msg)
            f.write("- **ID**: %s\n" % key)
            f.write("- **Severity**: %s\n" % severities)
            f.write("- **Explanation**: %s\n" % long_text)
            f.write("\n")
