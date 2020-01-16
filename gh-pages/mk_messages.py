#!/usr/bin/python

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
Each diagnostic has a unique ID (such as *catkin_order_violation*),
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
            messages[(short_text, key.lower())] = (long_text, ", ".join(list(severity[key])))
        for msg, key in sorted(messages.keys()):
            long_text, severities = messages[(msg, key)]
            f.write("## %s\n\n" % msg)
            f.write("- **ID**: %s\n" % key)
            f.write("- **Severity**: %s\n" % severities)
            f.write("- **Explanation**: %s\n" % long_text)
            f.write("\n")
