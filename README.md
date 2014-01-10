
catkin\_lint
============

[![Build Status](https://travis-ci.org/fkie/catkin_lint.png?branch=bleeding-edge)](https://travis-ci.org/fkie/catkin_lint)

## Overview

**catkin\_lint** is a tool to debug package configurations for the
[ROS Catkin](https://github.com/ros/catkin) build system. It is part of
an ongoing effort to simplify ROS packaging
(see also: [issue #153](https://github.com/ros/catkin/issues/153)).

## Installation

**catkin\_lint** is available as PyPI package or can be installed from source
with `setup.py install`.

## Running

**catkin\_lint** runs a static analysis of the `package.xml` and
`CMakeLists.txt` files. It can detect and report a number of common
problems.

If **catkin\_lint** is invoked with one or more paths as parameters, it
searches for packages recursively and checks all of them. Alternatively, the
`--pkg` option can be used to add the path of a particular ROS package.

If neither paths nor packages are specified, **catkin\_lint** looks for a
package in the current working directory.

A more detailed list of command line options can be obtained by running
`catkin_lint --help`.

## Catkin Build Integration

It is recommended to run **catkin\_lint** at workspace configuration time.
The simplest way is to symlink `/opt/ros/$ROS_DISTRO/share/catkin/cmake/toplevel.cmake`
to your `$WSDIR/src` folder manually (do not use `catkin_init_workspace`).
Then add the following `CMakeLists.txt`:

    cmake_minimum_required(VERSION 2.8.3)
    execute_process(COMMAND catkin_lint "${CMAKE_SOURCE_DIR}" RESULT_VARIABLE lint_result)
    if(NOT ${lint_result} EQUAL 0)
        message(FATAL_ERROR "catkin_lint failed")
    endif()
    include(toplevel.cmake)

## Problem Severities

Diagnostic messages come in three different categories:
errors, warnings, and notices. The `-W` option controls which problems
are reported to the user:

- `-W0`: only errors are reported (this is the default)
- `-W1`: errors and warnings are reported
- `-W2`: errors, warnings, and notices are reported

Normally, **catkin\_lint** returns a non-zero exit code if and only
if errors occured. The `--strict` option causes **catkin\_lint** to
treat any reported problem as error. The `--explain` option offers
a more detailed explanation for each problem. The explanation is
not repeated if a problem occurs multiple times in different contexts.

### Errors

Errors are severe enough to either break the build or produce unintended
results. Usually, they violate the rules outlined in the
[Catkin Manual](http://docs.ros.org/api/catkin/html/).

### Warnings

Potential errors which may indicate a bug in your package but may be
justified for reasons **catkin\_lint** cannot discern. Constructs which
trigger a warning can usually be modified in a way that is functionally
equivalent but more robust.

### Notices

Issues which are not objectionable from a technical view point but
should  be addressed to improve the quality of the package. Many notices
highlight violations of the recommendations and best practises from the
Catkin Manual.

## License

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

