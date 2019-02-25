# catkin_lint

## Overview

**catkin_lint** checks package configurations for the
[catkin](https://github.com/ros/catkin) build system of
[ROS](http://www.ros.org>). It runs a static analysis of the `package.xml`
and `CMakeLists.txt` files in your package, and it will detect and report a
number of common problems.

## Installation

### Install Ubuntu packages

Prebuilt packages are available from the official Ubuntu archive and the
[ROS repository](http://packages.ros.org/).
If you have installed ROS already, downloading **catkin_lint** is as simple as:
```sh
$ sudo apt install python-catkin-lint
```

Alternatively, you can use [Timo's PPA](https://launchpad.net/~roehling/+archive/latest) on Launchpad,
which will always ship the latest release:
```sh
$ sudo add-apt-repository ppa:roehling/latest
$ sudo apt update
$ sudo apt install python-catkin-lint
```

### Install Debian packages

Prebuilt packages are available from the official Debian archive. Install with
```sh
$ sudo apt install python-catkin-lint
```

### Install Fedora packages

Prebuilt packages have been made available by [Ankur Sinha](https://fedoraproject.org/wiki/User:Ankursinha). Install with
```sh
$ sudo dnf install python-catkin_lint
```

### Download from PyPI

You can download and install **catkin_lint** from the [Python Package Index](https://pypi.python.org/pypi/catkin_lint)
with:
```sh
$ sudo pip install catkin_lint
```

### Install from Source

You can clone **catkin_lint** from [GitHub](https://github.com/fkie/catkin_lint):
```sh
$ git clone https://github.com/fkie/catkin_lint
$ cd catkin_lint
$ sudo python setup.py install
```

## Running

If **catkin_lint** is invoked with one or more paths as parameters, it
searches for packages recursively and checks all of them. Alternatively, the
`--pkg` option can be used to add the path of a particular ROS package.

If neither paths nor packages are specified, **catkin_lint** looks for a
package in the current working directory.

A more detailed list of command line options can be obtained by running
```sh
$ catkin_lint --help
```

## Limitations

**catkin_lint** works by emulating the way CMake processes your package
during a build. However, since it does not _really_ build anything,
the emulation is not perfect. For instance,

- **catkin_lint** does not evaluate boolean expressions in `if()` clauses
  There is some special purpose code to detect `if(CATKIN_ENABLE_TESTING)` blocks,
  but in general, **catkin_lint** will just execute all statements, even mutually exclusive
  `if()`/`else()` blocks.
- **catkin_lint** uses mock values for `find_package()`, `find_file()`, and `find_library()`
  calls. That means that those function calls will always succeed and "find" something.
- **catkin_lint** ignores `function()` definitions. It does, however, expand macros and
  `foreach()` loops.

## Diagnostic Levels

**catkin_lint** has [messages](messages.md) in three different categories:
errors, warnings, and notices. The `-W` option controls which problems
are reported to the user:

- `-W0`: only errors are reported
- `-W1`: errors and warnings are reported (this is the default)
- `-W2`: errors, warnings, and notices are reported

Normally, **catkin_lint** returns a non-zero exit code if and only
if errors occurred. The `--strict` option causes **catkin_lint** to
treat any reported problem as error. You can also customize the category
for particular diagnostics with `--error ID`, `--warning ID`, or
`--notice ID`. You can also ignore messages entirely with `--ignore ID`.

### Errors

Errors are severe enough to break the build and/or produce unintended
side effects. Usually, they violate the rules outlined in the
[catkin manual](http://docs.ros.org/api/catkin/html/).

### Warnings

Potential errors which may indicate a bug in your package but may be
justified for reasons **catkin_lint** cannot discern. Constructs which
trigger a warning can usually be modified in a way that is functionally
equivalent but more robust.

### Notices

Issues which are not objectionable from a technical view point but
should  be addressed to improve the quality of the package. Many notices
highlight violations of the recommendations and best practises from the
catkin manual.

