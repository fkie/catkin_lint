# catkin_lint

## Overview

**catkin_lint** checks package configurations for the
[catkin](https://github.com/ros/catkin) build system of
[ROS](http://www.ros.org>). It runs a static analysis of the `package.xml`
and `CMakeLists.txt` files in your package, and it will detect and report a
number of common problems.

## Installation

### Ubuntu

Prebuilt packages are available from the official Ubuntu archive and the
[ROS repository](http://packages.ros.org/).
If you are using ROS Noetic on Ubuntu 20.04 or later, install **catkin_lint** with
```sh
$ sudo apt install catkin-lint
```

!!! note
    Starting with Ubuntu 20.04, the package has been renamed. If you are using an older
    release, please run `sudo apt install python-catkin-lint` instead.


Alternatively, you can use [Timo's Ubuntu PPA for ROS Packages](https://launchpad.net/~roehling/+archive/ros) on Launchpad,
which will always ship the latest release:
```sh
$ sudo add-apt-repository ppa:roehling/ros
$ sudo apt update
$ sudo apt install catkin-lint
```

### Debian

Prebuilt packages are available from the official Debian archive. Install with
```sh
$ sudo apt install catkin-lint
```

!!! note
    For Debian Buster, the package is named `python-catkin-lint`.

### Download from PyPI

You can download and install **catkin_lint** from the [Python Package Index](https://pypi.python.org/pypi/catkin_lint)
with:
```sh
$ sudo pip install catkin-lint
```

### Install from Source

You can clone **catkin_lint** from [GitHub](https://github.com/fkie/catkin_lint):
```sh
$ git clone https://github.com/fkie/catkin_lint.git
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

## Configuration files

Since version 1.6.9, **catkin_lint** can load settings from a configuration file.
Configuration files can be specified with the `--config` command line options.
Additionally, **catkin_lint** will look in the following locations (in precedence order):

 * `.catkin_lint` files in the `ROS_PACKAGE_PATH` directories
 * `$XDG_CONFIG_HOME/catkin_lint` (`$XDG_CONFIG_HOME` defaults to `~/.config`)
 * `~/.catkin_lint`

The file is expected to be in an INI-style format with different sections.
The `[catkin_lint]` section may contain command line options.
All other sections are considered package names and will override the
severity of diagnostic messages.
The special wildcard section `[*]` applies to all packages.

The following example illustrates the configuration file format:

    [catkin_lint]
    output = explain
    color = auto
    package_path = /path/to/extra/packages

    [foo]
    missing_catkin_depend = warning
    unknown_package = ignore


    [bar]
    deprecated_cmd = default

    [*]
    deprecated_cmd = error
    launch_depend = notice

Note that the `deprecated_cmd` override from the wildcard section applies to the package `foo`,
but not to the package `bar`.
