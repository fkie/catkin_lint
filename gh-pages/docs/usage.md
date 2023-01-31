# Usage

**catkin_lint** is a command-line tool. You can invoke it manually or as part
of a script.

## Synopsis

**catkin\_lint** \[**--config** _FILE_\] \[**--quiet** | **--no-quiet**\]
            \[**--severity-level** _LEVEL_\]
            \[**--notice** _ID_\] \[**--warning** _ID_\] \[**--error** _ID_\]
            \[**--ignore** _ID_\] \[**--show-ignored**\]
            \[**--strict** | **--no-strict**\]
            \[**--pkg** _PKG_\] \[**--skip-pkg** _PKG_\]
            \[**--package-path** _PATH_\] \[**--skip-path** _MATCH_\]
            \[**--rosdistro** _DISTRO_\] \[**--offline**\] \[**--no-offline**\]
            \[**--rosdep-cache-path** _PATH_\]
            \[**--resolve-env** | **--no-resolve-env**\]
            \[**--output** _FORMAT_ | **--text** | **--explain** | **--xml** | **--json**\]
            \[**--color** _MODE_\]
            \[_PATH_ \[_PATH_ ...\]\]


## Options

- **--config** _FILE_

    Read [configuration settings](#configuration-file-format) from _FILE_.
    This option can be used multiple times.

- **--quiet**, **-q**

    Suppress all outputs expect for the detected problems, in particular, do not show
    the summary at the end.

- **--no-quiet**

    Show a summary of the findings at the end. This is the default setting.

- **--severity-level** _LEVEL_, **-W** _LEVEL_

    Choose the [severity level](index.md#diagnostic-levels) for diagnostic output.
    Level 0 displays errors only, level 1 displays errors and warnings, and level 2
    displays everything. The default setting is **-W1**.

- **--notice** _ID_, **--warning** _ID_, **--error** _ID_

    Override the default severity of certain [problems](messages.md)

- **--ignore** _ID_

    Ignore a [problem](messages.md). This is mostly useful to work around a known bug in **catkin\_lint**,
    because problems tend to not go away if you ignore them.

- **--show-ignored**

    Show all problems you ignored. Use this if an ignored problem did not go away, but
    you forgot which one.

- **--strict**

    Treat any reported problem as a fatal error. Some people use the option to enforce
    that warnings get fixed, too. You can also combine this with **-W2**
    to turn even notices into errors, if you are exceptionally pedantic and/or have a
    high pain tolerance.

- **--no-strict**

    This will undo the effects of your rash decision to put `strict=yes` in your
    configuration file. This is also the default setting.

- **--pkg** _PKG_

    Check the package _PKG_ for problems. The package must be in the **ROS** package search
    path.

- **--skip-pkg** _PKG_

    Skip the package _PKG_ when processing a package subtree. The package will not be linted,
    but it will be treated as a known package for dependency resolutions.

- **--package-path** _PATH_\[:_PATH_\[...\]\]

    Add one or more additional locations to the **ROS** package search path. Separate multiple
    locations with `:`. **catkin\_lint** will use this path list and the **ROS\_PACKAGE\_PATH**
    environment variable to resolve the **--pkg** option and dependencies of linted packages.

- **--skip-path** _MATCH_

    Skip packages if their location contains _MATCH_. No wildcards or pattern matching allowed,
    only proper substrings are recognized.

- **--rosdistro** _DISTRO_

    Assume that all packages are supposed to work with the **ROS** distribution _DISTRO_. By default, this
    value is taken from the **ROS\_DISTRO** environment variable. **catkin\_lint** will use this to
    query the **ROS** database for packages which are not locally installed.

- **--offline**

    Forbid metadata queries to the **ROS** package index. This will disable certain diagnostics which
    rely on knowing details about all package dependencies. Metadata queries are not needed for
    packages which can be found locally through the **ROS** package search path.

- **--no-offline**

    Allow metadata queries to the **ROS** package index. This is the default.

- **--rosdep-cache-path** _PATH_

    Override the default location of the rosdep sources cache. This option has the same
    effect as setting the **ROSDEP_CACHE_PATH** environment variable.

- **--resolve-env**

    Resolve environment variables **$ENV{}** in CMake scripts. By default, **catkin\_lint** will treat
    all environment variables like empty strings and issue a warning.

- **--no-resolve-env**

    Do not resolve environment variables **$ENV{}** in CMake scripts and issue a warning if they are
    used. This is the default.

- **--output** _FORMAT_

    Select the [output format](#output-formats) for the diagnosed problems.
    Valid formats are **text**, **explain**, **json**, and **xml**.

- **--text**, **--explain**, **--json**, **--xml**

    These are deprecated aliases for **--output** _FORMAT_.

- **--color** _MODE_

    Configure the colorization of the **text** and **explain** output formats. The default mode is **auto**,
    which will use ANSI colors if **catkin\_lint** detects a terminal as output. The modes **always** and
    **never** will override the detection and output ANSI colors always or never, respectively.

## Output Formats

**catkin\_lint** supports a variety of output formats as argument for the **--output** option:

- **text**

    This is the default format, which outputs a short, human-readable description of all problems.

- **explain**

    This is an extended **text** format, which will show an additional explanation when a problem type
    occurs for the first time. It will also mention the message ID which you need to change the
    severity or ignore a problem.

- **json**

    Outputs all problems in JSON format.

- **xml**

    Outputs all problems in XML format.

## Environment

**catkin\_lint** uses **ROS\_PACKAGE\_PATH** to find locally available packages and **ROS\_DISTRO** to 
determine the installed **ROS** distribution when querying the **ROS** package index or the **rosdep** database.
All other environment variables will be ignored, unless **catkin\_lint** is instructed to resolve
them in CMake scripts by **--resolve-env**.

## Return Values

- **0**

    All packages were linted successfully and no errors occurred.

- **1**

    An error occurred.

- **66**

    No packages found.

## Configuration File Format

**catkin\_lint** will look for configuration options in the following files:

- Files explicitly specified using **--config**
- `.catkin_lint` files in all locations from **ROS\_PACKAGE\_PATH**
- `$XDG_CONFIG_HOME/catkin_lint`
- `~/.config/catkin_lint` if **XDG\_CONFIG\_HOME** is unset or empty
- `~/.catkin_lint`

**catkin\_lint** will read all available configuration files and merge them into
a single configuration. Earlier entries in the above list will take precedence
if conflicting options are found. Command line arguments will override any
configuration file setting. Configuration files are in an INI-style format with
one or more sections in it.

### \[catkin\_lint\] section

The main section is called `[catkin_lint]` and will take the following 
**catkin\_lint** specific options:

- **color** = **auto** | **never** | **always**

    Determine output colorization mode. The default setting **auto** will enable
    colorization if and only if standard output is connected to terminal. The modes **always**
    and **never** will override the detection and output ANSI colors always or never,
    respectively.

- **offline** = **no** | **yes**

    Allow or forbid metadata queries to the **ROS** package index. This will disable certain
    diagnostics which rely on knowing details about all package dependencies. Metadata
    queries are not needed for packages which can be found locally through the **ROS**
    package search path. The default setting is **no**.

- **output** = **text** | **explain** | **json** | **xml**

    Select the [output format](#output-formats) for problem reports.
    The default setting is **text**.

- **package\_path** = _PATH_\[:_PATH_\[...\]\]

    Prepend the specified _PATH_s to **ROS\_PACKAGE\_PATH** in order to find locally
    available packages and/or package dependencies.

- **quiet** = **no** | **yes**

    Select output verbosity. The default setting **no** allows **catkin\_lint** to print
    some additional information to standard error, such as a final summary of all
    detected problems.

- **resolve\_env** = **no** | **yes**

    Allow or forbid resolution of environment variables. The default setting **no**
    lets **catkin\_lint** ignore environment variables and issue a warning whenever
    a CMake command references the environment using **$ENV{}**.

- **rosdistro** = _DISTRO_

    Assume that all packages are supposed to work with the **ROS** distribution _DISTRO_.
    By default, this value is taken from the **ROS\_DISTRO** environment variable.
    **catkin\_lint** will use this to query the **ROS** database for packages which are not
    locally installed.

- **rosdep\_cache\_path** = _PATH_

    Override the default location of the rosdep sources cache. This option has the same
    effect as setting the **ROSDEP_CACHE_PATH** environment variable.

- **severity\_level** = **0** | **1** | **2**

    Choose the [severity level](index.md#diagnostic-levels) for diagnostic output.
    Level 0 displays errors only, level 1 displays errors and warnings, and level 2
    displays everything. The default setting is **1**.

- **strict** = **no** | **yes**

    In strict mode, **catkin\_lint** will treat every reported problem as error and return
    with [exit code](#return-values) 1. By default, warnings and notices are
    informational only. This setting is mostly interesting for automated test runs,
    where the exit code is evaluated. Note that ignored problems and everything hidden
    by the chosen **severity\_level** will not be considered failures.

### Package-specific severity overrides

All section names besides `[catkin_lint]` are interpreted as package names and
will override the reported severities of [problems](messages.md). As a special case, the
section `[*]` will apply to all packages. The overrides are specified in the
format `ID = Severity`, where the severity can be one of
**ignore**, **error**, **warning**, **notice**, or **default**.

Unlike **--strict**, this gives you very fine-grained control over which problems
are supposed to be fatal, so you are encouraged to integrate **catkin\_lint** into
your CI test pipeline and tune the settings in a way that fits your project.

## Examples

### Command line arguments

- **catkin\_lint ~/catkin\_ws/src**

    Check all packages in `~/catkin_ws/src` for problems.

- **catkin\_lint --pkg foo --pkg bar**

    Check the packages `foo` and `bar` for problems, assuming that
    they can be found in one of the locations listed in **ROS\_PACKAGE\_PATH**.

- **catkin\_lint --pkg foo --pkg bar --package-path ~/my\_other\_ws/src**

    Check the packages `foo` and `bar` for problems when the
    former assumption turned out to be false.

- **catkin\_lint ~/catkin\_ws/src --skip-path unstable**

    Check all packages in `~/catkin_ws/src`, but skip all packages in a path
    that contains the string `unstable`.

- **catkin\_lint ~/catkin\_ws/src --skip-pkg baz**

    Check all packages except `baz` in `~/catkin_ws/src`.

### Configuration file

The following configuration file will instruct **catkin\_lint** to output its results
in JSON format, ignore any problems with unknown packages (except in the package foo),
and elevate the notice about unsorted lists to a warning for all packages (including foo):

    [catkin_lint]
    output = json
    quiet = yes

    [*]
    unknown_package = ignore
    unsorted_list = warning

    [foo]
    unknown_package = default
