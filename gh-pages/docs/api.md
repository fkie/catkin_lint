# Register your own checks

**catkin_lint** checks are implemented as
submodules, and interact with the main program
via callbacks. You can write your own checks for
**catkin_lint** and run them with the `-c` command line option.
You can also add them to your `.catkin_lint` configuration file
to use them by default:

    [catkin_lint]
    extra_checks = my.customized.check my.other.check

Each check is included via an entry function which setups
the callbacks for the check. The entry function has a single
parameter `linter`. The Linter object provides the following
methods:

## require()

```python
linter.require(check_name)
```
Checks may depend on the results of other checks.
The `require` method ensures that `check_name`
is called exactly once. Circular dependencies of the
form `A->B->C->A` will be detected and cause an
exception. In particular, tests must not require themselves.

## register_message()

!!! note
    This method was added in version 1.6.6

```python
linter.register_message(msg_id, text, description=None)
```
Adds a custom message that can be issued with [`report()`](#report).
Message IDs must be unique. Valid characters are upper-case letters,
digits, and underscores only. Custom message IDs are also
required to start with `X_`, to avoid conflicts with built-in
IDs.

The text should be a short one-liner that concisely
describes the problem. The description can be a paragraph
or two for a more in-depth explanation. The description will reformat
line breaks automatically to adjust for different terminal widths.
Also, you can have placeholders of the form `%(keyword)s`.

## add_init_hook()

```python
def callback(info):
    ...

linter.add_init_hook(callback)
```
Registers an initialization hook that is called when
the lint check for a particular package begins.


## add_command_hook()

```python
def callback(info, cmd, args):
    ...

linter.add_command_hook(name, callback)
```
Registers a command hook that is called each time the
CMake parser encounters the command `name`. The command
name and a list of its arguments are passed to the callback.
All command names are converted to lower-case.


## add_final_hook()

```python
def callback(info):
    ...

linter.add_final_hook(callback)
```
Registers a final hook that is called when the CMake parser
has finished parsing the `CMakeLists.txt` file.


## execute_hook()

```python
linter.execute_hook(info, cmd, args)
```
Executes all registered command hooks for `cmd`. This is useful
for CMake wrapper macros, so you don't have to duplicate all checks
for the wrapped call. For example, the `cuda_add_executable`
command hook calls the `add_executable` hooks this way.


# Info Object

For each linted package, **catkin_lint** creates a
dedicated info object that is passed to all callbacks.
The info object can be used to store relevant data.
Typically, the init hook is used to initialize check-specific
variables in the info object. As a general rule, checks must not
modify variables they do not own, and each check must ensure it
uses unique variable names that do not conflict with other checks.

The following variables are defined by **catkin_lint** itself:

- `env`:
    A `CatkinEnvironment` object that provides information about
    the ROS environment.
- `path`:
    The filesystem path to the package source folder
- `manifest`:
    A `catkin_pkg.packages.Package` object that is created from
    information in the `package.xml`.
- `file`:
    The currently parsed CMake file, relative to the package source
    folder. Is either `CMakeLists.txt` or an include file. Only
    valid in command hooks.
- `line`:
    The line number of the currently processed CMake command. Only
    valid in command hooks.
- `commands`:
    A set of all command names that have been encountered up to this point.
- `find_packages`:
    A set of all packages that have been configured with `find_package()`
    up to this point.
- `targets`:
    A set of all make targets that have been defined up to this point.
- `executables`:
    A set of all executables that have been defined up to this point.
- `libraries`:
    A set of all libraries that have been defined up to this point.
- `var`:
    A dictionary of all known CMake variables. Note that many variables
    have mocked values.

## report()

```python
info.report(level, msg_id, **kwargs)
```
Reports a problem to the user. `level` must be one of
`catkin_lint.linter.ERROR`, `catkin_lint.linter.WARNING`, or
`catkin_lint.linter.NOTICE`. The `msg_id` refers to one
of the defined diagnostic messages. Certain messages have placeholder
variables that must be specified, e.g. `cmd` for the command name. If
you wish to report a path, you should sanitize it with `report_path()`
first.

!!! note
    Starting with version 1.6.0, you can target a report to a different
    file location with the `file_location` keyword argument.


## location_of()

!!! note
    This method was added in version 1.6.0

```python
info.location_of(cmd)
```
Returns the file location of the most recent invocation of the CMake
command `cmd`. This is useful if a problem is detected belatedly,
but should still be attributed to a particular faulty command.

## source_relative_path()

```python
info.source_relative_path(path)
```
Returns a path relative to the package source directory or
an absolute path if the path is not within the package. Can handle
`${CMAKE_CURRENT_SOURCE_DIR}` correctly.

## binary_relative_path()

```python
info.binary_relative_path(path)
```
Returns a path relative to the package build directory or
an absolute path if the path is not inside the build directory. Can handle
`${CMAKE_CURRENT_BINARY_DIR}` correctly.

## real_path()

```python
info.real_path(path)
```
Returns the actual file system path for relative package path as
returned by `source_relative_path()`.

## is_internal_path()

```python
info.is_internal_path(path)
```
Returns `True` if the path is either below the package source
directory or the package build directory.

## path_class()

!!! note
    This method was added in version 1.5.0

```python
info.path_class(path)
```
Returns a value from the `PathClass` class, which can be one of
`SOURCE`, `BINARY`, `DISCOVERED`, or `OTHER`.

## is_valid_path()

!!! note
    This method was added in version 1.5.0

```python
info.is_valid_path(path, valid=[PathClass.SOURCE, PathClass.BINARY, PathClass.DISCOVERED])
```
Returns `True` if the path belongs to any of the valid path classes. By default,
any path is accepted which is either in the source tree, the a built file, or
a path discovered by the appropriate CMake functions such as `find_file()`.

## is_existing_path()

!!! note
    This method was added in version 1.5.0

```python
info.is_existing_path(path, check=os.path.exists, require_source_folder=False, discovered_path_ok=True)
```
Returns `True` if the path is a valid path argument for a catkin command, which
means it's either an existing file or a file that will be generated by `configure_file`
or `add_custom_command`. If `discovered_path_ok` is `True`, then a path 
discovered by `find_package()`, `find_path()`, `find_file()` or `find_library()`
will be accepted as well. If `require_source_folder` is `True`, then any files
which are not physically located within the source folder will be rejected.

## report_path()

```python
info.report_path(path)
```
Returns `path` in a form that is suitable for user consumption, i.e., as argument of
`report()`. In particular, this will ensure that mock paths are rewritten in a form
that is informative for the user.

## is_catkin_install_destination()

```python
info.is_catkin_install_destination(path, subdir=None)
```
Returns `True` if the path points to the install space of
the catkin workspace. If `subdir` is not `None`, it checks
if the path points to the specified subdirectory in the install
space.


# CatkinEnvironment object

## get_package_type()

!!! note
    This method was added in version 1.6.10

```python
env.get_package_type(name)
```
Returns one of the following values:

* `catkin_lint.environment.PackageType.CATKIN` if `name` is a catkin package
* `catkin_lint.environment.PackageType.OTHER` if `name` is a package but not catkin-based
* `catkin_lint.environment.PackageType.UNKNOWN` if `name` is not a known package
* `catkin_lint.environment.PackageType.INDETERMINATE` if the package type could not be determined

## is_catkin_pkg()
```python
env.is_catkin_pkg(name)
```
Returns `True` if `name` is a catkin package. May return
wrong results if `env.ok` is not `True`.

!!! warning
    This function is deprecated since version 1.6.10.
    Please use `get_package_type()` instead.

## is_known_pkg()

```python
env.is_known_pkg(name)
```
Returns `True` if `name` is a known package. May return
wrong results if `env.ok` is not `True`.

!!! warning
    This function is deprecated since version 1.6.10.
    Please use `get_package_type()` instead.

## ok

```python
    env.ok
```
Is `True` if the list of ROS dependencies was properly
initialized.

!!! warning
    This property is deprecated since version 1.6.10.
    Please use `get_package_type()` instead.
