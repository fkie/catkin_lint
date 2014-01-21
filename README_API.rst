Lint Check API for catkin_lint
##############################

**catkin_lint** checks are implemented as
submodules, and interact with the main program
via callbacks.

Registration
============

Each check must provide a setup function
``check_name(linter)`` that defines the
callbacks for the check.
The function is called with an object
that provides the following methods:

require()
---------

::

    linter.require(check_name)

Checks may depend on the results of other checks.
The ``require`` method ensures that ``check_name``
is called exactly once. Circular dependencies of the
form ``A->B->C->A`` will be detected and cause an
exception. In particular, tests must not require themselves.


add_init_hook()
---------------

::

    def callback(info)

    linter.add_init_hook(callback)

Registers an initialization hook that is called when
the lint check for a particular package begins.


add_command_hook()
------------------

::

    def callback(info, cmd, args)

    linter.add_command_hook(name, callback)

Registers a command hook that is called each time the
CMake parser encounters the command ``name``. The command
name and a list of its arguments are passed to the callback.
All command names are converted to lower-case.


add_final_hook()
----------------

::

    def callback(info)

    linter.add_final_hook(callback)

Registers a final hook that is called when the CMake parser
has finished parsing the ``CMakeLists.txt`` file.


Info Object
===========

For each linted package, **catkin_lint** creates a
dedicated info object that is passed to all callbacks.
The info object can be used to store relevant data.
Typically, the init hook is used to initialize check-specific
variables in the info object. As a general rule, checks must not
modify variables they do not own, and each check must ensure it
uses unique variable names that do not conflict with other checks.

The following variables are defined by **catkin_lint** itself:

``env``
    A ``CatkinEnvironment`` object that provides information about
    the ROS environment.
``path``
    The filesystem path to the package source folder
``manifest``
    A ``catkin_pkg.packages.Package`` object that is created from
    information in the ``package.xml``.
``file``
    The currently parsed CMake file, relative to the package source
    folder. Is either ``CMakeLists.txt`` or an include file. Only
    valid in command hooks.
``line``
    The line number of the currently processed CMake command. Only
    valid in command hooks.
``commands``
    A set of all command names that have been encountered up to this point.
``find_packages``
    A set of all packages that have been configured with ``find_package()``
    up to this point.
``targets``
    A set of all make targets that have been defined up to this point.
``executables``
    A set of all executables that have been defined up to this point.
``libraries``
    A set of all libraries that have been defined up to this point.
``var``
    A dictionary of all known CMake variables. Note that many variables
    have mocked values. In particular, the package source and build folder
    are ``/pkg-source`` and ``pkg-build`` respectively.


report()
--------

::

    info.report(level, msg_id, **kwargs)

Reports a problem to the user. ``level`` must be one of
``catkin_lint.linter.ERROR``, ``catkin_lint.linter.WARNING``, or
``catkin_lint.linter.NOTICE``. The ``msg_id`` refers to one
of the defined diagnostic messages. Certain messages have placeholder
variables that must be specified, e.g. ``cmd`` for the command name.


package_path()
--------------

::

    info.package_path(path)

Returns a path relative to the package source directory or
an absolute path if the path is not within the package. Can handle
``${CMAKE_CURRENT_SOURCE_DIR}`` correctly.


real_path()
-----------

::

    info.real_path(path)

Returns the actual file system path for relative package path as
returned by ``package_path()``.


is_internal_path(path)
----------------------

::

    info.is_internal_path(path)

Returns ``True`` if the path is either below the package source
directory or the package build directory.


is_catkin_target()
------------------

::

    info.is_catkin_target(path, subdir=None)

Returns ``True`` if the path points to the install space of
the catkin workspace. If ``subdir`` is not ``None``, it checks
if the path points to the specified subdirectory in the install
space.


CatkinEnvironment object
========================

is_catkin_pkg()
---------------

::

    env.is_catkin_pkg(name)

Returns ``True`` if ``name`` is a known catkin package.


is_system_pkg()
---------------

::

    env.is_system_pkg(name)

Returns ``True`` if ``name`` is a known system dependency.


is_known_pkg()
---------------

::

    env.is_known_pkg(name)

Alias for ``is_catkin_pkg(name) or is_system_pkg(name)``.

has_rosdep()
---------------

::

    env.has_rosdep()

Returns ``True`` if list of ROS dependencies was properly
initialized. If it returns ``False``, the function ``is_system_pkg()``
will never return ``True``. In this case, checks for invalid dependencies 
should be skipped to prevent false positives.

