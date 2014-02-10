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

message_list = {

"DUPLICATE_CMD": ( "duplicate %(cmd)s()",
"""\
You have called this command more than once, but this does not
make sense. If the calls use different arguments, consolidate them
into a single call, otherwise simply remove the second.
"""
),
"MISSING_CMD": ( "missing %(cmd)s()",
"""\
You failed to call a command that is required for your package
to work. Please refer to the catkin build manual for details.
"""
),
"DUPLICATE_FIND" : ( "duplicate find_package(%(pkg)s)",
"""\
The find_package() searches for a dependency and caches the result.
A second call will be silently ignored. In particular, if you
specify different arguments to the second call, those will not have
any effect at all, which is most likely not what you want.
"""
),
"MISSING_FIND" : ( "missing find_package(%(pkg)s)",
"""\
You failed to call find_package() for a dependency of your package.
"""
),
"PROJECT_NAME" : ( "project name '%(name)s' differs from package name",
"""\
The CMake project name must be identical to the package name. For
backwards compatibility reasons, the both names should also be identical
to the name of the source folder that contains the package.
"""
),
"ORDER_VIOLATION" : ( "%(first_cmd)s() is called before %(second_cmd)s()",
"""\
Certain configuration macros must be called in a specific order as
specified by the catkin build manual. Failure to do so may lead to
improper configuration of the package and build problems.
"""
),
"CATKIN_ORDER_VIOLATION" : ( "%(cmd)s() is called before find_package(catkin)",
"""\
Catkin macros cannot be called before catkin has been configured with
find_package(catkin).
"""
),
"DEPRECATED_ROSBUILD" : ( "%(cmd)s() is deprecated",
"""\
This construct was intended to facilitate the migration from
Rosbuild to Catkin. It is deprecated and should not be used any more.
"""
),
"MISSING_REQUIRED" : ( "find_package(%(pkg)s) has no REQUIRED option",
"""\
The package cannot build without this dependency, so it should be
marked as REQUIRED accordingly.
"""
),
"MISSING_COMPONENTS" : ("missing COMPONENTS keyword before '%(pkg)s'",
"""\
The find_package(catkin) call can add other catkin packages as
dependencies with the COMPONENTS keyword. The find_package() command
lists additional packages but has no COMPONENTS keyword.
"""
),
"NO_CATKIN_COMPONENT" : ("'%(pkg)s' in find_package(catkin) is not a catkin package",
"""\
The find_package(catkin) call can list other catkin packages as
dependencies with the COMPONENTS keyword. This is shorter than
multiple find_package() calls, but does not work for system dependencies.
"""
),
"MISSING_CATKIN_DEPEND" : ("run_depend '%(pkg)s' is not listed in catkin_package()",
"""\
You have specified a catkin run dependency but failed to list
it in the CATKIN_DEPENDS stanza of the catkin_package() call.
"""
),
"INVALID_META_COMMAND" : ("%(cmd)s() is not allowed in meta packages",
"""\
Meta packages do not contain code or data and are merely dependency lists
with very strict requirements for the format of the CMakeLists.txt file.
"""
),
"INVALID_META_DEPEND" : ("meta packages must not have %(type)s_depends",
"""\
Meta packages do not contain code or data and are merely dependency lists.
As meta packages do neither build nor test anything, the only valid
dependency type is the run_depend.
"""
),
"CATKIN_PKG_VS_META" : ("catkin_package() in meta package",
"""\
Meta packages use the catkin_metapackage() command to declare a
meta package. This performs additional checks and ensures that all
requirements are met.
"""
),
"CATKIN_META_VS_PKG" : ("catkin_metapackage() in regular package",
"""\
The catkin_metapackage() command signals your intent to declare
a meta package, but the package.xml does not contain a <meta> tag.
"""
),
"IMMUTABLE_VAR" :  ("variable %(var)s is modified",
"""\
You have modified a CMake variable that is initialized by CMake
itself and must not be modified under any circumstances.
"""
),
"CRITICAL_VAR_OVERWRITE" :  ("variable %(var)s is overwritten",
"""\
You have overwritten a critical CMake variable and its original
content is lost. This will most likely break the build on
different systems or affect the global catkin workspace in
unintended ways.
"""
),
"CRITICAL_VAR_APPEND" :  ("variable %(var)s is modified",
"""\
You have appended extra data to a critical CMake variable.
This might break the build on different systems or affect
the global catkin workspace in unintended ways.
"""
),
"MISSING_FILE" : ("%(cmd)s() needs missing file '%(file)s'",
"""\
This catkin command processes a particular file which is missing
from the package source folder.
"""
),
"INSTALL_DESTINATION" : ("install(%(type)s ... %(dest)s) is not one of the ${CATKIN_*_DESTINATION}s",
"""\
Catkin provides a number of standard variables to specify
installation folders. You should use those to ensure that your
package will continue to work if the file system layout is
changed in the future.
"""
),
"UNUSED_DEPEND" : ("unused %(type)s_depend on '%(pkg)s'",
"""\
You have a listed a package dependency but do not appear
to use any of the features it provides.
"""
),
"MISSING_DEPEND" : ("missing %(type)s_depend on '%(pkg)s'",
"""\
Your package uses features of another package but you
failed to list this dependency in your package.xml
"""
),
"UNCONFIGURED_BUILD_DEPEND" : ("unconfigured build_depend on '%(pkg)s'",
"""\
You declare a build dependency on another package but neither
call find_package() nor have it listed as catkin component in
the find_package(catkin) call.
"""
),
"WRONG_DEPEND" : ("%(wrong_type)s_depend '%(pkg)s' should be a %(right_type)s_depend",
"""\
You have listed a package as the wrong dependency type. build_depends are needed
to build your package (as in compile the declared executables and libraries).
run_depends are needed at runtime to run the nodes or use the libraries
and exported headers in other projects. buildtool_depends are significant only
for cross-compiling; in that case, buildtool_depends are host architecture (and run
during the build process) while build_depends are target architecture (and are
linked against). test_depends are additional run_depends which only apply to unit tests.
"""
),
"UNUSED_GENERATE_MSG" : ("generate_messages() called but no message declared",
"""\
The generate_messages() call creates the messages, services, and actions
which are declared in your package. If your package does not supply any
of these, you do not have to call generate_messages() at all.
"""
),
"MISSING_GENERATE_MSG" : ("missing generate_messages()",
"""\
The generate_messages() call creates the messages, services, and actions
which are declared in your package by add_message_files(), add_service_files(),
and add_action_files() respectively.
"""
),
"SYSTEM_AS_CATKIN_DEPEND" : ("catkin_package() lists '%(pkg)s' as catkin package but it is not",
"""\
In your catkin_package() call, you have listed a system dependency in the
CATKIN_DEPENDS stanza, but it belongs in the DEPENDS stanza instead.
"""
),
"CATKIN_AS_SYSTEM_DEPEND" : ("catkin_package() lists '%(pkg)s' as system package but it is not",
"""\
In your catkin_package() call, you have listed a catkin package in the
DEPENDS stanza, but it belongs in the CATKIN_DEPENDS stanza instead.
"""
),
"UNCONFIGURED_SYSTEM_DEPEND" : ("catkin_package() lists unconfigured system package '%(pkg)s'",
"""\
In order to export a system package as dependency, you must either
call find_package(%(pkg)s) first or initialize the %(pkg)s_INCLUDE_DIRS and
%(pkg)s_LIBRARIES variables manually.
"""
),
"MISSING_BUILD_INCLUDE" : ("include path '%(path)s' is exported but not used for the build",
"""\
You have listed an include path in the INCLUDE_DIRS stanza of the
catkin_package() command, but that path is not mentioned in any
include_directories() call.
"""
),
"MISSING_EXPORT_INCLUDE_PATH" : ("exported include path '%(path)s' does not exist",
"""\
You have listed an invalid include path in the INCLUDE_DIRS stanza of the
catkin_package() command.
"""
),
"MISSING_BUILD_INCLUDE_PATH" : ("build include path '%(path)s' does not exist",
"""\
You have listed an invalid include path in the include_directories() command.
"""
),
"EXTERNAL_INCLUDE_PATH" : ("catkin_package() exports non-package include path",
"""\
You listed one or more include paths in the INCLUDE_DIRS stanza of
your catkin_package() call which are not part of your package. If you
want to export include paths of other modules, use find_package(),
find_path(), and/or find_library() and add the dependency to the
DEPENDS stanza.
"""
),
"MISSING_CATKIN_INCLUDE" : ("missing include_directories(${catkin_INCLUDE_DIRS})",
"""\
You must add the catkin include paths to your include search list, or
you might experience build failures.
"""
),
"MISSING_INSTALL_INCLUDE" : ("catkin_package() exports package include path that is not installed",
"""\
Your package can be used from the devel space but cannot be installed
properly, because the header files will not be copied to the proper location.
"""
),
"MISSING_INSTALL_TARGET" : ("target '%(target)s' is not installed",
"""\
Your package can be used from the devel space but cannot be installed
properly, because the build target will not be copied to the proper location.
"""
),
"MISSING_PYTHON_SETUP" : ("file setup.py found but no catkin_python_setup() call",
"""\
The catkin_python_setup() call is required to properly configure python
modules, and the existing setup.py indicates that your package provides one or
more python modules.
"""
),
"MISSING_EXPORT_LIB" : ("exported package include path but no exported library",
"""\
Your package exports a package include path and builds at least one
library, which suggests that you may want to export the library to
other packages as well.
"""
),
"UNCONFIGURED_MSG_DEPEND" : ("unconfigured message dependency '%(pkg)s'",
"""\
Your messages depend on another package which is neither find_package()'d
nor listed as a component in the find_package(catkin) call.
"""
),
"MISSING_MSG_DEPEND" : ("message dependency '%(pkg)s' is not listed as %(type)s_depend",
"""\
Your messages depend on another package which is not listed as %(type)s_depend in
your package.xml
"""
),
"MISSING_MSG_CATKIN" : ("message dependency '%(pkg)s' is not listed in catkin_package()",
"""\
Your messages depend on another package which is not in the CATKIN_DEPENDS
stanza of your catkin_package() call.
"""
),
"UNINSTALLED_EXPORT_LIB" : ("exported library '%(target)s' is not installed",
"""\
Your package can be used from the devel space but cannot be installed
properly, because a library that is exported via catkin_package() will
not be copied to the proper location.
"""
),
"EXPORT_LIB_NOT_LIB" : ("exported library '%(target)s' is not a library",
"""\
You listed a library in the LIBRARIES stanza of your catkin_package() call,
but it really is an executable.
"""
),
"EXPORT_LIB_RENAMED" : ("exported library '%(target)s' cannot have different output name",
"""\
Due to a limitation of the catkin build system, the catkin_package()
library export function will break if the logical target name is not
equal to the actual library name.
"""
),
"SUGGEST_CATKIN_DEPEND" : ("package '%(pkg)s' should be listed in catkin_package()",
"""\
Your package configures another package as build dependency, it is listed as
run_depend in your package.xml, and its name suggests
that it contains ROS messages. In that case, you must add it to the
CATKIN_DEPENDS stanza of your catkin_package()
"""
),
"UNDEFINED_TARGET" : ("exported target '%(target)s' is not defined",
"""\
Your package provides a CMake target to other packages, but the listed
target is not defined at all.
"""
),
"INVALID_TARGET_OUTPUT" : ("target '%(target)s' has invalid characters in its output file name",
"""\
The output file that your target is supposed to generate contains invalid
characters in its name. You probably forget to call set_target_properties(... PROPERTIES
OUTPUT_NAME ...)
"""
),
"TARGET_NAME_COLLISION" : ("target '%(target)s' should contain package name",
"""\
The CMake build system requires all target identifiers to be globally unique.
For this reason, it is highly recommended that you add the package name as in
'${PROJECT_NAME}_target' or '${PROJECT_NAME}/target'.
You can use set_target_properties(... PROPERTIES OUTPUT_NAME ...)
to give your target a different output file name (which does not have to
be unique if it is installed in a package-specific location).
"""
),
"UNINSTALLED_DEPEND" : ("target '%(export_target)s' depends on target '%(target)s' which is not installed",
"""\
Your package can be used from the devel space but cannot be installed
properly, because one of your installed targets depends on a library from
your package that is not installed as well.
"""
),
"LINK_DIRECTORY": ("use of link_directories() is strongly discouraged",
"""\
Directories which are added to the search path with link_directories()
will not be propagated to dependent packages. You should avoid this
command or at least be aware that it might not work as expected in dependent
packages.
"""
),
"EXTERNAL_LINK_DIRECTORY": ("link_directories() must not be used for system depends",
"""\
Directories which are added to the search path with link_directories()
will not be propagated to dependent packages. Use find_package()
or find_library() with the appropriate PATHS or HINTS instead.
"""
),
"PLUGIN_EXPORT_PREFIX" : ("%(export)s plugin file reference must start with '${prefix}/'",
"""\
The ${prefix} variable is carefully overloaded to work with both
devel space and install space and must be used in all <export plugin='...'> tags.
"""
),
"PLUGIN_MISSING_FILE" : ("%(export)s plugin refers to missing file '%(file)s'",
"""\
A plugin declaration file which is listed in your package.xml is missing from
the package source folder.
"""
),
"PLUGIN_MISSING_INSTALL" : ("%(export)s plugin file '%(file)s' is not installed to ${CATKIN_PACKAGE_SHARE_DESTINATION}",
"""\
Your package can be used from the devel space but cannot be installed
properly, because a plugin declaration file which is listed in your package.xml
is not installed to the correct location.
"""
),
"PLUGIN_DEPEND" : ("package exports %(export)s plugin but does not %(type)s_depend on '%(pkg)s'",
"""\
Your package exports a plugin for another package, but fails to list said
package as a dependency.
"""
),
"DESCRIPTION_BOILERPLATE" : ("package description starts with boilerplate '%(text)s'",
"""\
Your package description starts with a number of typical filler words which
do not actually describe the contents of your package. Typically, you can
simply delete these words from the description, and it will still make sense
and be much more concise.
"""
),
"DESCRIPTION_MEANINGLESS" : ("meaningless package description '%(text)s'",
"""\
Your package description merely consists of typical filler words which
do not actually describe the contents of your package in a meaningful way.
"""
),
"FIND_BY_INCLUDE": ("use find_package(%(pkg)s) instead of include(Find%(pkg)s.cmake)",
"""\
The FindXXX.cmake modules are intended to be included by the find_package()
command.
"""
),
"REDUNDANT_TEST_DEPEND" : ("redundant test_depend '%(pkg)s'",
"""\
Test dependencies are additional dependencies for testing, so there is no
need to list any build or run dependency a second time.
"""
),
"REDUNDANT_LIB_PREFIX" : ("library output name '%(output)s' has redundant 'lib' prefix",
"""\
Libraries are automatically prefixed with 'lib', so your library will end up
with a file name like 'lib%(output)s.so'. You can use
set_target_properties(... PROPERTIES OUTPUT_NAME ...) to give your library a
different file name without changing the target name.
"""
),
"UNKNOWN_DEPEND" : ("unknown %(type)s_depend '%(pkg)s'",
"""\
The specified dependency is neither a catkin package nor a known system dependency
from the rosdep database.
"""
),
"UNSUPPORTED_CMD" : ("unsupported command '%(cmd)s'",
"""\
Your package uses CMake constructs which cannot be linted properly at this time.
Certain errors may go unnoticed while other errors may be false positives.
"""
),
"EXTERNAL_SUBDIR" : ( "subdirectory %(subdir)s is not in package",
"""\
You added another subdirectory with add_subdirectory(), but the
specified path points outside of the package source directory.
"""
),
"MISSING_SUBDIR" : ( "subdirectory '%(subdir)s' is missing",
"""\
You specified a subdirectory which does not exists or is unreadable.
"""
),
"DUPLICATE_SUBDIR" : ( "subdirectory '%(subdir)s' is added a second time",
"""\
You added another subdirectory with add_subdirectory() multiple times.
This can also happen if you accidentally created a loop where subdir A
adds subdir B, which adds subdir A again.
"""
),
"SUBPROJECT" : ( "subdirectory '%(subdir)s' contains a subproject",
"""\
Your package has an independent subproject. This can interact
with catkin in unusual ways and is strongly discouraged. No
further checks are performed in this subdirectory.
"""
),
"GLOBAL_VAR_COLLISION" : ( "global variable '%(var)s' should contain project name",
"""\
Global variables and options are stored in the cache.
You should prefix your variable names with the project name to
avoid name collisions with other packages.
"""
),
"ENV_VAR" : ( "environment variables should not be used",
"""\
The behavior of your build should not depend on any
environment variables.
"""
),
"EXPORTED_PKG_CONFIG" : ( "catkin_package() exports pkg-config module '%(pkg)s'",
"""\
Although CMake can invoke pkg-config to detect other modules,
this does not work well with catkin, as pkg-config may require
you to add link directories. Use the results of pkg_check_module()
as hint for find_path() and find_library() instead.
"""),
"OS_ERROR" : ( "OS error: %(msg)s",
"""\
An operating system error has occured. This is not a linting problem per se but
might be caused by a missing or unreadable file.
"""
),
}

def msg(msg_id, **kwargs):
    text, explanation = message_list[msg_id]
    return ( msg_id, text % kwargs, explanation % kwargs )
