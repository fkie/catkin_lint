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

## '<i>pkg</i>' in find_package(catkin) is not a catkin package

- **ID**: no_catkin_component
- **Severity**: error
- **Explanation**: The <code>find_package(catkin)</code> call can list other catkin packages as dependencies with the COMPONENTS keyword. This is shorter than multiple <code>find_package()</code> calls, but does not work for system dependencies.

## <i>cmd</i>() is called before find_package(catkin)

- **ID**: catkin_order_violation
- **Severity**: error
- **Explanation**: Catkin macros cannot be called before catkin has been configured with <code>find_package(catkin)</code>.

## <i>cmd</i>() is not allowed in meta packages

- **ID**: invalid_meta_command
- **Severity**: error
- **Explanation**: Meta packages do not contain code or data and are merely dependency lists with very strict requirements for the format of the CMakeLists.txt file.

## <i>cmd</i>() needs missing directory '<i>directory</i>'

- **ID**: missing_directory
- **Severity**: error
- **Explanation**: This catkin command processes a particular directory which is missing from the package source folder.

## <i>cmd</i>() needs missing file '<i>file</i>'

- **ID**: missing_file
- **Severity**: error
- **Explanation**: This catkin command processes a particular file which is missing from the package source folder.

## <i>cmd</i>() should be all lower-case

- **ID**: cmd_case
- **Severity**: notice
- **Explanation**: The catkin manual recommends that all commands be written in lower case.

## <i>cmd</i>() used without if(CATKIN_ENABLE_TESTING)

- **ID**: unguarded_test_cmd
- **Severity**: error
- **Explanation**: You have used a test command without properly guarding it by a <code>if(CATKIN_ENABLE_TESTING)</code> block.

## <i>cmd</i>() uses directory '<i>directory</i>' which is not in package

- **ID**: external_directory
- **Severity**: warning
- **Explanation**: This catkin command uses a directory which lies outside of the package source folder. While this may work in your particular setup, you cannot assume file locations in general. Use <code>find_path()</code> to detect external locations insteed.

## <i>cmd</i>() uses file '<i>file</i>' which is not in package

- **ID**: external_file
- **Severity**: warning, error
- **Explanation**: This catkin command uses a file which lies outside of the package source folder. While this may work in your particular setup, you cannot assume file locations in general. Use <code>find_file()</code> to detect external locations insteed.

## <i>export</i> plugin file '<i>file</i>' is not installed to ${CATKIN_PACKAGE_SHARE_DESTINATION}

- **ID**: uninstalled_plugin
- **Severity**: warning, error
- **Explanation**: Your package can be used from the devel space but cannot be installed properly, because a plugin declaration file which is listed in your package.xml is not installed to the correct location.

## <i>export</i> plugin file reference must start with '${prefix}/'

- **ID**: plugin_export_prefix
- **Severity**: error
- **Explanation**: The ${prefix} variable is carefully overloaded to work with both devel space and install space and must be used in all <export plugin='...'> tags.

## <i>export</i> plugin refers to missing file '<i>file</i>'

- **ID**: missing_plugin
- **Severity**: error
- **Explanation**: A plugin declaration file which is listed in your package.xml is missing from the package source folder.

## <i>first_cmd</i>() is called before <i>second_cmd</i>()

- **ID**: order_violation
- **Severity**: error
- **Explanation**: Certain configuration macros must be called in a specific order as specified by the catkin build manual. Failure to do so may lead to improper configuration of the package and build problems.

## <i>old_cmd</i>() is deprecated, use <i>new_cmd</i>() instead

- **ID**: deprecated_cmd
- **Severity**: error
- **Explanation**: Some macros have been deprecated and replaced by newer versions. Please upgrade your CMakeLists.txt to ensure compatibility with future caktin versions.

## <i>type</i>_depend '<i>pkg</i>' is not listed in catkin_package()

- **ID**: missing_catkin_depend
- **Severity**: error
- **Explanation**: You have a catkin runtime dependency which is not exported in the CATKIN_DEPENDS stanza of the <code>catkin_package()</code>.

## <i>wrong_type</i>_depend '<i>pkg</i>' should be a <i>right_type</i>_depend

- **ID**: wrong_depend
- **Severity**: error
- **Explanation**: You have listed a package as the wrong dependency type. build_depends are needed to build your package (as in compile the declared executables and libraries). run_depends are needed at runtime to run the nodes or use the libraries and exported headers in other projects. buildtool_depends are significant only for cross-compiling; in that case, buildtool_depends are host architecture (and run during the build process) while build_depends are target architecture (and are linked against). test_depends are additional run_depends which only apply to unit tests.

## CMake module '<i>old_module</i>' is deprecated, use '<i>new_module</i>' instead

- **ID**: deprecated_cmake_module
- **Severity**: warning
- **Explanation**: Some CMake modules have been provided by cmake_modules in the past, but are now provided by the system package or CMake itself. Please upgrade your CMakeLists.txt to ensure compatibility with future catkin versions.

## OS error: <i>msg</i>

- **ID**: os_error
- **Severity**: error
- **Explanation**: An operating system error has occured. This is not a linting problem per se but might be caused by a missing or unreadable file.

## call to find_package(<i>pkg</i>) shadows previously selected components

- **ID**: shadowed_find
- **Severity**: error
- **Explanation**: You have more than one <code>find_package()</code> call for a package, and the COMPONENTS list of the later call does not include a previously chosen component.

## catkin_metapackage() in regular package

- **ID**: wrong_catkin_metapackage
- **Severity**: error
- **Explanation**: The <code>catkin_metapackage()</code> command signals your intent to declare a meta package, but the package.xml does not contain a <meta> tag.

## catkin_package() exports non-package include path

- **ID**: external_include_path
- **Severity**: error
- **Explanation**: You listed one or more include paths in the INCLUDE_DIRS stanza of your <code>catkin_package()</code> call which are not part of your package. If you want to export include paths of other modules, use <code>find_package()</code>, <code>find_path()</code>, and/or <code>find_library()</code> and add the dependency to the DEPENDS stanza.

## catkin_package() exports package include path that is not installed

- **ID**: uninstalled_include_path
- **Severity**: warning, error
- **Explanation**: Your package can be used from the devel space but cannot be installed properly, because the header files will not be copied to the proper location.

## catkin_package() exports pkg-config module '<i>pkg</i>'

- **ID**: exported_pkg_config
- **Severity**: error
- **Explanation**: Although CMake can invoke pkg-config to detect other modules, this does not work well with catkin, as pkg-config may require you to add link directories. Use the results of <code>pkg_check_module()</code> as hint for <code>find_path()</code> and <code>find_library()</code> instead.

## catkin_package() in meta package

- **ID**: wrong_catkin_package
- **Severity**: error
- **Explanation**: Meta packages use the <code>catkin_metapackage()</code> command to declare a meta package. This performs additional checks and ensures that all requirements are met.

## catkin_package() lists '<i>pkg</i>' as catkin package but it is not

- **ID**: system_as_catkin_depend
- **Severity**: error
- **Explanation**: In your <code>catkin_package()</code> call, you have listed a system dependency in the CATKIN_DEPENDS stanza, but it belongs in the DEPENDS stanza instead.

## catkin_package() lists '<i>pkg</i>' as system package but it is not

- **ID**: catkin_as_system_depend
- **Severity**: error
- **Explanation**: In your <code>catkin_package()</code> call, you have listed a catkin package in the DEPENDS stanza, but it belongs in the CATKIN_DEPENDS stanza instead.

## catkin_package() lists unconfigured system package '<i>pkg</i>'

- **ID**: unconfigured_system_depend
- **Severity**: error
- **Explanation**: In order to export a system package as dependency, you must either call <code>find_package(<i>pkg</i>)</code> first or initialize the <i>pkg</i>_INCLUDE_DIRS and <i>pkg</i>_LIBRARIES variables manually.

## condition '<i>cond</i>' is ambiguous

- **ID**: ambiguous_condition
- **Severity**: warning
- **Explanation**: Historically, the <code>if()</code> command will interpret a single token as a variable name and transparently resolve it if possible. Explicit variable references like <code>if(${var})</code> can lead to incorrect results if ${var} resolves to a different variable name. Use <code>if(var)</code> instead.

## duplicate <i>cmd</i>()

- **ID**: duplicate_cmd
- **Severity**: error
- **Explanation**: You have called this command more than once, but this does not make sense. If the calls use different arguments, consolidate them into a single call, otherwise simply remove the second.

## duplicate find_package(<i>pkg</i>)

- **ID**: duplicate_find
- **Severity**: warning
- **Explanation**: You called <code>find_package()</code> more than once for a particular package, which is not needed except for very specific, advanced circumstances.

## duplicate include path ${<i>pkg</i>_INCLUDE_DIRS}

- **ID**: duplicate_include_path
- **Severity**: warning
- **Explanation**: Include paths of packages listed in the <code>find_package(catkin)</code> command are added implicitly by the ${catkin_INCLUDE_DIRS} variable. There is no need to add it a second time.

## environment variables should not be used

- **ID**: env_var
- **Severity**: warning
- **Explanation**: The behavior of your build should not depend on any environment variables.

## executable file is not installed to bin destination

- **ID**: wrong_bin_install_destination
- **Severity**: warning
- **Explanation**: Your package installs one or more files to an unexpected location. Executable files should end up in either ${CATKIN_GLOBAL_BIN_DESTINATION} or ${CATKIN_PACKAGE_BIN_DESTINATION}.

## exported include path '<i>path</i>' does not exist

- **ID**: missing_include_path
- **Severity**: error
- **Explanation**: You have listed an invalid include path in the INCLUDE_DIRS stanza of the <code>catkin_package()</code> command.

## exported library '<i>target</i>' cannot have different output name

- **ID**: export_lib_renamed
- **Severity**: error
- **Explanation**: Due to a limitation of the catkin build system, the <code>catkin_package()</code> library export function will break if the logical target name is not equal to the actual library name.

## exported library '<i>target</i>' is not a library

- **ID**: export_lib_not_lib
- **Severity**: error
- **Explanation**: You listed a library in the LIBRARIES stanza of your <code>catkin_package()</code> call, but it really is an executable.

## exported library '<i>target</i>' is not installed

- **ID**: uninstalled_export_lib
- **Severity**: warning, error
- **Explanation**: Your package can be used from the devel space but cannot be installed properly, because a library that is exported via <code>catkin_package()</code> will not be copied to the proper location.

## exported package include path but no exported library

- **ID**: missing_export_lib
- **Severity**: warning
- **Explanation**: Your package exports a package include path and builds at least one library, which suggests that you may want to export the library to other packages as well.

## extra arguments in <i>cmd</i>()

- **ID**: endblock_args
- **Severity**: notice
- **Explanation**: The catkin manual recommends that <i>cmd</i> and other end-of-block statements have no arguments. If you have nested blocks, you should indent them properly instead.

## file '<i>script</i>' is executable but not installed

- **ID**: uninstalled_script
- **Severity**: warning
- **Explanation**: Your package contains a file that is marked as executable but not installed. If it is a script intended to be run (e.g. with rosrun), it will not work outside the devel tree. If it is not an executable script, you should fix the file permissions.

## file setup.py found but no catkin_python_setup() call

- **ID**: missing_python_setup
- **Severity**: error
- **Explanation**: The <code>catkin_python_setup()</code> call is required to properly configure python modules, and the existing setup.py indicates that your package provides one or more python modules.

## find_package(<i>pkg</i>) before find_package(cmake_modules)

- **ID**: missing_cmake_modules
- **Severity**: warning
- **Explanation**: You need to <code>find_package()</code> cmake_modules before you can use one of its custom configuration modules.

## find_package(<i>pkg</i>) has no REQUIRED option

- **ID**: missing_required
- **Severity**: warning, error
- **Explanation**: The package cannot build without this dependency, so it should be marked as REQUIRED accordingly. Use <code>if(<i>pkg</i>_FOUND)</code> clauses to use optional packages.

## generate_messages() called but no message declared

- **ID**: unused_generate_msg
- **Severity**: warning
- **Explanation**: The <code>generate_messages()</code> call creates the messages, services, and actions which are declared in your package. If your package does not supply any of these, you do not have to call <code>generate_messages()</code> at all.

## global variable '<i>var</i>' should contain project name

- **ID**: global_var_collision
- **Severity**: notice
- **Explanation**: Global variables and options are stored in the cache. You should prefix your variable names with the project name to avoid name collisions with other packages.

## include path '<i>path</i>' is exported but not used for the build

- **ID**: unused_include_path
- **Severity**: warning
- **Explanation**: You have listed an include path in the INCLUDE_DIRS stanza of the <code>catkin_package()</code> command, but that path is not mentioned in any <code>include_directories()</code> call.

## include paths '<i>path</i>' and '<i>parent_path</i>' are ambiguous

- **ID**: ambiguous_include_path
- **Severity**: warning
- **Explanation**: You have used two include paths where one is a parent of the other. Thus the same headers can be included with two different include paths which may confuse users. It is recommended that you keep your include paths consistent.

## install(<i>type</i> ... <i>dest</i>) does not install to ${CATKIN_INSTALL_PREFIX}

- **ID**: wrong_install_destination
- **Severity**: warning
- **Explanation**: Your package installs one or more files to an unexpected location. Catkin provides a number of standard variables ${CATKIN_*_DESTINATION} to specify installation folders. You should use those to ensure that your package will continue to work if the file system layout is changed in the future.

## launch configuration needs <i>type</i>_depend on '<i>pkg</i>'

- **ID**: launch_depend
- **Severity**: warning
- **Explanation**: Your package refers to another package in one of its launch files, but you do not have this dependency in your package.xml

## library output name '<i>output</i>' has redundant 'lib' prefix

- **ID**: redundant_lib_prefix
- **Severity**: notice
- **Explanation**: Libraries are automatically prefixed with 'lib', so your library will end up with a file name like 'lib<i>output</i>.so'. You can use <code>set_target_properties(... PROPERTIES OUTPUT_NAME ...)</code> to give your library a different file name without changing the target name.

## line is not indented properly

- **ID**: indentation
- **Severity**: notice
- **Explanation**: For better readability, each command should be placed on its own line. <code>if()</code> and <code>foreach()</code> bodies should be indented by one or more extra spaces.

## link_directories() must not be used for system depends

- **ID**: external_link_directory
- **Severity**: error
- **Explanation**: Directories which are added to the search path with <code>link_directories()</code> will not be propagated to dependent packages. Use <code>find_package()</code> or <code>find_library()</code> with the appropriate PATHS or HINTS instead.

## list <i>name</i> should be sorted

- **ID**: unsorted_list
- **Severity**: notice
- **Explanation**: The catkin manual recommends that list element be kept in order.

## malformed argument list: <i>msg</i>

- **ID**: argument_error
- **Severity**: warning
- **Explanation**: You invoked a CMake command with a malformed argument list. Most likely, you forgot to properly quote variables which may be empty or undefined.

## meaningless package description '<i>text</i>'

- **ID**: description_meaningless
- **Severity**: notice
- **Explanation**: Your package description merely consists of typical filler words which do not actually describe the contents of your package in a meaningful way.

## meta packages must not have <i>type</i>_depends

- **ID**: invalid_meta_depend
- **Severity**: error
- **Explanation**: Meta packages do not contain code or data and are merely dependency lists. As meta packages do neither build nor test anything, the only valid dependency type is the run_depend.

## misconfigured catkin package '<i>pkg</i>'

- **ID**: misconfigured_catkin_package
- **Severity**: error
- **Explanation**: You use an unsupported way to include a catkin package in your build. Even though this might work in your particular case, you should use the proper <code>find_package()</code> mechanism to make sure that all relevant CMake macros will be run.

## missing <i>cmd</i>()

- **ID**: missing_cmd
- **Severity**: error
- **Explanation**: You failed to call a command that is required for your package to work. Please refer to the catkin build manual for details.

## missing <i>type</i>_depend on '<i>pkg</i>'

- **ID**: missing_depend
- **Severity**: error
- **Explanation**: Your package uses features of another package but you failed to list this dependency in your package.xml

## missing COMPONENTS keyword before '<i>pkg</i>'

- **ID**: missing_components
- **Severity**: notice
- **Explanation**: The <code>find_package(catkin)</code> call can add other catkin packages as dependencies with the COMPONENTS keyword. The <code>find_package()</code> command lists additional packages but has no COMPONENTS keyword.

## missing find_package(<i>pkg</i>)

- **ID**: missing_find
- **Severity**: error
- **Explanation**: You failed to call <code>find_package()</code> for a dependency of your package.

## missing generate_messages()

- **ID**: missing_generate_msg
- **Severity**: error
- **Explanation**: The <code>generate_messages()</code> call creates the messages, services, and actions which are declared in your package by <code>add_message_files()</code>, <code>add_service_files()</code>, and <code>add_action_files()</code> respectively.

## missing include_directories(${catkin_INCLUDE_DIRS})

- **ID**: unused_catkin_include_dirs
- **Severity**: error
- **Explanation**: You must add the catkin include paths to your include search list, or you might experience build failures.

## operands for operator <i>op</i> should be quoted strings

- **ID**: unquoted_string_op
- **Severity**: notice
- **Explanation**: The catkin manual recommends that <code>if()</code> conditions with string operators should have the operands enclosed in double quotes.

## package '<i>pkg</i>' should be listed in catkin_package()

- **ID**: suggest_catkin_depend
- **Severity**: warning
- **Explanation**: Your package configures another package as build dependency, it is listed as run_depend in your package.xml, and its name suggests that it contains ROS messages. In that case, you must add it to the CATKIN_DEPENDS stanza of your <code>catkin_package()</code>

## package description starts with boilerplate '<i>text</i>'

- **ID**: description_boilerplate
- **Severity**: notice
- **Explanation**: Your package description starts with a number of typical filler words which do not actually describe the contents of your package. Typically, you can simply delete these words from the description, and it will still make sense and be much more concise.

## package exports <i>export</i> plugin but does not <i>type</i>_depend on '<i>pkg</i>'

- **ID**: plugin_depend
- **Severity**: warning
- **Explanation**: Your package exports a plugin for another package, but fails to list said package as a dependency.

## package path name '<i>path</i>' differs from package name

- **ID**: package_path_name
- **Severity**: notice
- **Explanation**: Your package resides in a folder that has a different name than the package itself. This is confusing and might break the assumptions of some tools.

## parse error: <i>msg</i>

- **ID**: parse_error
- **Severity**: warning
- **Explanation**: Your package has a malformed file that could not be processed for linting.

## project name '<i>name</i>' differs from package name

- **ID**: project_name
- **Severity**: error
- **Explanation**: The CMake project name must be identical to the package name. For backwards compatibility reasons, both names should also be identical to the name of the source folder that contains the package.

## referenced target '<i>target</i>' is not defined

- **ID**: undefined_target
- **Severity**: error
- **Explanation**: Your package installs or exports a CMake target which is not defined at all. This could be a typo, or the target is implicitly defined by a macro that is unknown to **catkin_lint**.

## script '<i>file</i>' has no <i>interpreter</i> shebang line

- **ID**: missing_shebang
- **Severity**: error
- **Explanation**: All <i>interpreter</i> scripts need an appropriate shebang line, i.e. the first line has to start with '#!' and needs to name the full path to the <i>interpreter</i> executable.

## script '<i>script</i>' must be executable

- **ID**: script_not_executable
- **Severity**: error
- **Explanation**: Your package contains a script file that has to be marked as executable. On Un*x systems, run 'chmod +x "<i>script</i>"' to set the executable bit.

## subdirectory '<i>subdir</i>' contains a subproject

- **ID**: subproject
- **Severity**: warning
- **Explanation**: Your package has an independent subproject. This can interact with catkin in unusual ways and is strongly discouraged. No further checks are performed in this subdirectory.

## subdirectory '<i>subdir</i>' is added a second time

- **ID**: duplicate_subdir
- **Severity**: error
- **Explanation**: You added another subdirectory with <code>add_subdirectory()</code> multiple times. This can also happen if you accidentally created a loop where subdir A adds subdir B, which adds subdir A again.

## subdirectory '<i>subdir</i>' is missing

- **ID**: missing_subdir
- **Severity**: error
- **Explanation**: You specified a subdirectory which does not exists or is unreadable.

## subdirectory <i>subdir</i> is not in package

- **ID**: external_subdir
- **Severity**: error
- **Explanation**: You added another subdirectory with <code>add_subdirectory()</code>, but the specified path points outside of the package source directory.

## target '<i>export_target</i>' depends on target '<i>target</i>' which is not installed

- **ID**: uninstalled_depend
- **Severity**: error
- **Explanation**: Your package can be used from the devel space but cannot be installed properly, because one of your installed targets depends on a library from your package that is not installed as well.

## target '<i>target</i>' has invalid characters in its output file name

- **ID**: invalid_target_output
- **Severity**: error
- **Explanation**: The output file that your target is supposed to generate contains invalid characters in its name. You probably forget to call <code>set_target_properties(... PROPERTIES OUTPUT_NAME ...)</code>

## target '<i>target</i>' is not installed

- **ID**: uninstalled_target
- **Severity**: warning
- **Explanation**: Your package can be used from the devel space but cannot be installed properly, because the build target will not be copied to the proper location.

## test_depend '<i>pkg</i>' used without if(CATKIN_ENABLE_TESTING)

- **ID**: unguarded_test_depend
- **Severity**: error
- **Explanation**: You have used a test dependency without properly guarding it by a <code>if(CATKIN_ENABLE_TESTING)</code> block. You must add a proper build dependency if you wish to use this package even if tests are disabled.

## unconfigured build_depend on '<i>pkg</i>'

- **ID**: unconfigured_build_depend
- **Severity**: error
- **Explanation**: You declare a build dependency on another package but neither call <code>find_package()</code> nor have it listed as catkin component in the <code>find_package(catkin)</code> call.

## unconfigured message dependency '<i>pkg</i>'

- **ID**: unconfigured_msg_depend
- **Severity**: error
- **Explanation**: Your messages depend on another package which is neither <code>find_package()</code>'d nor listed as a component in the <code>find_package(catkin)</code> call.

## unknown package '<i>pkg</i>'

- **ID**: unknown_package
- **Severity**: error
- **Explanation**: You are referring to a package which seems to be neither a catkin package nor a known system dependency. You may have misspelled the name, or your rosdep database needs to be refreshed with "rosdep update".

## unused <i>type</i>_depend on '<i>pkg</i>'

- **ID**: unused_depend
- **Severity**: error
- **Explanation**: You have a listed a package dependency but do not appear to use any of the features it provides.

## use ${PROJECT_NAME} instead of '<i>name</i>'

- **ID**: literal_project_name
- **Severity**: notice
- **Explanation**: The catkin manual recommends that you use the ${PROJECT_NAME} variable instead of the literal project name.

## use find_package(<i>pkg</i>) instead of include(Find<i>pkg</i>.cmake)

- **ID**: find_by_include
- **Severity**: error
- **Explanation**: The FindXXX.cmake modules are intended to be included by the <code>find_package()</code> command.

## use of link_directories() is strongly discouraged

- **ID**: link_directory
- **Severity**: warning
- **Explanation**: Directories which are added to the search path with <code>link_directories()</code> will not be propagated to dependent packages. You should avoid this command or at least be aware that it might not work as expected in dependent packages.

## variable <i>var</i> is modified

- **ID**: critical_var_append
- **Severity**: warning
- **Explanation**: You have appended extra data to a critical CMake variable. This might break the build on different systems or affect the global catkin workspace in unintended ways.

## variable <i>var</i> is modified

- **ID**: immutable_var
- **Severity**: error
- **Explanation**: You have modified a CMake variable that is initialized by CMake itself and must not be modified under any circumstances.

## variable <i>var</i> is overwritten

- **ID**: critical_var_overwrite
- **Severity**: error
- **Explanation**: You have overwritten a critical CMake variable and its original content is lost. This will most likely break the build on different systems or affect the global catkin workspace in unintended ways.

## variable CMAKE_BUILD_TYPE is overwritten unconditionally

- **ID**: cmake_build_type
- **Severity**: error
- **Explanation**: If you wish to provide a default value for CMAKE_BUILD_TYPE, make sure that you do not overwrite user preferences. You should guard the <code>set()</code> command with an appropriate <code>if(NOT CMAKE_BUILD_TYPE)</code> block.

