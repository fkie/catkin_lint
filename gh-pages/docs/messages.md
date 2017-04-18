# catkin_lint diagnostic messages

This is a list of all messages which might be shown by **catkin_lint**.
Each diagnostic has a unique ID (such as *catkin_order_violation*),
which you can use to disable certain messages, either with the command line option
`--ignore ID`, or by adding a pragma line `#catkin_lint: ignore ID` at the beginning
of the CMakeLists.txt file.

## '*pkg*' in find_package(catkin) is not a catkin package

- **ID**: no_catkin_component
- **Severity**: error
- **Explanation**:         The find_package(catkin) call can list other catkin packages as         dependencies with the COMPONENTS keyword. This is shorter than         multiple find_package() calls, but does not work for system dependencies.         

## *cmd*() is called before find_package(catkin)

- **ID**: catkin_order_violation
- **Severity**: error
- **Explanation**:         Catkin macros cannot be called before catkin has been configured with         find_package(catkin).         

## *cmd*() is deprecated

- **ID**: deprecated_rosbuild
- **Severity**: warning, error
- **Explanation**:         This construct was intended to facilitate the migration from         Rosbuild to Catkin. It is deprecated and should not be used any more.         

## *cmd*() is not allowed in meta packages

- **ID**: invalid_meta_command
- **Severity**: error
- **Explanation**:         Meta packages do not contain code or data and are merely dependency lists         with very strict requirements for the format of the CMakeLists.txt file.         

## *cmd*() needs missing directory '*directory*'

- **ID**: missing_directory
- **Severity**: error
- **Explanation**:         This catkin command processes a particular directory which is missing         from the package source folder.         

## *cmd*() needs missing file '*file*'

- **ID**: missing_file
- **Severity**: error
- **Explanation**:         This catkin command processes a particular file which is missing         from the package source folder.         

## *cmd*() should be all lower-case

- **ID**: cmd_case
- **Severity**: notice
- **Explanation**:         The catkin manual recommends that all commands be written in lower case.         

## *cmd*() used without if(CATKIN_ENABLE_TESTING)

- **ID**: unguarded_test_cmd
- **Severity**: error
- **Explanation**:         You have used a test command without properly guarding it by a         if(CATKIN_ENABLE_TESTING) block.         

## *export* plugin file '*file*' is not installed to ${CATKIN_PACKAGE_SHARE_DESTINATION}

- **ID**: plugin_missing_install
- **Severity**: warning, error
- **Explanation**:         Your package can be used from the devel space but cannot be installed         properly, because a plugin declaration file which is listed in your package.xml         is not installed to the correct location.         

## *export* plugin file reference must start with '${prefix}/'

- **ID**: plugin_export_prefix
- **Severity**: error
- **Explanation**:         The ${prefix} variable is carefully overloaded to work with both         devel space and install space and must be used in all <export plugin='...'> tags.         

## *export* plugin refers to missing file '*file*'

- **ID**: plugin_missing_file
- **Severity**: error
- **Explanation**:         A plugin declaration file which is listed in your package.xml is missing from         the package source folder.         

## *first_cmd*() is called before *second_cmd*()

- **ID**: order_violation
- **Severity**: error
- **Explanation**:         Certain configuration macros must be called in a specific order as         specified by the catkin build manual. Failure to do so may lead to         improper configuration of the package and build problems.         

## *old_cmd*() is deprecated, use *new_cmd*() instead

- **ID**: deprecated_cmd
- **Severity**: error
- **Explanation**:         Some macros have been deprecated and replaced by newer versions.         Please upgrade your CMakeLists.txt to ensure compatibility with         future caktin versions.         

## *type*_depend '*pkg*' is not listed in catkin_package()

- **ID**: missing_catkin_depend
- **Severity**: error
- **Explanation**:         You have specified a catkin run dependency but failed to list         it in the CATKIN_DEPENDS stanza of the catkin_package() call.         

## *wrong_type*_depend '*pkg*' should be a *right_type*_depend

- **ID**: wrong_depend
- **Severity**: error
- **Explanation**:         You have listed a package as the wrong dependency type. build_depends are needed         to build your package (as in compile the declared executables and libraries).         run_depends are needed at runtime to run the nodes or use the libraries         and exported headers in other projects. buildtool_depends are significant only         for cross-compiling; in that case, buildtool_depends are host architecture (and run         during the build process) while build_depends are target architecture (and are         linked against). test_depends are additional run_depends which only apply to unit tests.         

## CMake module '*old_module*' is deprecated, use '*new_module*' instead

- **ID**: deprecated_cmake_module
- **Severity**: warning
- **Explanation**:         Some CMake modules have been provided by cmake_modules in the past, but         are now provided by the system package or CMake itself.         Please upgrade your CMakeLists.txt to ensure compatibility with future         catkin versions.         

## OS error: *msg*

- **ID**: os_error
- **Severity**: error
- **Explanation**:         An operating system error has occured. This is not a linting problem per se but         might be caused by a missing or unreadable file.         

## build include path '*path*' does not exist

- **ID**: missing_build_include_path
- **Severity**: error
- **Explanation**:         You have listed an invalid include path in the include_directories() command.         

## catkin_metapackage() in regular package

- **ID**: catkin_meta_vs_pkg
- **Severity**: error
- **Explanation**:         The catkin_metapackage() command signals your intent to declare         a meta package, but the package.xml does not contain a <meta> tag.         

## catkin_package() exports non-package include path

- **ID**: external_include_path
- **Severity**: error
- **Explanation**:         You listed one or more include paths in the INCLUDE_DIRS stanza of         your catkin_package() call which are not part of your package. If you         want to export include paths of other modules, use find_package(),         find_path(), and/or find_library() and add the dependency to the         DEPENDS stanza.         

## catkin_package() exports package include path that is not installed

- **ID**: missing_install_include
- **Severity**: warning, error
- **Explanation**:         Your package can be used from the devel space but cannot be installed         properly, because the header files will not be copied to the proper location.         

## catkin_package() exports pkg-config module '*pkg*'

- **ID**: exported_pkg_config
- **Severity**: error
- **Explanation**:         Although CMake can invoke pkg-config to detect other modules,         this does not work well with catkin, as pkg-config may require         you to add link directories. Use the results of pkg_check_module()         as hint for find_path() and find_library() instead.         

## catkin_package() in meta package

- **ID**: catkin_pkg_vs_meta
- **Severity**: error
- **Explanation**:         Meta packages use the catkin_metapackage() command to declare a         meta package. This performs additional checks and ensures that all         requirements are met.         

## catkin_package() lists '*pkg*' as catkin package but it is not

- **ID**: system_as_catkin_depend
- **Severity**: error
- **Explanation**:         In your catkin_package() call, you have listed a system dependency in the         CATKIN_DEPENDS stanza, but it belongs in the DEPENDS stanza instead.         

## catkin_package() lists '*pkg*' as system package but it is not

- **ID**: catkin_as_system_depend
- **Severity**: error
- **Explanation**:         In your catkin_package() call, you have listed a catkin package in the         DEPENDS stanza, but it belongs in the CATKIN_DEPENDS stanza instead.         

## catkin_package() lists unconfigured system package '*pkg*'

- **ID**: unconfigured_system_depend
- **Severity**: error
- **Explanation**:         In order to export a system package as dependency, you must either         call find_package(*pkg*) first or initialize the *pkg*_INCLUDE_DIRS and         *pkg*_LIBRARIES variables manually.         

## condition '*cond*' is ambiguous

- **ID**: ambiguous_condition
- **Severity**: warning
- **Explanation**:         Historically, the if() command will interpret a single token as a variable         name and transparently resolve it if possible. Explicit variable references         like if(${var}) can lead to incorrect results if ${var} resolves to a different         variable name. Use if(var) instead.         

## duplicate *cmd*()

- **ID**: duplicate_cmd
- **Severity**: error
- **Explanation**:         You have called this command more than once, but this does not         make sense. If the calls use different arguments, consolidate them         into a single call, otherwise simply remove the second.         

## duplicate find_package(*pkg*)

- **ID**: duplicate_find
- **Severity**: error
- **Explanation**:         The find_package() searches for a dependency and caches the result.         A second call will be silently ignored. In particular, if you         specify different arguments to the second call, those will not have         any effect at all, which is most likely not what you want.         

## duplicate include path ${*pkg*_INCLUDE_DIRS}

- **ID**: duplicate_build_include
- **Severity**: warning
- **Explanation**:         Include paths of packages listed in the find_package(catkin) command are added implicitly         by the ${catkin_INCLUDE_DIRS} variable. There is no need to add it a second time.         

## environment variables should not be used

- **ID**: env_var
- **Severity**: warning
- **Explanation**:         The behavior of your build should not depend on any         environment variables.         

## exported include path '*path*' does not exist

- **ID**: missing_export_include_path
- **Severity**: error
- **Explanation**:         You have listed an invalid include path in the INCLUDE_DIRS stanza of the         catkin_package() command.         

## exported library '*target*' cannot have different output name

- **ID**: export_lib_renamed
- **Severity**: error
- **Explanation**:         Due to a limitation of the catkin build system, the catkin_package()         library export function will break if the logical target name is not         equal to the actual library name.         

## exported library '*target*' is not a library

- **ID**: export_lib_not_lib
- **Severity**: error
- **Explanation**:         You listed a library in the LIBRARIES stanza of your catkin_package() call,         but it really is an executable.         

## exported library '*target*' is not installed

- **ID**: uninstalled_export_lib
- **Severity**: warning, error
- **Explanation**:         Your package can be used from the devel space but cannot be installed         properly, because a library that is exported via catkin_package() will         not be copied to the proper location.         

## exported package include path but no exported library

- **ID**: missing_export_lib
- **Severity**: warning
- **Explanation**:         Your package exports a package include path and builds at least one         library, which suggests that you may want to export the library to         other packages as well.         

## exported target '*target*' is not defined

- **ID**: undefined_target
- **Severity**: error
- **Explanation**:         Your package provides a CMake target to other packages, but the listed         target is not defined at all.         

## extra arguments in *cmd*()

- **ID**: endblock_args
- **Severity**: notice
- **Explanation**:         The catkin manual recommends that *cmd* and other end-of-block statements         have no arguments. If you have nested blocks, you should indent them         properly instead.         

## file '*script*' is executable but not installed

- **ID**: uninstalled_script
- **Severity**: warning
- **Explanation**:         Your package contains a file that is marked as executable but not         installed. If it is a script intended to be run (e.g. with rosrun), it         will not work outside the devel tree. If it is not an executable         script, you should fix the file permissions.         

## file setup.py found but no catkin_python_setup() call

- **ID**: missing_python_setup
- **Severity**: error
- **Explanation**:         The catkin_python_setup() call is required to properly configure python         modules, and the existing setup.py indicates that your package provides one or         more python modules.         

## find_package(*pkg*) before find_package(cmake_modules)

- **ID**: missing_cmake_modules
- **Severity**: warning
- **Explanation**:         You need to find_package() cmake_modules before you can use         one of its custom configuration modules.         

## find_package(*pkg*) has no REQUIRED option

- **ID**: missing_required
- **Severity**: warning, error
- **Explanation**:         The package cannot build without this dependency, so it should be         marked as REQUIRED accordingly. Use if(*pkg*_FOUND) clauses to use         optional packages.         

## generate_messages() called but no message declared

- **ID**: unused_generate_msg
- **Severity**: warning
- **Explanation**:         The generate_messages() call creates the messages, services, and actions         which are declared in your package. If your package does not supply any         of these, you do not have to call generate_messages() at all.         

## global variable '*var*' should contain project name

- **ID**: global_var_collision
- **Severity**: notice
- **Explanation**:         Global variables and options are stored in the cache.         You should prefix your variable names with the project name to         avoid name collisions with other packages.         

## include path '*path*' is exported but not used for the build

- **ID**: missing_build_include
- **Severity**: warning
- **Explanation**:         You have listed an include path in the INCLUDE_DIRS stanza of the         catkin_package() command, but that path is not mentioned in any         include_directories() call.         

## include paths '*path*' and '*parent_path*' are ambiguous

- **ID**: ambiguous_build_include
- **Severity**: warning
- **Explanation**:         You have used two include paths where one is a parent of         the other. Thus the same headers can be included with two different include paths         which may confuse users. It is recommended that you keep your include paths consistent.         

## install(*type* ... *dest*) is not one of the ${CATKIN_*_DESTINATION}s

- **ID**: install_destination
- **Severity**: warning
- **Explanation**:         Catkin provides a number of standard variables to specify         installation folders. You should use those to ensure that your         package will continue to work if the file system layout is         changed in the future.         

## installed target '*target*' is not defined

- **ID**: undefined_install_target
- **Severity**: error
- **Explanation**:         Your package installs a CMake target which is neither a library nor an         executable.         

## library output name '*output*' has redundant 'lib' prefix

- **ID**: redundant_lib_prefix
- **Severity**: notice
- **Explanation**:         Libraries are automatically prefixed with 'lib', so your library will end up         with a file name like 'lib*output*.so'. You can use         set_target_properties(... PROPERTIES OUTPUT_NAME ...) to give your library a         different file name without changing the target name.         

## line is not indented properly

- **ID**: indentation
- **Severity**: notice
- **Explanation**:         For better readability, each command should be placed on its own line.         if() and foreach() bodies should be indented by one or more extra spaces.         

## link_directories() must not be used for system depends

- **ID**: external_link_directory
- **Severity**: error
- **Explanation**:         Directories which are added to the search path with link_directories()         will not be propagated to dependent packages. Use find_package()         or find_library() with the appropriate PATHS or HINTS instead.         

## list *name* should be sorted

- **ID**: unsorted_list
- **Severity**: notice
- **Explanation**:         The catkin manual recommends that list element be kept in order.         

## malformed argument list: *msg*

- **ID**: argument_error
- **Severity**: warning
- **Explanation**:         You invoked a CMake command with a malformed argument list. Most         likely, you forgot to properly quote variables which may be empty         or undefined.         

## meaningless package description '*text*'

- **ID**: description_meaningless
- **Severity**: notice
- **Explanation**:         Your package description merely consists of typical filler words which         do not actually describe the contents of your package in a meaningful way.         

## message dependency '*pkg*' is not listed as *type*_depend

- **ID**: missing_msg_depend
- **Severity**: error
- **Explanation**:         Your messages depend on another package which is not listed as *type*_depend in         your package.xml         

## message dependency '*pkg*' is not listed in catkin_package()

- **ID**: missing_msg_catkin
- **Severity**: error
- **Explanation**:         Your messages depend on another package which is not in the CATKIN_DEPENDS         stanza of your catkin_package() call.         

## meta packages must not have *type*_depends

- **ID**: invalid_meta_depend
- **Severity**: error
- **Explanation**:         Meta packages do not contain code or data and are merely dependency lists.         As meta packages do neither build nor test anything, the only valid         dependency type is the run_depend.         

## missing *cmd*()

- **ID**: missing_cmd
- **Severity**: error
- **Explanation**:         You failed to call a command that is required for your package         to work. Please refer to the catkin build manual for details.         

## missing *type*_depend on '*pkg*'

- **ID**: missing_depend
- **Severity**: error
- **Explanation**:         Your package uses features of another package but you         failed to list this dependency in your package.xml         

## missing COMPONENTS keyword before '*pkg*'

- **ID**: missing_components
- **Severity**: error
- **Explanation**:         The find_package(catkin) call can add other catkin packages as         dependencies with the COMPONENTS keyword. The find_package() command         lists additional packages but has no COMPONENTS keyword.         

## missing find_package(*pkg*)

- **ID**: missing_find
- **Severity**: error
- **Explanation**:         You failed to call find_package() for a dependency of your package.         

## missing generate_messages()

- **ID**: missing_generate_msg
- **Severity**: error
- **Explanation**:         The generate_messages() call creates the messages, services, and actions         which are declared in your package by add_message_files(), add_service_files(),         and add_action_files() respectively.         

## missing include_directories(${catkin_INCLUDE_DIRS})

- **ID**: missing_catkin_include
- **Severity**: error
- **Explanation**:         You must add the catkin include paths to your include search list, or         you might experience build failures.         

## operands for operator *op* should be quoted strings

- **ID**: unquoted_string_op
- **Severity**: notice
- **Explanation**:         The catkin manual recommends that if() conditions with string operators should         have the operands enclosed in double quotes.         

## package '*pkg*' should be listed in catkin_package()

- **ID**: suggest_catkin_depend
- **Severity**: warning
- **Explanation**:         Your package configures another package as build dependency, it is listed as         run_depend in your package.xml, and its name suggests         that it contains ROS messages. In that case, you must add it to the         CATKIN_DEPENDS stanza of your catkin_package()         

## package description starts with boilerplate '*text*'

- **ID**: description_boilerplate
- **Severity**: notice
- **Explanation**:         Your package description starts with a number of typical filler words which         do not actually describe the contents of your package. Typically, you can         simply delete these words from the description, and it will still make sense         and be much more concise.         

## package exports *export* plugin but does not *type*_depend on '*pkg*'

- **ID**: plugin_depend
- **Severity**: warning
- **Explanation**:         Your package exports a plugin for another package, but fails to list said         package as a dependency.         

## project name '*name*' differs from package name

- **ID**: project_name
- **Severity**: error
- **Explanation**:         The CMake project name must be identical to the package name. For         backwards compatibility reasons, both names should also be identical         to the name of the source folder that contains the package.         

## subdirectory '*subdir*' contains a subproject

- **ID**: subproject
- **Severity**: warning
- **Explanation**:         Your package has an independent subproject. This can interact         with catkin in unusual ways and is strongly discouraged. No         further checks are performed in this subdirectory.         

## subdirectory '*subdir*' is added a second time

- **ID**: duplicate_subdir
- **Severity**: error
- **Explanation**:         You added another subdirectory with add_subdirectory() multiple times.         This can also happen if you accidentally created a loop where subdir A         adds subdir B, which adds subdir A again.         

## subdirectory '*subdir*' is missing

- **ID**: missing_subdir
- **Severity**: error
- **Explanation**:         You specified a subdirectory which does not exists or is unreadable.         

## subdirectory *subdir* is not in package

- **ID**: external_subdir
- **Severity**: error
- **Explanation**:         You added another subdirectory with add_subdirectory(), but the         specified path points outside of the package source directory.         

## target '*export_target*' depends on target '*target*' which is not installed

- **ID**: uninstalled_depend
- **Severity**: error
- **Explanation**:         Your package can be used from the devel space but cannot be installed         properly, because one of your installed targets depends on a library from         your package that is not installed as well.         

## target '*target*' has invalid characters in its output file name

- **ID**: invalid_target_output
- **Severity**: error
- **Explanation**:         The output file that your target is supposed to generate contains invalid         characters in its name. You probably forget to call set_target_properties(... PROPERTIES         OUTPUT_NAME ...)         

## target '*target*' is not installed

- **ID**: missing_install_target
- **Severity**: warning
- **Explanation**:         Your package can be used from the devel space but cannot be installed         properly, because the build target will not be copied to the proper location.         

## target name '*target*' might not be sufficiently unique

- **ID**: target_name_collision
- **Severity**: notice
- **Explanation**:         The CMake build system requires all target identifiers to be globally unique.         For this reason, it is highly recommended that you add the package name as in         '${PROJECT_NAME}_target' or '${PROJECT_NAME}/target'.         You can use set_target_properties(... PROPERTIES OUTPUT_NAME ...)         to give your target a different output file name (which does not have to         be unique if it is installed in a package-specific location).         

## test_depend '*pkg*' used without if(CATKIN_ENABLE_TESTING)

- **ID**: unguarded_test_depend
- **Severity**: error
- **Explanation**:         You have used a test dependency without properly guarding it by a         if(CATKIN_ENABLE_TESTING) block. You must add a proper build dependency if         you wish to use this package even if tests are disabled.         

## unconfigured build_depend on '*pkg*'

- **ID**: unconfigured_build_depend
- **Severity**: warning, error
- **Explanation**:         You declare a build dependency on another package but neither         call find_package() nor have it listed as catkin component in         the find_package(catkin) call.         

## unconfigured message dependency '*pkg*'

- **ID**: unconfigured_msg_depend
- **Severity**: error
- **Explanation**:         Your messages depend on another package which is neither find_package()'d         nor listed as a component in the find_package(catkin) call.         

## unknown *type*_depend '*pkg*'

- **ID**: unknown_depend
- **Severity**: error
- **Explanation**:         The specified dependency is neither a catkin package nor a known system dependency         from the rosdep database.         

## unknown package '*pkg*'

- **ID**: unknown_package
- **Severity**: error
- **Explanation**:         You have listed a package which is neither a catkin package nor a known system         dependency.         

## unused *type*_depend on '*pkg*'

- **ID**: unused_depend
- **Severity**: error
- **Explanation**:         You have a listed a package dependency but do not appear         to use any of the features it provides.         

## use ${PROJECT_NAME} instead of '*name*'

- **ID**: literal_project_name
- **Severity**: notice
- **Explanation**:         The catkin manual recommends that you use the ${PROJECT_NAME} variable instead         of the literal project name.         

## use find_package(*pkg*) instead of include(Find*pkg*.cmake)

- **ID**: find_by_include
- **Severity**: error
- **Explanation**:         The FindXXX.cmake modules are intended to be included by the find_package()         command.         

## use of link_directories() is strongly discouraged

- **ID**: link_directory
- **Severity**: warning
- **Explanation**:         Directories which are added to the search path with link_directories()         will not be propagated to dependent packages. You should avoid this         command or at least be aware that it might not work as expected in dependent         packages.         

## variable *var* is modified

- **ID**: critical_var_append
- **Severity**: warning
- **Explanation**:         You have appended extra data to a critical CMake variable.         This might break the build on different systems or affect         the global catkin workspace in unintended ways.         

## variable *var* is modified

- **ID**: immutable_var
- **Severity**: error
- **Explanation**:         You have modified a CMake variable that is initialized by CMake         itself and must not be modified under any circumstances.         

## variable *var* is overwritten

- **ID**: critical_var_overwrite
- **Severity**: error
- **Explanation**:         You have overwritten a critical CMake variable and its original         content is lost. This will most likely break the build on         different systems or affect the global catkin workspace in         unintended ways.         

## variable CMAKE_BUILD_TYPE is overwritten unconditionally

- **ID**: cmake_build_type
- **Severity**: error
- **Explanation**:         If you wish to provide a default value for CMAKE_BUILD_TYPE, make         sure that you do not overwrite user preferences. You should guard         the set() command with an appropriate if(NOT CMAKE_BUILD_TYPE) block.         

