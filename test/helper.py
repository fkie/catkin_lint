from catkin_lint.linter import CMakeLinter, LintInfo
from catkin_lint.environment import CatkinEnvironment
from catkin_pkg.package import Package, Dependency, Person, Export
from catkin_lint.checks import all
from catkin_lint.util import iteritems

import os

def create_env(catkin_pkgs=[ "catkin", "message_generation", "message_runtime", "other_catkin", "other_msgs", "first_pkg", "second_pkg" ], system_pkgs=[ "other_system" ]):
    env = CatkinEnvironment(use_rosdep=False, use_cache=False)
    env.known_catkin_pkgs = set(catkin_pkgs)
    env.known_other_pkgs = set(system_pkgs)
    return env


def create_manifest(name, description="", buildtool_depends=[ "catkin" ], build_depends=[], run_depends=[], test_depends=[], meta=False):
    return Package(
        name=name,
        version="0.0.0",
        package_format=1,
        description=description,
        maintainers=[ Person("John Foo", "foo@bar.com") ],
        buildtool_depends=[ Dependency(d) for d in buildtool_depends ],
        build_depends=[ Dependency(d) for d in build_depends ],
        run_depends=[ Dependency(d) for d in run_depends ],
        test_depends=[ Dependency(d) for d in test_depends ],
        exports=[ Export("metapackage") ] if meta else []
    )


def create_manifest2(name, description="", buildtool_depends=[ "catkin" ], build_depends=[], depends=[], buildtool_export_depends=[], build_export_depends=[], exec_depends=[], test_depends=[], meta=False):
    return Package(
        name=name,
        version="0.0.0",
        package_format=2,
        description=description,
        maintainers=[ Person("John Foo", "foo@bar.com") ],
        depends=[ Dependency(d) for d in depends ],
        buildtool_depends=[ Dependency(d) for d in buildtool_depends ],
        build_depends=[ Dependency(d) for d in build_depends ],
        buildtool_export_depends=[ Dependency(d) for d in buildtool_export_depends ],
        build_export_depends=[ Dependency(d) for d in build_export_depends ],
        exec_depends=[ Dependency(d) for d in exec_depends ],
        test_depends=[ Dependency(d) for d in test_depends ],
        exports=[ Export("metapackage") ] if meta else []
    )


def mock_lint(env, manifest, cmakelist, checks=all, indentation=False, return_var=False):
    linter = CMakeLinter(env)
    if type(cmakelist) is dict:
        tmp = {}
        for key, value in iteritems(cmakelist):
            tmp[os.path.normpath(key)] = value
        cmakelist = tmp
    def get_cmakelist(filename):
        if type(cmakelist) is dict:
            if filename in cmakelist:
                 return cmakelist[filename]
            else:
                return ""
        else:
            if filename == os.path.normpath("/mock-path/CMakeLists.txt"): return cmakelist
            return ""
    linter._read_file = get_cmakelist
    if checks is not None: linter.require(checks)
    info = LintInfo(env)
    linter.lint (os.path.normpath("/mock-path"), manifest, info)
    if not indentation:
        linter.messages = [ m for m in linter.messages if m.id != "INDENTATION" ]
    if return_var:
        return info.var
    else:
        return [ m.id for m in linter.messages ]
