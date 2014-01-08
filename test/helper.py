from catkin_lint.main import CatkinEnvironment, CMakeLinter 
from catkin_pkg.package import Package, Dependency, Person
from catkin_lint.checks import everything

def create_env(catkin_pkgs=[], system_pkgs=[]):
    env = CatkinEnvironment(rosdep_view={})
    env.known_catkin_pkgs = set(catkin_pkgs)
    env.known_other_pkgs = set(system_pkgs)
    return env


def create_manifest(name, description="", buildtool_depends=[ "catkin" ], build_depends=[], run_depends=[], test_depends=[]):
    return Package(
        name=name,
        version="0.0.0",
        description=description,
        maintainers=[ Person("John Foo", "foo@bar.com") ],
        buildtool_depends=[ Dependency(d) for d in buildtool_depends ],
        build_depends=[ Dependency(d) for d in build_depends ],
        run_depends=[ Dependency(d) for d in run_depends ],
        test_depends=[ Dependency(d) for d in test_depends ]
    )


def mock_lint(env, manifest, cmakelist, checks=everything):
    linter = CMakeLinter(env)
    def get_cmakelist(filename): return cmakelist
    linter._read_file = get_cmakelist
    checks(linter)
    linter.lint ("/mock-path", manifest)
    return [ m[4] for m in linter.messages ]
