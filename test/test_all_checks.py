import unittest
from .helper import create_env, create_manifest, mock_lint

import sys
sys.stderr = sys.stdout


class AllChecksTest(unittest.TestCase):
    def test_project(self):
        env = create_env(catkin_pkgs=[ "catkin", "foo", "foo_msgs" ])
        pkg = create_manifest("mock", description="Cool Worf", build_depends=[ "foo", "foo_msgs" ], run_depends=[ "foo_msgs" ])
        result = mock_lint(env, pkg, 
            """\
            project(mock)
            find_package(catkin REQUIRED COMPONENTS foo foo_msgs)
            catkin_package(CATKIN_DEPENDS foo_msgs)
            include_directories(${catkin_INCLUDE_DIRS})
            add_executable(mock_test src/test.cpp)
            target_link_libraries(${catkin_LIBRARIES})
            install(TARGETS mock_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
            """)
        self.assertEqual([], result)

