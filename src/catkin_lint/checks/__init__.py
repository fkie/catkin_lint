#!/usr/bin/env python
# coding=utf-8

__author__ = "Timo Röhling <timo.roehling@fkie.fraunhofer.de>"
__copyright__ = "Copyright (c) 2013,2014 Fraunhofer FKIE"
__license__ = "BSD"


def all(linter):
    import catkin_lint.checks.build as build
    import catkin_lint.checks.cmake as cmake
    import catkin_lint.checks.depends as depends
    import catkin_lint.checks.misc as misc

    linter.require(build.all)
    linter.require(cmake.all)
    linter.require(depends.all)
    linter.require(misc.all)
