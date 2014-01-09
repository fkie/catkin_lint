#!/usr/bin/env python
# coding=utf-8

__author__ = "Timo Röhling <timo.roehling@fkie.fraunhofer.de>"
__copyright__ = "Copyright (c) 2013,2014 Fraunhofer FKIE"
__license__ = "BSD"


def all(linter):
    import catkin_lint.checks.build as build
    import catkin_lint.checks.manifest as manifest
    import catkin_lint.checks.misc as misc
    import catkin_lint.checks.python as python
    import catkin_lint.checks.fkie as fkie

    linter.require(build.all)
    linter.require(manifest.all)
    linter.require(misc.all)
    linter.require(python.all)
    linter.require(fkie.all)
