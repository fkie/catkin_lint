#!/usr/bin/env python
# coding=utf-8

__author__ = "Timo Röhling <timo.roehling@fkie.fraunhofer.de>"
__copyright__ = "Copyright (c) 2013,2014 Fraunhofer FKIE"
__license__ = "BSD"

import build
import cmake
import depends
import misc


def everything(linter):
    build.targets(linter)
    build.plugins(linter)
    cmake.project(linter)
    cmake.special_vars(linter)
    cmake.singleton_commands(linter)
    misc.package_description(linter)
    depends.catkin_depends(linter)
    depends.message_generation(linter)

