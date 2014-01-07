#!/usr/bin/env python
# coding=utf-8

__author__ = "Timo Röhling <timo.roehling@fkie.fraunhofer.de>"
__copyright__ = "Copyright (c) 2013 Fraunhofer FKIE"
__license__ = "BSD"

import cmake
def add_all_checks(linter):
    cmake.project(linter)
    cmake.special_vars(linter)
    cmake.singleton_commands(linter)
