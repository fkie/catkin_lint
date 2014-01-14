#!/usr/bin/env python
# coding=utf-8

__author__ = "Timo Röhling <timo.roehling@fkie.fraunhofer.de>"
__copyright__ = "Copyright (c) 2013,2014 Fraunhofer FKIE"
__license__ = "BSD"


def all(linter):
    from .build import all as build_all
    from .manifest import all as manifest_all
    from .misc import all as misc_all
    from .python import all as python_all
    from .fkie import all as fkie_all

    linter.require(build_all)
    linter.require(manifest_all)
    linter.require(misc_all)
    linter.require(python_all)
    linter.require(fkie_all)
