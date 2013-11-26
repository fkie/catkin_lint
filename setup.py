#!/usr/bin/env python
# coding=utf-8

from distutils.core import setup

setup(
  name          = "catkin_lint",
  description   = "Check catkin packages for common errors",
  author        = "Timo Röhling",
  author_email  = "timo.roehling@fkie.fraunhofer.de",
  license       = "BSD",
  packages      = [],
  package_dir   = {},
  scripts       = [ "catkin_lint" ],
  version       = "1.0.1",
  requires      = [ "catkin_pkg" ]
)

