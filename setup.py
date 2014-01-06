#!/usr/bin/env python
# coding=utf-8

from distutils.core import setup
import sys
sys.path.insert(0, "lib")
from catkin_lint import __version__ as catkin_lint_version

setup(
  name          = "catkin_lint",
  description   = "Check catkin packages for common errors",
  author        = "Timo Röhling",
  author_email  = "timo.roehling@fkie.fraunhofer.de",
  license       = "BSD",
  url           = "https://github.com/fkie/catkin_lint",
  download_url  = "https://github.com/fkie/catkin_lint/tarball/%s" % catkin_lint_version,
  packages      = [ "catkin_lint" ],
  package_dir   = { "" : "lib" },
  scripts       = [ "catkin_lint" ],
  version       = catkin_lint_version,
  requires      = [ "catkin_pkg" ],
  classifiers   = [
                    "Development Status :: 4 - Beta",
                    "Intended Audience :: Developers",
                    "License :: OSI Approved :: BSD License",
                    "Topic :: Software Development :: Quality Assurance",
                    "Topic :: Scientific/Engineering :: Artificial Intelligence",
                  ]
)

