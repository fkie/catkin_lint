#!/usr/bin/env python
# coding=utf-8
#
# catkin_lint
# Copyright 2013-2022 Fraunhofer FKIE
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
# IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from setuptools import setup
import os


def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


setup(
    name="catkin_lint",
    description="Check catkin packages for common errors",
    long_description=read("README.rst"),
    author="Timo Röhling",
    author_email="timo.roehling@fkie.fraunhofer.de",
    license="BSD",
    url="https://github.com/fkie/catkin_lint",
    keywords=["catkin", "ROS"],
    packages=["catkin_lint", "catkin_lint.checks"],
    package_dir={"": "src"},
    data_files=[
        ("share/bash-completion/completions", ["shell/bash/catkin_lint"]),
        ("share/fish/vendor_completions.d", ["shell/catkin_lint.fish"]),
    ],
    scripts=["bin/catkin_lint"],
    use_scm_version={"write_to": "src/catkin_lint/_version.py"},
    setup_requires=["setuptools_scm"],
    install_requires=["catkin_pkg", "lxml", 'configparser<5;python_version<"3"'],
    extras_require={
        "ros": ["rosdistro", "rosdep"],
    },
    tests_require=["nose2", "coverage", "mock"],
    test_suite="nose2.collector.collector",
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: BSD License",
        "Topic :: Software Development :: Quality Assurance",
        "Environment :: Console",
        "Operating System :: OS Independent",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
    ],
    entry_points={
        "catkin_tools.commands.catkin.verbs": [
            "lint = catkin_lint.main:description",
        ],
    },
)
