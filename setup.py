#!/usr/bin/env python
# coding=utf-8

from setuptools import setup
import sys
import os


def catkin_lint_version():
    from catkin_lint import __version__
    return __version__


def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

setup(
    name="catkin_lint",
    description="Check catkin packages for common errors",
    long_description=read("README.rst"),
    author="Timo Röhling",
    author_email="timo.roehling@fkie.fraunhofer.de",
    license="BSD",
    url="https://github.com/fkie/catkin_lint",
    download_url="https://github.com/fkie/catkin_lint/tarball/%s" % catkin_lint_version(),
    keywords=["catkin", "ROS"],
    packages=["catkin_lint", "catkin_lint.checks"],
    package_dir={"": "src"},
    data_files=[("share/bash-completion/completions", ["bash/catkin_lint"])],
    scripts=["bin/catkin_lint"],
    version=catkin_lint_version(),
    install_requires=["catkin_pkg", "lxml"],
    test_suite="nose.collector",
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: BSD License",
        "Topic :: Software Development :: Quality Assurance",
        "Environment :: Console",
        "Operating System :: OS Independent",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3"
    ],
    entry_points={
        "catkin_tools.commands.catkin.verbs": [
            "lint = catkin_lint.main:description",
        ],
    },
)
