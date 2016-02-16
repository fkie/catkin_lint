catkin_lint
############

Overview
========

**catkin_lint** checks package configurations for the
`catkin <https://github.com/ros/catkin>`_ build system of `ROS <http://www.ros.org>`_.

Installation
============

Install Ubuntu Packages
-----------------------

Prebuilt packages are available from the `ROS repository <http://packages.ros.org/>`_.
If you have installed ROS already, downloading **catkin_lint** is as simple as::

    sudo apt-get install python-catkin-lint

Alternatively, you can use `Timo's PPA <https://launchpad.net/~roehling/+archive/latest>`_ on Launchpad::

    sudo add-apt-repository ppa:roehling/latest
    sudo apt-get update
    sudo apt-get install python-catkin-lint

Download from PyPI
------------------

You can download and install **catkin_lint** from the `Python Package Index <https://pypi.python.org/pypi/catkin_lint>`_
with::

    sudo pip install catkin_lint

Install from Source
-------------------

You can clone **catkin_lint** from `GitHub <https://github.com/fkie/catkin_lint>`_::

    git clone https://github.com/fkie/catkin_lint
    cd catkin_lint
    sudo python setup.py install

Build status of latest version:

.. image:: https://travis-ci.org/fkie/catkin_lint.png?branch=master
   :target: https://travis-ci.org/fkie/catkin_lint
.. image:: https://codecov.io/github/fkie/catkin_lint/coverage.svg?branch=master
    :target: https://codecov.io/github/fkie/catkin_lint?branch=master

Build your own Debian packages
------------------------------

If your distribution is not supported, you can build your own packages::

    sudo apt-get install dpkg-dev
    git clone https://github.com/fkie/catkin_lint
    cd catkin_lint
    git checkout debian
    ./debian/rules make-orig-tar
    dpkg-buildpackage -tc -uc -us -i\\..*
    sudo dpkg -i ../python-catkin-lint_*_all.deb

Build status of packaging branch:

.. image:: https://travis-ci.org/fkie/catkin_lint.png?branch=debian
   :target: https://travis-ci.org/fkie/catkin_lint

Running
=======

**catkin_lint** runs a static analysis of the ``package.xml`` and
``CMakeLists.txt`` files. It can detect and report a number of common
problems.

If **catkin_lint** is invoked with one or more paths as parameters, it
searches for packages recursively and checks all of them. Alternatively, the
``--pkg`` option can be used to add the path of a particular ROS package.

If neither paths nor packages are specified, **catkin_lint** looks for a
package in the current working directory.

A more detailed list of command line options can be obtained by running
``catkin_lint --help``.

Limitations
===========

**catkin_lint** emulates a limited subset of CMake. It does not
evaluate boolean expressions in ``if()`` clauses, emulates ``find_package()``
calls with mock values, and ignores ``function()`` definitions.

Catkin Build Integration
========================

It is recommended to run **catkin_lint** at workspace configuration time.
The simplest way is to symlink ``/opt/ros/$ROS_DISTRO/share/catkin/cmake/toplevel.cmake``
to your ``$WSDIR/src`` folder manually (do not use ``catkin_init_workspace``).
Then add the following ``CMakeLists.txt``::

    cmake_minimum_required(VERSION 2.8.3)
    find_program(CATKIN_LINT catkin_lint)
    if(CATKIN_LINT)
        execute_process(COMMAND "${CATKIN_LINT}" "${CMAKE_SOURCE_DIR}" RESULT_VARIABLE lint_result)
        if(NOT ${lint_result} EQUAL 0)
            message(FATAL_ERROR "catkin_lint failed")
        endif()
    endif()
    include(toplevel.cmake)

Diagnostic Levels
=================

**catkin_lint** has messages in three different categories:
errors, warnings, and notices. The ``-W`` option controls which problems
are reported to the user:

- ``-W0``: only errors are reported
- ``-W1``: errors and warnings are reported (this is the default)
- ``-W2``: errors, warnings, and notices are reported

Normally, **catkin_lint** returns a non-zero exit code if and only
if errors occurred. The ``--strict`` option causes **catkin_lint** to
treat any reported problem as error.

Errors
------

Errors are severe enough to break the build and/or produce unintended
side effects. Usually, they violate the rules outlined in the
`catkin manual <http://docs.ros.org/api/catkin/html/>`_

Warnings
--------

Potential errors which may indicate a bug in your package but may be
justified for reasons **catkin_lint** cannot discern. Constructs which
trigger a warning can usually be modified in a way that is functionally
equivalent but more robust.

Notices
-------

Issues which are not objectionable from a technical view point but
should  be addressed to improve the quality of the package. Many notices
highlight violations of the recommendations and best practises from the
catkin manual.

Contribution
============

If you would like to contribute, you are very welcome to do so.
Please contact `@roehling <https://github.com/roehling>`_ first
to avoid any duplication of work.

Known Issues
============

* Ubuntu distributions which have reached their End-of-Life will no longer
  receive updated package versions.

