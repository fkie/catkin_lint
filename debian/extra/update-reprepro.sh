#!/bin/bash
set -e
build_type=$(<debian/source/format)
sa="-sa"
if [ "$build_type" = "3.0 (quilt)" ]
then
	debian/rules make-orig-tar
fi
export BUILD_ARCH=amd64
export BUILD_DISTROS="precise trusty"
export DPKG_BUILDPACKAGE_OPTS="-i\\..*"
../builddeb_distro.sh -sa
git checkout -- debian/changelog

