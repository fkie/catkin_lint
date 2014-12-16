#!/bin/bash
set -e
build_type=$(<debian/source/format)
sa="-sa"
if [ "$build_type" = "3.0 (quilt)" ]
then
	debian/rules make-orig-tar
fi
export BUILD_ARCH=amd64
export DPKG_BUILDPACKAGE_OPTS="-i\\..*"
for distro in precise trusty
do
	sed -i -e '1s/\(~[a-z]\+\)\?) [a-z]\+;/~'$distro') '$distro';/' debian/changelog
	../builddeb.sh "$sa"
	sa="-sd"
done
git checkout -- debian/changelog

