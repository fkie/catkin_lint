#!/bin/bash
set -e
sa="-sa"
./debian/rules make-orig-tar
for distro in precise saucy trusty
do
    sed -i -e '1s/\(~[a-z]\+\)\?) [a-z]\+;/~'$distro') '$distro';/' debian/changelog
    dpkg-buildpackage -S $sa -i\\..*
    version="$( dpkg-parsechangelog | grep Version | cut -d' ' -f2 )"
    dput ppa:roehling/latest ../catkin-lint_${version}_source.changes
    sa="-sd"
done
git checkout -- debian/changelog

