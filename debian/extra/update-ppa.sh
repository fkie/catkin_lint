#!/bin/bash
set -e
declare -A ubuntu=( [precise]=12.04 [trusty]=14.04 [utopic]=14.10 [vivid]=15.04 [wily]=15.10 )
sa="-sa"
./debian/rules make-orig-tar
for distro in precise trusty utopic vivid
do
    sed -i -e '1s/\(~.\+\)\?) [a-z]\+;/~ubuntu'${ubuntu[$distro]}') '$distro';/' debian/changelog
    dpkg-buildpackage -S $sa -i\\..*
    version="$( dpkg-parsechangelog | grep Version | cut -d' ' -f2 )"
    dput ppa:roehling/latest ../catkin-lint_${version}_source.changes
    sa="-sd"
done
git checkout -- debian/changelog

