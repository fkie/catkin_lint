#!/bin/bash
set -e
pkgname=$( dpkg-parsechangelog | grep ^Source: | cut -d' ' -f2 )
pkgversion=$( dpkg-parsechangelog | grep ^Version: | cut -d' ' -f2 )
build_type=$(<debian/source/format)
if [ "$build_type" = "3.0 (quilt)" ]
then
	debian/rules make-orig-tar
fi
dpkg-buildpackage -tc -i"\\..*" "$@"
changefile=$( readlink -f ../${pkgname}_${pkgversion}_*.changes )
buildfiles=( $( grep -A999 Files: "${changefile}" | awk '{ print $5 }' ) )

cleanup()
{
  cd $(dirname "${changefile}" )
  rm "${buildfiles[@]}" "${changefile}"
}

trap cleanup EXIT

tar -C "$(dirname "${changefile}")" -cf ../catkin-lint_${pkgversion}.tar "${buildfiles[@]}" "$(basename "${changefile}")"

