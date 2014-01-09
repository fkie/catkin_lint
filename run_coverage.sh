#!/bin/bash
set -e

version=

if [[ -n "$1" && "$1" != -* ]]
then
    version=-$1
    shift
fi

x()
{
    echo '$' "$@"
    "$@"
}

x rm -f .coverage
x coverage${version} run "$( which nosetests${version} )"
x coverage${version} report "$@" $( find src -name "*.py" )

