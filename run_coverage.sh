#!/bin/bash
set -e

version=${1:-2.7}
rm -f .coverage
coverage-${version} run $( which nosetests-${version} ) -s
coverage-${version} report $( find src -name "*.py" )

