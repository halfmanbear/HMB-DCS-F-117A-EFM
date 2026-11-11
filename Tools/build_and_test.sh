#!/bin/bash
# Release x64 build (warnings are errors) followed by the regression suite.
# Set F117_MOD_DIR to include the tests that need the companion mod repository.
cd "$(dirname "$0")/.."
MSB="/c/Program Files/Microsoft Visual Studio/2022/Community/MSBuild/Current/Bin/MSBuild.exe"
"$MSB" "EFM Source Code/F117.sln" -p:Configuration=Release -p:Platform=x64 -v:minimal -nologo > build/msbuild.log 2>&1
build_status=$?
grep -E "warning|error|->" build/msbuild.log | sed -E 's/ \[C:.*//' | sort -u
[ $build_status -eq 0 ] || exit $build_status
python tests/run_all.py build/tests > build/tests.log 2>&1
test_status=$?
tail -${TAIL:-3} build/tests.log
grep FAIL build/tests.log
exit $test_status
