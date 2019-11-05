#!/bin/bash
src/ServiceRobotControlStack/ci/cpplint.py $(find src/ServiceRobotControlStack/control_stack -name '*.cc' -o -name '*.h')
ret=$?
if [ $ret -ne 0 ]; then
    echo -e "CI Lint Failures!!!";
    exit -1;
fi
echo -e "CI Lint Passed...";
exit 0;
