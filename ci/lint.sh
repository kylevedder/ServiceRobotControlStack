#!/bin/bash
find catkin_ws/src/control_stack/src/ -name '*.cpp' -o -name '*.h' > /tmp/lint_files
find catkin_ws/src/control_stack/include/ -name '*.cpp' -o -name '*.h' >> /tmp/lint_files
find catkin_ws/src/control_stack/test/ -name '*.cpp' -o -name '*.h' >> /tmp/lint_files
ci/cpplint.py $(cat /tmp/lint_files)
ret=$?
if [ $ret -ne 0 ]; then
    echo -e "CI Lint Failures!!!";
    exit -1;
fi
echo -e "CI Lint Passed...";
exit 0;
