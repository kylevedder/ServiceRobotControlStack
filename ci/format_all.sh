#!/bin/bash
header_files=$(find catkin_ws/src/control_stack/ -name \*.h)
source_files=$(find catkin_ws/src/control_stack/ -name \*.cpp)

echo "Formatting Headers!"

for file in $header_files; do
    # echo "Formatting: $file"
    if [[ "$file" != *"control_stack/lib"* ]]; then
        clang-format -i "$file"
    fi
done

echo "Formatting Source!"

for file in $source_files; do
    # echo "Formatting: $file"
    if [[ "$file" != *"control_stack/lib"* ]]; then
        clang-format -i "$file"
    fi
done

echo "Format done!"
