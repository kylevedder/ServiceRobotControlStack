#!/bin/bash
header_files=$(find control_stack/ -name \*.h)
source_files=$(find control_stack/ -name \*.cpp)

echo "Formatting Headers!"

for file in $header_files; do
    echo "Formatting: $file"
    clang-format -i "$file"
done

echo "Formatting Source!"

for file in $source_files; do
    echo "Formatting: $file"
    clang-format -i "$file"
done

echo "Format done!"