#! /bin/sh
find . -name "*.h" -o -name "*.c" -o -name "*.rc"  -o -name "*.cpp" -o -name "*.java" -o -name "*.mk" -o -name "*.sh" -o -name "*.pl" > cscope.files
cscope -Rbkq -i cscope.files
ctags -R
