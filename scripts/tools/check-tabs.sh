#!/bin/bash
# Ensure that there are no tabs in source code.
# GREP returns 0 (true) if there are any matches, and
# we don't want any matches. If there are matches,
# print a helpful message, and make the test fail by using "false".
# The GREP command here checks for any tab characters in the the files
# that match the specified pattern. GREP does not pick up explicit tabs
# (e.g., literally a \t in a source file).

if grep --line-num --recursive \
        --exclude-dir="*dependencies*" \
        --exclude-dir="*snopt*" \
        --include={CMakeLists.txt,*.cpp,*.c,*.h} \
        -P "\t" .; then
    echo "Tabs found in the lines shown above. See CONTRIBUTING.md about tabs."
    exit 1  # Exit with failure status
fi
