#!/bin/bash
# Check the changed code against the clang-format code style for the repository.
# Uses a copy of https://github.com/llvm/llvm-project/blob/main/clang/tools/clang-format/git-clang-format
# located in the scripts directory to check the formatting of the diff against main.
# The script will exit with a 1 if a diff is detected and a 0 if it isn't.
# If a diff is detected, you should run the script to fix the issues locally and commit again.

# Disable exit on error so we can handle return codes manually
set +e

# directory where this script is located
SCRIPT_DIR="./scripts/tools"


# Use first argument as base ref, default to "main" if not provided
BASE_REF=${1:-main}

# Run git-clang-format to check diffs
git-clang-format.py "$BASE_REF" --diff
rc1=$?

# Run git-clang-format to show diffstat
git-clang-format.py "$BASE_REF" --diffstat
rc2=$?

# Combine return codes to detect any formatting errors
FORMAT_ERROR=$(( rc1 || rc2 ))

if [ "$FORMAT_ERROR" -ne 0 ]; then
  echo
  echo "Formatting issues found! Run the following command to fix them:"
  echo
  echo "    $SCRIPT_DIR/git-clang-format $BASE_REF"
  echo 
fi

exit $FORMAT_ERROR
