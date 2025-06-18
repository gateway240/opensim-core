#!/bin/bash
# Check the changed code against the clang-format code style for the repository.
# Uses a copy of https://github.com/llvm/llvm-project/blob/main/clang/tools/clang-format/git-clang-format
# located in the scripts directory to check the formatting of the diff against main.
# The script will exit with a 1 if a diff is detected and a 0 if it isn't.
# If a diff is detected, you should run the script to fix the issues locally and commit again.

# Disable exit on error so we can handle return codes manually
set +e

SCRIPT_DIR="./scripts/tools"
BASE_REF=${1:-main}
LOG_DIR="logs"
LOG_FILE="$LOG_DIR/clang-format.log"

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Clear or create the log file
: > "$LOG_FILE"

# Run git-clang-format with logging
"$SCRIPT_DIR/git-clang-format.py" "$BASE_REF" --diff | tee -a "$LOG_FILE"
rc1=${PIPESTATUS[0]}

"$SCRIPT_DIR/git-clang-format.py" "$BASE_REF" --diffstat | tee -a "$LOG_FILE"
rc2=${PIPESTATUS[0]}

# Combine exit codes
FORMAT_ERROR=$(( rc1 || rc2 ))

if [ "$FORMAT_ERROR" -ne 0 ]; then
  echo | tee -a "$LOG_FILE"
  echo "❌ Formatting issues found! Run the following command to fix them:" | tee -a "$LOG_FILE"
  echo | tee -a "$LOG_FILE"
  echo "    $SCRIPT_DIR/git-clang-format $BASE_REF" | tee -a "$LOG_FILE"
  echo | tee -a "$LOG_FILE"
else
  echo "✅ No formatting issues found." | tee -a "$LOG_FILE"
fi

exit $FORMAT_ERROR

