#!/bin/bash
# Check the changed code against the include-what-you-use tool

# Run include-what-you-use (IWYU) on changed C/C++ files using compile_commands.json

set -e

BASE_REF=${1:-main}
BUILD_DIR=${2:-"../opensim-core-build"}
IWYU_BIN=${3:-"iwyu_tool.py"}
LOG_DIR="logs"
# Create log directory
mkdir -p "$LOG_DIR"

# Ensure compile_commands.json exists
if [ ! -f "$BUILD_DIR/compile_commands.json" ]; then
  echo "❌ compile_commands.json not found in $BUILD_DIR"
  exit 1
fi

# Get list of changed C/C++ files
CHANGED_FILES=$(git diff --name-only "$BASE_REF" | grep -E '\.(c|cc|cpp|cxx|h|hpp|hxx)$')

if [ -z "$CHANGED_FILES" ]; then
  echo "✅ No C/C++ source file changes to check against $BASE_REF"
  exit 0
fi

# Run IWYU on each changed file
echo "📋 Running include-what-you-use on changed files..."
echo "🔍 Using compile_commands.json from: $BUILD_DIR"
echo

HAS_ISSUES=0

for FILE in $CHANGED_FILES; do
  echo "🔎 Checking $FILE ..."
  OUTPUT_FILE="$LOG_DIR/iwyu_$(basename "$FILE").log"

  "$IWYU_BIN" -p "$BUILD_DIR" "$FILE" 2>&1 | tee "$OUTPUT_FILE"

  if grep -q -E "should add these lines|should remove these lines" "$OUTPUT_FILE"; then
    echo "⚠️  IWYU suggestions found in $FILE"
    HAS_ISSUES=1
  else
    echo "✅ No IWYU issues in $FILE"
    rm "$OUTPUT_FILE"
  fi
done

echo
if [ "$HAS_ISSUES" -ne 0 ]; then
  echo "⚠️  include-what-you-use found issues."
  echo "📁 See log files in: $LOG_DIR"
  exit 1
else
  echo "✅ All checked files are clean."
  rm -r "$LOG_DIR"
  exit 0
fi
