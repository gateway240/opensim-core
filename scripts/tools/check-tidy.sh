#!/bin/bash
# Check the repository with clang-tidy
# Uses script from https://clang.llvm.org/extra/doxygen/run-clang-tidy_8py_source.html Check the changed code against clang-tidy diagnostics for the repository.
# Uses a script (clang-git-tidy.py) located in the scripts directory to run clang-tidy
# on diffs against the given base reference (defaults to "main").
# The script exits with 1 if issues are found, 0 if clean.

# Directory where this script is located
SCRIPT_DIR="./scripts/tools"

# Use first argument as base ref, default to "main"
BASE_REF=${1:-main}
# where your compile_commands.json lives
BUILD_DIR=${2:-"../opensim-core-build"} 

LOG_DIR="logs"
# Create log directory
mkdir -p "$LOG_DIR"

LOG_FILE="clang-tidy.log"

if [ ! -f "$BUILD_DIR/compile_commands.json" ]; then
  echo "compile_commands.json not found in $BUILD_DIR"
  echo "Make sure your project is built with CMake and that -DCMAKE_EXPORT_COMPILE_COMMANDS=ON is set."
  exit 1
fi

# Filter to only valid source/header files
VALID_FILES=$(git diff --name-only "$BASE_REF" | grep -E '\.(c|cc|cpp|cxx|h|hpp|hxx)$')

if [ -z "$VALID_FILES" ]; then
  echo "✅ No C/C++ source changes to check against $BASE_REF."
  exit 0
fi

# Run clang-tidy and capture output
OUTPUT_FILE="$LOG_DIR/$LOG_FILE"
"$SCRIPT_DIR/run-clang-tidy.py" \
  -config-file=".clang-tidy" \
  -header-filter=.* \
  -p "$BUILD_DIR" \
  $VALID_FILES 2>&1 | tee "$OUTPUT_FILE"

if grep -qE 'warning:|error:' "$OUTPUT_FILE"; then
  echo
  echo "⚠️  clang-tidy issues found in changed files!"
  echo "📄 See full output in: $OUTPUT_FILE"
  echo "💡 To auto-fix (where possible), run:"
  echo
  echo "  $SCRIPT_DIR/run-clang-tidy.py -config-file=.clang-tidy -header-filter=.* -p \"$BUILD_DIR\" -fix $VALID_FILES"
  echo
  exit 1
else
  echo "✅ clang-tidy found no issues in changed files."
  echo "📄 Output log saved to: $OUTPUT_FILE"
  exit 0
fi