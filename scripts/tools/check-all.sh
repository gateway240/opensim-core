#!/bin/bash

# Don't exit immediately; we want to run all checks
set +e

# Define the directory containing the check scripts
SCRIPT_DIR="./scripts/tools"

# Define the list of scripts
scripts=(
  "check-format.sh"
  "check-iwyu.sh"
  "check-tabs.sh"
  "check-tidy.sh"
)

# Track failures
declare -a failed_scripts=()

# Run each check script
for script in "${scripts[@]}"; do
  script_path="${SCRIPT_DIR}/${script}"
  echo "Running $script..."
  if [[ -x "$script_path" ]]; then
    "$script_path" main
    exit_code=$?
    if [[ $exit_code -ne 0 ]]; then
      failed_scripts+=("$script")
    fi
  else
    echo "⚠️  Skipping $script: not found or not executable at $script_path"
    failed_scripts+=("$script (not found or not executable)")
  fi
done

# Summary
echo ""
echo "===== Check Summary ====="
if [ ${#failed_scripts[@]} -eq 0 ]; then
  echo "🎉 All checks passed!"
  exit 0
else
  echo "❌ The following checks failed:"
  for failed in "${failed_scripts[@]}"; do
    echo " - $failed"
  done
  exit 1
fi
