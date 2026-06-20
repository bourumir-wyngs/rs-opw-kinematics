#!/bin/bash
# Move tag 2.0.0 to the current tip of main branch

set -euo pipefail

# Delete the old tag locally and remotely
git tag -d 2.0.0
git push origin :refs/tags/2.0.0

# Create a new tag at the current HEAD
git tag 2.0.0

# Push the new tag
git push origin 2.0.0

echo "Tag 2.0.0 moved to $(git rev-parse HEAD)"
