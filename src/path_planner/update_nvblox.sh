#!/usr/bin/env bash
# NVBlox vendored update script.
#
# Usage:
#   ./update_nvblox.sh              # reinstall default (v0.0.6)
#   ./update_nvblox.sh v0.0.9       # switch to given tag
#
# Steps:
#   1) clone into a temp directory
#   2) checkout the requested tag
#   3) rsync to path_planner/nvblox/ (exclude build artifacts)
#   4) reapply GCC 13 compat patch
#   5) update NVBLOX_VERSION.txt

set -eu

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DEST="$SCRIPT_DIR/nvblox"
VERSION="${1:-v0.0.6}"
TMPDIR="$(mktemp -d)"
trap 'rm -rf "$TMPDIR"' EXIT

echo "[update] Target version: $VERSION"
echo "[update] Cloning to $TMPDIR..."
git clone --quiet https://github.com/nvidia-isaac/nvblox.git "$TMPDIR/nvblox"

cd "$TMPDIR/nvblox"
git fetch --tags --quiet
git checkout --quiet "$VERSION"
COMMIT=$(git rev-parse --short HEAD)
rm -rf .git

echo "[update] Syncing to $DEST..."
rm -rf "$DEST"
mkdir -p "$DEST"
# Exclude build artifacts.
rsync -a --exclude='build/' --exclude='.git/' "$TMPDIR/nvblox/" "$DEST/"

echo "[update] Applying patches..."
RATES_H="$DEST/nvblox/include/nvblox/utils/rates.h"
if [ -f "$RATES_H" ] && ! grep -q '^#include <array>' "$RATES_H"; then
    sed -i '/^#include <string>$/a #include <array>' "$RATES_H"
    echo "  patched rates.h (added #include <array>)"
fi

cat > "$DEST/NVBLOX_VERSION.txt" <<EOF
NVBlox vendored for MMP project

  Version: $VERSION
  Commit:  $COMMIT
  Source:  https://github.com/nvidia-isaac/nvblox
  License: Apache-2.0 (see LICENSE.md in this directory)
  Updated: $(date +%Y-%m-%d)

Local patches applied:
  - nvblox/include/nvblox/utils/rates.h: added #include <array> (GCC 13 compat)

To upgrade again, run update_nvblox.sh <version_tag>.
EOF

echo ""
echo "[update] Done."
echo "  Version: $VERSION ($COMMIT)"
echo "  Path:    $DEST"
echo "  Size:    $(du -sh "$DEST" | cut -f1)"
