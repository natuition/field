#!/bin/sh

set -e

REPO="git@github.com:natuition/multiple-view-intelligence.git"
PACKAGE_NAME="multiple-view-intelligence"

echo "=== Get latest tag of multiple-view-intelligence repo ==="

LATEST_TAG=$(git ls-remote --tags --refs "$REPO" \
  | awk '{print $2}' \
  | sed 's|refs/tags/||' \
  | sort -V \
  | tail -n 1)

if [ -z "$LATEST_TAG" ]; then
  echo "ERROR: No tag found for $REPO"
  exit 1
fi

echo "Latest tag: $LATEST_TAG"

echo "=== Remove last installation of $PACKAGE_NAME ==="
sudo -E python3 -m pip uninstall -y "$PACKAGE_NAME" || true

echo "=== Installing $PACKAGE_NAME from GitHub tag $LATEST_TAG ==="
sudo -E python3 -m pip install \
  --no-deps \
  --ignore-requires-python \
  --no-build-isolation \
  "git+ssh://git@github.com/natuition/multiple-view-intelligence.git@${LATEST_TAG}#egg=${PACKAGE_NAME}"

echo "=== Installed package info ==="
sudo -E python3 -m pip show "$PACKAGE_NAME"
