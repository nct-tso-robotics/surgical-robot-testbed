#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p src
vcs import --recursive --skip-existing src < sources.repos
python3 tools/check_sources.py
