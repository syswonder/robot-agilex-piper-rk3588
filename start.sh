#!/usr/bin/env bash
# Run the documented Piper boot command in the deployment container.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
exec bash "$ROOT/containers/piper/run.sh" -e \
  'cd /root/workspace && rbnx boot -f robonix_manifest.local.yaml --no-update-check --verbose'
