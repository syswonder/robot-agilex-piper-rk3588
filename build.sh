#!/usr/bin/env bash
# Run the documented Piper build command in the deployment container.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
exec bash "$ROOT/containers/piper/run.sh" -e \
  'cd /root/workspace && rbnx build -f robonix_manifest.local.yaml'
