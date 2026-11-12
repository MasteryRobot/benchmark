#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
packages="$root/.venvs/physx/lib/python3.12/site-packages"
export LD_LIBRARY_PATH="$root/.runtime-libs/usr/lib/x86_64-linux-gnu:$packages/ovstage/bin:$packages/ovstage/bin/plugins${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$root${PYTHONPATH:+:$PYTHONPATH}"
exec "$root/.venvs/physx/bin/python" -B "$@"
