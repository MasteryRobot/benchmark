#!/usr/bin/env bash
# Isolated benchmark environment. Does not modify the system or RSI runtime.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
venv="$root/.venvs/isaac6"
mkdir -p "$root/runtime_checks" "$root/.cache/pip"
export PIP_CONFIG_FILE=/dev/null
export PIP_INDEX_URL=https://pypi.org/simple
export PIP_EXTRA_INDEX_URL=
export PIP_DISABLE_PIP_VERSION_CHECK=1
export PIP_CACHE_DIR="$root/.cache/pip"
export PYTHONNOUSERSITE=1
python_bin=${ISAAC_SETUP_PYTHON:-python3.12}
if [[ ! -x "$venv/bin/python" ]]; then
  "$python_bin" -m venv "$venv"
fi
"$venv/bin/python" -m pip install --timeout 45 --retries 1 \
  'isaacsim[compatibility-check]==6.0.1.0' 'numpy==2.3.1' scipy pillow \
  'https://pypi.nvidia.com/isaacsim-extscache-kit/isaacsim_extscache_kit-6.0.1.0-cp312-none-manylinux_2_35_x86_64.whl#sha256=35b64cf35ce3d31875ef43025f243c8bd03e29cb28fe844b0cd06b76ba535714' \
  'https://pypi.nvidia.com/isaacsim-extscache-kit-sdk/isaacsim_extscache_kit_sdk-6.0.1.0-cp312-none-manylinux_2_35_x86_64.whl#sha256=a39c7a71f7a859e9db4ce6f81382f63a480d7e1938c6f3653702c9e6dd609c5e' \
  'isaacsim-extscache-physics==6.0.1.0'
"$venv/bin/python" -m pip freeze > "$root/runtime_checks/isaac6_packages.txt"
"$venv/bin/python" -m pip check
