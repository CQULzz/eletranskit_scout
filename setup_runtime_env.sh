#!/usr/bin/env bash
# Source this file after ROS setup to expose CUDA libraries bundled by the
# active Python/Jetson installation. No repository-external workspace is used.

if [[ -z "${KIT_CUPTI_LIB:-}" ]]; then
  KIT_CUPTI_LIB="$(python3 - <<'PY'
from pathlib import Path
import site

roots = []
try:
    roots.extend(site.getsitepackages())
except AttributeError:
    pass
roots.append(site.getusersitepackages())
for root in roots:
    candidate = Path(root) / "nvidia" / "cuda_cupti" / "lib"
    if candidate.is_dir():
        print(candidate)
        break
PY
)"
fi

KIT_CUDA_LIB=""
for cuda_root in /usr/local/cuda /usr/local/cuda-*; do
  if [[ -d "${cuda_root}/lib64" ]]; then
    KIT_CUDA_LIB="${cuda_root}/lib64"
    break
  fi
done

KIT_EXTRA_LIBRARY_PATH="${KIT_CUPTI_LIB}"
if [[ -n "${KIT_CUDA_LIB}" ]]; then
  KIT_EXTRA_LIBRARY_PATH="${KIT_EXTRA_LIBRARY_PATH:+${KIT_EXTRA_LIBRARY_PATH}:}${KIT_CUDA_LIB}"
fi
if [[ -n "${KIT_EXTRA_LIBRARY_PATH}" ]]; then
  export LD_LIBRARY_PATH="${KIT_EXTRA_LIBRARY_PATH}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

unset KIT_CUDA_LIB KIT_EXTRA_LIBRARY_PATH
