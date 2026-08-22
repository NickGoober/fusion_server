#!/usr/bin/env bash
# Build libfusion.so for the Python fusion server (Oracle Ubuntu / Linux).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}" && pwd)"

FUSION_DIR="${REPO_ROOT}/components/Fusion"
CF_DIR="${FUSION_DIR}/cf"
OUT_DIR="${SCRIPT_DIR}/native"
OUT_SO="${OUT_DIR}/libfusion.so"

mkdir -p "${OUT_DIR}"

SOURCES=(
  "${FUSION_DIR}/fusion.c"
  "${CF_DIR}/collar_gravity.c"
  "${CF_DIR}/kalman_core.c"
  "${CF_DIR}/kalman_supervisor.c"
  "${CF_DIR}/arm_math_shim.c"
  "${CF_DIR}/mm_flow.c"
  "${CF_DIR}/mm_tof.c"
)

INCLUDES=(
  -I"${REPO_ROOT}"
  -I"${REPO_ROOT}/main"
  -I"${FUSION_DIR}/include"
  -I"${CF_DIR}"
)

CFLAGS=(
  -std=c11 -O2 -fPIC -shared
  -Wall -Wextra
  -Wno-unused-parameter -Wno-unused-variable
  -Wno-pointer-to-int-cast -Wno-strict-aliasing -Wno-absolute-value
)

echo "[build_lib] Compiling ${OUT_SO} ..."
gcc "${CFLAGS[@]}" -o "${OUT_SO}" "${SOURCES[@]}" "${INCLUDES[@]}" -lm
echo "[build_lib] Done: ${OUT_SO}"
