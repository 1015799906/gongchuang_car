#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build/win-debug}"
if command -v wslpath >/dev/null 2>&1; then
  to_win() { wslpath -w "$1"; }
elif command -v cygpath >/dev/null 2>&1; then
  to_win() { cygpath -m "$1"; }
else
  echo "ERROR: need wslpath (WSL) or cygpath (Git Bash)" >&2; exit 1
fi
BUILD_DIR_WIN="$(to_win "$BUILD_DIR")"
CMAKE_EXE_WIN="${CMAKE_EXE_WIN:-C:\\Users\\10157\\AppData\\Local\\stm32cube\\bundles\\cmake\\4.2.3+st.1\\bin\\cmake.exe}"
GCC_BIN_WIN="${GCC_BIN_WIN:-C:\\Users\\10157\\AppData\\Local\\stm32cube\\bundles\\gnu-tools-for-stm32\\14.3.1+st.2\\bin}"

if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
  "$ROOT/cfg.sh"
fi

powershell.exe -NoProfile -Command "\
  \$env:PATH = '$GCC_BIN_WIN;' + \$env:PATH; \
  & '$CMAKE_EXE_WIN' --build '$BUILD_DIR_WIN' --target STM32f407_wuliu --"

echo "built: $BUILD_DIR/STM32f407_wuliu.elf"
