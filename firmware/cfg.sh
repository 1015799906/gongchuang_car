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
ROOT_WIN="$(to_win "$ROOT")"
BUILD_DIR_WIN="$(to_win "$BUILD_DIR")"
TOOLCHAIN_FILE_WIN="$(to_win "$ROOT/cmake/gcc-arm-none-eabi.cmake")"
CMAKE_EXE_WIN="${CMAKE_EXE_WIN:-C:\\Users\\10157\\AppData\\Local\\stm32cube\\bundles\\cmake\\4.2.3+st.1\\bin\\cmake.exe}"
GCC_BIN_WIN="${GCC_BIN_WIN:-C:\\Users\\10157\\AppData\\Local\\stm32cube\\bundles\\gnu-tools-for-stm32\\14.3.1+st.2\\bin}"
NINJA_EXE_WIN="${NINJA_EXE_WIN:-C:\\Users\\10157\\AppData\\Local\\stm32cube\\bundles\\ninja\\1.13.2+st.1\\bin\\ninja.exe}"
GCC_EXE_WIN="${GCC_EXE_WIN:-$GCC_BIN_WIN\\arm-none-eabi-gcc.exe}"
GXX_EXE_WIN="${GXX_EXE_WIN:-$GCC_BIN_WIN\\arm-none-eabi-g++.exe}"

mkdir -p "$BUILD_DIR"
rm -f "$BUILD_DIR/CMakeCache.txt"

powershell.exe -NoProfile -Command "\
  \$env:PATH = '$GCC_BIN_WIN;' + \$env:PATH; \
  & '$CMAKE_EXE_WIN' \
    -S '$ROOT_WIN' \
    -B '$BUILD_DIR_WIN' \
    -G Ninja \
    -DCMAKE_MAKE_PROGRAM='$NINJA_EXE_WIN' \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_TOOLCHAIN_FILE='$TOOLCHAIN_FILE_WIN' \
    -DCMAKE_C_COMPILER='$GCC_EXE_WIN' \
    -DCMAKE_CXX_COMPILER='$GXX_EXE_WIN' \
    -DCMAKE_ASM_COMPILER='$GCC_EXE_WIN'"

echo "configured: $BUILD_DIR"
