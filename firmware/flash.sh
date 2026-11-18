#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build/win-debug}"
ELF="${ELF:-$BUILD_DIR/STM32f407_wuliu.elf}"
OPENOCD_EXE="${OPENOCD_EXE:-D:\\BaiduNetdiskDownload\\OpenOCD-20260302-0.12.0\\bin\\openocd.exe}"
OPENOCD_SCRIPTS="${OPENOCD_SCRIPTS:-D:\\BaiduNetdiskDownload\\OpenOCD-20260302-0.12.0\\share\\openocd\\scripts}"

if [ ! -f "$ELF" ]; then
  "$ROOT/build.sh"
fi

if command -v wslpath >/dev/null 2>&1; then
  ELF_WIN="$(wslpath -w "$ELF")"
elif command -v cygpath >/dev/null 2>&1; then
  ELF_WIN="$(cygpath -m "$ELF")"
else
  echo "ERROR: need wslpath or cygpath" >&2; exit 1
fi

powershell.exe -NoProfile -Command "& '$OPENOCD_EXE' -s '$OPENOCD_SCRIPTS' -f interface/cmsis-dap.cfg -c 'transport select swd' -c 'adapter speed 4000' -c 'cmsis_dap_backend hid' -c 'gdb_port disabled' -c 'tcl_port disabled' -c 'telnet_port disabled' -f target/stm32f4x.cfg -c 'program {$ELF_WIN} verify reset exit'"

echo "flashed: $ELF"
