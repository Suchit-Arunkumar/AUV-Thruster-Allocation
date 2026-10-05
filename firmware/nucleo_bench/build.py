"""Build the NUCLEO-F446RE allocation bench (no make needed).

    python firmware/nucleo_bench/build.py [--gcc PATH/TO/arm-none-eabi-gcc]

Writes firmware/nucleo_bench/build/alloc_bench.elf and prints its size.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = HERE / "build"

SOURCES = [
    HERE / "main.c",
    HERE / "board.c",
    HERE / "system_stm32f4xx.c",
    ROOT / "src" / "alloc.c",
    ROOT / "src" / "alloc_v2.c",
    HERE / "startup_stm32f446retx.s",
]

CPU = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
CFLAGS = CPU + [
    "-O2", "-std=c99", "-ffp-contract=off",
    "-ffunction-sections", "-fdata-sections",
    "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
    "-DSTM32F446xx",
    f"-I{HERE}", f"-I{HERE / 'cmsis'}", f"-I{ROOT / 'include'}",
]
LDFLAGS = CPU + [
    f"-T{HERE / 'STM32F446RETX_FLASH.ld'}",
    "-nostartfiles", "--specs=nano.specs", "--specs=nosys.specs",
    "-Wl,--gc-sections", f"-Wl,-Map={OUT / 'alloc_bench.map'}",
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gcc", default=shutil.which("arm-none-eabi-gcc") or "arm-none-eabi-gcc")
    a = ap.parse_args()

    OUT.mkdir(exist_ok=True)
    elf = OUT / "alloc_bench.elf"
    cmd = [a.gcc, *CFLAGS, *map(str, SOURCES), *LDFLAGS, "-o", str(elf)]
    print(subprocess.run([a.gcc, "--version"], capture_output=True, text=True).stdout.splitlines()[0])
    r = subprocess.run(cmd)
    if r.returncode:
        return r.returncode
    size = Path(a.gcc).with_name(Path(a.gcc).name.replace("gcc", "size"))
    subprocess.run([str(size) if size.exists() else "arm-none-eabi-size", str(elf)])
    print(f"built {elf.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
