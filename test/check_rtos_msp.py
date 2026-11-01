#!/usr/bin/env python3
"""Check the built ARM code uses the Teensy linker stack top, never VTOR[0]."""
import argparse
from pathlib import Path
import re
import shutil
import subprocess


def tool(name):
    executable = "arm-none-eabi-" + name
    found = shutil.which(executable)
    if found:
        return found
    bundled = Path.home() / ".platformio/packages/toolchain-gccarmnoneeabi-teensy/bin" / executable
    if bundled.is_file():
        return str(bundled)
    raise RuntimeError(f"Cannot find {executable}")


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", nargs="?", type=Path,
                        default=root / ".pio/build/teensy41/firmware.elf")
    args = parser.parse_args()
    symbols = subprocess.check_output([tool("nm"), str(args.elf)], text=True)
    stack = re.search(r"^([0-9a-fA-F]+)\s+\w\s+_estack$", symbols, re.M)
    if not stack:
        raise RuntimeError("Missing linker stack top")
    stack_top = int(stack[1], 16)
    if not (0x20000000 < stack_top <= 0x20080000 and stack_top % 8 == 0):
        raise RuntimeError(f"Invalid DTCM stack top: 0x{stack_top:08x}")

    assembly = subprocess.check_output(
        [tool("objdump"), "-d", "--disassemble=prvPortStartFirstTask", str(args.elf)], text=True)
    # Verify the literal actually loaded into r0, rather than merely finding
    # an unrelated stack address somewhere in the executable.
    load = re.search(r"ldr\s+r0,\s*\[pc,[^\]]+\]\s*;\s*\(([0-9a-f]+)\s", assembly)
    if not load:
        raise RuntimeError("Missing initial MSP literal load")
    # Literal pools can be outside the function's symbol size.
    address = int(load[1], 16)
    pool = subprocess.check_output(
        [tool("objdump"), "-d", f"--start-address={address}",
         f"--stop-address={address + 4}", str(args.elf)], text=True)
    literal = re.search(r"^\s*" + load[1] + r":\s+([0-9a-f]{8})\s+\.word", pool, re.M)
    if not literal or int(literal[1], 16) != stack_top:
        raise RuntimeError("Initial MSP literal is not _estack (legacy VTOR lookup is unsafe on Teensy)")
    setup = assembly[load.start():]
    write = re.search(r"msr\s+MSP,\s*r0", setup)
    if not write or re.search(r"ldr\s+r0,\s*\[r0", setup[:write.end()]):
        raise RuntimeError("MSP setup dereferences an address instead of using the linker stack top")
    enable = re.search(r"cpsie\s+i", setup)
    if not enable or write.end() > enable.start():
        raise RuntimeError("Interrupts enabled before MSP setup")
    print(f"PASS: generated MSP setup loads _estack=0x{stack_top:08x} before interrupts; no VTOR lookup")


if __name__ == "__main__":
    main()
