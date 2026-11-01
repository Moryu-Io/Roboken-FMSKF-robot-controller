#!/usr/bin/env python3
"""Compile the board's RTOS config with and without device CMSIS definitions."""
from pathlib import Path
import os
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    program = '''
#include "Teensy4FreeRTOSConfig.h"
static_assert(configPRIO_BITS == 4, "RT1062 implements four priority bits");
static_assert(configMAX_SYSCALL_INTERRUPT_PRIORITY == 0x20, "BASEPRI mask");
static_assert((configMAX_SYSCALL_INTERRUPT_PRIORITY & 0xf0) != 0,
              "BASEPRI must survive hardware masking of unimplemented bits");
static_assert(configKERNEL_INTERRUPT_PRIORITY == 0xf0, "Kernel priority");
'''
    with tempfile.TemporaryDirectory(prefix="teensy-priority-test-") as temporary:
        stub = Path(temporary)
        (stub / "Arduino.h").write_text("#include <stdint.h>\n")
        command = [os.environ.get("CXX", "g++"), "-std=c++14", "-x", "c++",
                   "-fsyntax-only", "-I" + str(stub), "-I" + str(root / "include")]
        for bits in (None, 4, 3, 8):
            flags = [] if bits is None else [f"-D__NVIC_PRIO_BITS={bits}"]
            result = subprocess.run(command + flags + ["-"], input=program,
                                    text=True, capture_output=True)
            expected = bits in (None, 4)
            if (result.returncode == 0) != expected:
                raise RuntimeError(f"Unexpected compile result for CMSIS bits={bits}:\n{result.stderr}")
    print("PASS: Teensyduino/CMSIS-4 yield BASEPRI=0x20; incompatible CMSIS widths rejected")


if __name__ == "__main__":
    main()
