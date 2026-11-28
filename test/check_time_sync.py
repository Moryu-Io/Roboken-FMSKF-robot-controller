#!/usr/bin/env python3
"""Exercise the production sync scheduler with an unreliable agent and tick wrap."""
from pathlib import Path
import os
import re
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / 'src/RobotManager/RM_task_main.cpp').read_text()
    routine = re.search(r'static void retry_time_sync\(\) \{.*?\n\}', source, re.S).group()
    program = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
using TickType_t = uint32_t;
using rmw_ret_t = int;
constexpr int RMW_RET_OK = 0;
static bool is_session_time_synchronized = false;
static TickType_t last_time_sync_attempt = 0, last_time_sync_success = 0, tick = 0;
static int calls = 0, result = 1;
TickType_t xTaskGetTickCount() { return tick; }
TickType_t pdMS_TO_TICKS(uint32_t ms) { return ms; }
int rmw_uros_sync_session(int timeout) { assert(timeout == 20); ++calls; return result; }
void report_rmw_failure(const char*, int) {}
#define DEBUG_PRINT_STR_RMT(...) ((void)0)
''' + routine + r'''
int main() {
  // Same reconnect initialization as production: immediately eligible to try.
  last_time_sync_attempt = tick - pdMS_TO_TICKS(1000);
  retry_time_sync(); assert(calls == 1 && !is_session_time_synchronized);
  tick = 999; retry_time_sync(); assert(calls == 1);
  tick = 1000; result = RMW_RET_OK; retry_time_sync();
  assert(calls == 2 && is_session_time_synchronized);
  tick = 10999; retry_time_sync(); assert(calls == 2);
  tick = 11000; result = 1; retry_time_sync();
  assert(calls == 3 && is_session_time_synchronized); // Keep publishing on refresh failure.
  tick = 11999; retry_time_sync(); assert(calls == 3);
  tick = 12000; result = RMW_RET_OK; retry_time_sync(); assert(calls == 4);
  tick = 21999; retry_time_sync(); assert(calls == 4);
  tick = 22000; retry_time_sync(); assert(calls == 5);
  // A reconnect must not wait for the previous session's successful sync.
  is_session_time_synchronized = false;
  last_time_sync_attempt = tick - pdMS_TO_TICKS(1000);
  retry_time_sync(); assert(calls == 6 && is_session_time_synchronized);
  last_time_sync_success = last_time_sync_attempt = UINT32_MAX - 5000;
  tick = 4998; retry_time_sync(); assert(calls == 6);
  tick = 4999; retry_time_sync(); assert(calls == 7);
  std::puts("PASS: initial retry, periodic sync, refresh failure, reconnect, tick wrap");
}
'''
    with tempfile.TemporaryDirectory(prefix='time-sync-test-') as temp:
        cpp, binary = Path(temp) / 'check.cpp', Path(temp) / 'check'
        cpp.write_text(program)
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++14', '-Wall', '-Wextra',
                        '-Werror', str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
