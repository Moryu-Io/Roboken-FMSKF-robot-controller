#!/usr/bin/env python3
"""Execute the firmware's mode transitions with host fake modes (no hardware)."""
from pathlib import Path
import subprocess
import tempfile


def function(source, signature):
    start = source.index(signature + " {")
    end = source.index("{", start) + 1
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / "src/ArmDrive/AD_task_main.cpp").read_text()
    # Verify that the production loop actually polls after updating the active mode.
    loop = source[source.index("while(1)"):source.index("static void process_message() {")]
    assert loop.index("m_nowProcess->update();") < loop.index("advance_mode();")
    assert loop.index("advance_mode();") < loop.index("autonomous_ready_value =")
    program = r'''
#include <cassert>
enum class MODE_ID { OFF, INIT, INIT_POS_MOVE, POSITIONING, POSITIONING_SEQ };
struct Mode {
  bool completed=false; int inits=0, ends=0;
  bool isCompleted(){return completed;}
  void init(){completed=false; ++inits;}
  void end(){++ends;} void cancel(){}
};
Mode m_off, m_init, m_init_posmove, m_posi, m_posseq;
Mode *m_nowProcess=&m_init_posmove, *m_nextProcess=&m_init_posmove;
bool manual_enabled=false, follow_active=false, follow_armed=false;
bool follow_permitted=false, follow_locked=false, autonomous_ready_value=false;
int manual_state_value=0, p_FollowLatest=0;
void xQueueReset(int){}
#define DEBUG_PRINT_STR_ADT(x) ((void)0)
static void advance_mode(bool force=false);
'''
    program += function(source, "static void set_next_mode(MODE_ID _id, bool force)")
    program += function(source, "static void advance_mode(bool force)")
    program += r'''
int main(){
  set_next_mode(MODE_ID::POSITIONING_SEQ, false);
  for(int i=0;i<100;++i) advance_mode();
  assert(m_nowProcess==&m_init_posmove && m_posseq.inits==0);
  m_init_posmove.completed=true;
  advance_mode();
  assert(m_nowProcess==&m_posseq && m_posseq.inits==1);
  // Polling a completed mode must not restart it.
  m_posseq.completed=true;
  for(int i=0;i<100;++i) advance_mode();
  assert(m_posseq.inits==1);
  // STOP cancels a queued START even while READY has not finished.
  set_next_mode(MODE_ID::INIT_POS_MOVE, false);
  set_next_mode(MODE_ID::POSITIONING_SEQ, false);
  set_next_mode(MODE_ID::OFF, true);
  m_init_posmove.completed=true; m_off.completed=true;
  for(int i=0;i<100;++i) advance_mode();
  assert(m_nowProcess==&m_off && m_nextProcess==&m_off && m_posseq.inits==1);
  // A subsequent READY/START follows the same single-command path.
  set_next_mode(MODE_ID::INIT_POS_MOVE, false);
  set_next_mode(MODE_ID::POSITIONING_SEQ, false);
  advance_mode(); assert(m_nowProcess==&m_init_posmove);
  m_init_posmove.completed=true; advance_mode();
  assert(m_nowProcess==&m_posseq && m_posseq.inits==2);
}
'''
    with tempfile.TemporaryDirectory(prefix="arm-start-") as temp:
        cpp, exe = Path(temp) / "test.cpp", Path(temp) / "test"
        cpp.write_text(program)
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", str(cpp), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    print("PASS: deferred START, no repeated init, STOP cancellation, single START restart")


if __name__ == "__main__":
    main()
