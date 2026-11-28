#!/usr/bin/env python3
"""Run production sequence/reset/mailbox code on a host; no board is accessed."""
from pathlib import Path
import re
import subprocess
import tempfile


def function(source, signature):
    start = source.index(signature + ' {')
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'


def without_includes(text):
    return re.sub(r'^#include[^\n]*\n', '', text, flags=re.M)


def main():
    root = Path(__file__).resolve().parents[1]
    task = (root / 'src/ArmDrive/AD_task_main.cpp').read_text()
    program = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>
#include <algorithm>
namespace JointAxis { constexpr int J_NUM=5, J0_YAW=0; }
namespace UTIL {
struct IIR1 { IIR1(float,float,float){} void reset(){} float update(float x){return x;} };
namespace mymath { float satf(float x,float hi,float lo){return std::min(hi,std::max(lo,x));} }
}
namespace ADT {
struct Joint { float target=0; float get_tgt_deg(){return target;}
  void set_tgt_ang_deg(float value){target=value;} };
struct ADTModeBase {
  bool is_comp=true;
  virtual void doInit()=0; virtual void update()=0; virtual void end()=0;
  bool isCompleted(){return is_comp;}
  static Joint *P_JOINT_[5]; static float FL_CYCLE_TIME_S;
};
Joint joints[5]; Joint *ADTModeBase::P_JOINT_[5]={joints,joints+1,joints+2,joints+3,joints+4};
float ADTModeBase::FL_CYCLE_TIME_S=.01f;
}
#define DEBUG_PRINT_ADT(...) ((void)0)
#define DEBUG_PRINT_STR_ADT(...) ((void)0)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
'''
    program += without_includes((root / 'src/ArmDrive/AD_task_main.hpp').read_text())
    program += without_includes((root / 'src/ArmDrive/AD_mode_positioning_seq.hpp').read_text())
    program += without_includes((root / 'src/ArmDrive/AD_mode_positioning_seq.cpp').read_text())
    program += r'''
namespace ADT {
using UBaseType_t=unsigned;
constexpr int pdPASS=1;
struct Queue { size_t width,capacity; std::deque<std::vector<uint8_t>> values; };
using QueueHandle_t=Queue*;
int xQueueSend(QueueHandle_t q,const void *p,int){
  if(q->values.size()==q->capacity)return 0;
  q->values.emplace_back(static_cast<const uint8_t*>(p),static_cast<const uint8_t*>(p)+q->width);
  return pdPASS;
}
int xQueueReceive(QueueHandle_t q,void *p,int){
  if(q->values.empty())return 0;
  memcpy(p,q->values.front().data(),q->width);q->values.pop_front();return pdPASS;
}
void xQueueReset(QueueHandle_t q){q->values.clear();}
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q){return q->values.size();}
Queue requests{sizeof(MSG_REQ),8,{}}, events{sizeof(ExecutionProgress),16,{}}, follow{20,1,{}};
QueueHandle_t p_MsgQueueReq=&requests, execution_events=&events, p_FollowLatest=&follow;
ADTModePositioningSeq m_posseq;
bool autonomous_ready_value=false, arm_reset_requested=false, arm_reset_completed=false;
bool follow_active=false, execution_progress_valid=false, emergency_stop_requested=false;
int8_t manual_mode_request=-1;
ExecutionProgress execution_progress{};
struct SerialStub { void println(const char*){} } Serial;
float get_height_err(){return 0;} uint32_t get_height_err_age_ms(){return 10000;}
bool is_z_correction_available(){return true;}
void report_execution_progress(uint32_t id,uint32_t elapsed_ms,uint8_t status){
  execution_progress={id,elapsed_ms,status};execution_progress_valid=true;
  if(status) xQueueSend(execution_events,&execution_progress,0);
}
'''
    for signature in ('void request_timeangle_reset()', 'bool timeangle_reset_pending()',
                      'uint32_t get_timeangle_reset_status()',
                      'static void process_timeangle_reset()',
                      'bool send_req_msg(const MSG_REQ *_msg)'):
        program += function(task, signature)
    program += r'''
void complete(uint32_t id){
  ADTModePositioningSeq::PosCmdSeq seq{};seq.u32_id=id;seq.u8_cmd_seq_len=1;
  seq.cmd_seq[0].u32_dt_ms=100;
  for(auto &v:seq.cmd_seq[0].fl_tgt_pos_deg)v=.5f;
  m_posseq.push_cmdseq(seq);
  for(int i=0;i<15;++i)m_posseq.update();
  assert(m_posseq.get_q_cmdseq_status(id)==1);
}
void test(){
  m_posseq.doInit();
  assert(get_timeangle_reset_status()==ARM_RESET_PENDING);
  complete(20); complete(38);
  assert(execution_progress.id==38 && execution_progress.status==1);
  ADTModePositioningSeq::PosCmdSeq seq{};seq.u32_id=22;seq.u8_cmd_seq_len=1;
  seq.cmd_seq[0].u32_dt_ms=1000;m_posseq.push_cmdseq(seq);m_posseq.update();
  seq.u32_id=23;m_posseq.push_cmdseq(seq);
  MSG_REQ traj{};traj.common.MsgId=REQ_MOVE_TIMEANGLE;traj.time_angle.u32_id=24;
  assert(send_req_msg(&traj));
  MSG_REQ ready{};ready.common.MsgId=REQ_CHANGE_MODE;ready.change_mode.u32_mode_id=INIT_POS_MOVE;
  MSG_REQ start=ready;start.change_mode.u32_mode_id=POSITIONING_SEQ;
  assert(send_req_msg(&ready));assert(send_req_msg(&start));
  const float held=joints[0].target;
  request_timeangle_reset();
  assert(timeangle_reset_pending() && get_timeangle_reset_status()==ARM_RESET_PENDING);
  assert(!send_req_msg(&traj)); // Closed before the arm task can run.
  process_timeangle_reset();
  assert(get_timeangle_reset_status()==ARM_RESET_DONE && !timeangle_reset_pending());
  for(auto id:{20u,38u,22u,23u,24u})assert(m_posseq.get_q_cmdseq_status(id)==99);
  assert(m_posseq.isCompleted() && joints[0].target==held);
  assert(!execution_progress_valid && events.values.empty());
  assert(requests.values.size()==2); // READY/START after STOP survived the purge.
  MSG_REQ out{};
  assert(xQueueReceive(&requests,&out,0) && out.change_mode.u32_mode_id==INIT_POS_MOVE);
  assert(xQueueReceive(&requests,&out,0) && out.change_mode.u32_mode_id==POSITIONING_SEQ);
  request_timeangle_reset();process_timeangle_reset(); // repeated READY, no motion
  assert(get_timeangle_reset_status()==ARM_RESET_DONE);
  traj.time_angle.u32_id=20;assert(send_req_msg(&traj));
  assert(get_timeangle_reset_status()==ARM_RESET_PENDING);
  xQueueReset(&requests);complete(20); // Same ID works after ROS-only restart.
  // STOP receipt removes older mailbox work, never newer READY/START.
  assert(send_req_msg(&start));
  MSG_REQ stop=ready;stop.change_mode.u32_mode_id=OFF;stop.change_mode.u8_forced=1;
  request_timeangle_reset();assert(send_req_msg(&stop));assert(requests.values.empty());
  assert(send_req_msg(&ready));assert(send_req_msg(&start));process_timeangle_reset();
  assert(requests.values.size()==2 && emergency_stop_requested);
}
} // namespace ADT
int main(){ADT::test();}
'''
    with tempfile.TemporaryDirectory(prefix='arm-reset-host-') as temp:
        cpp, exe = Path(temp) / 'test.cpp', Path(temp) / 'test'
        cpp.write_text(program)
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        str(cpp), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    print('PASS: active/queued cancellation, ID history and progress clearing, '
          'ID reuse, repeated reset and preservation of newer READY/START')


if __name__ == '__main__':
    main()
