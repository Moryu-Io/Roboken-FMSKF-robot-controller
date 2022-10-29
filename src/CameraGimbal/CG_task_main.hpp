#ifndef CG_TASK_MAIN_HPP_
#define CG_TASK_MAIN_HPP_

#include "global_config.hpp"

namespace CGT {

enum MSG_ID {
  REQ_INIT          = 0x01,
  REQ_MOVE_PITCH    = 0x10,
  REQ_DEFAULT_PITCH = 0x11,
  MSG_UNKNOWN       = 0xFF,
};

// Message共通変数
struct MsgCommon {
  uint8_t MsgId;
  uint8_t Sender;
  uint8_t Recv0;
  uint8_t Recv1;
};

// ID:0x10 REQ_MOVE_PITCH
// Positioning
struct MSG_ReqMovePitch {
  MsgCommon cmn;
  float     fl_pitch_deg;
};

// Message共用体
union MSG_REQ {
  MsgCommon        common;
  MSG_ReqMovePitch move_pitch;
};

void prepare_task();
void main(void *params);

void send_req_msg(MSG_REQ *_msg);

float get_pitch_angle_deg();

}; // namespace CGT

#endif