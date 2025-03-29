#ifndef VD_TASK_MAIN_HPP_
#define VD_TASK_MAIN_HPP_

#include "global_config.hpp"

namespace VDT {

enum MSG_ID {
  REQ_MOVE_DIR      = 0x01,
  REQ_MOVE_CONT_DIR = 0x02,
  MSG_UNKNOWN       = 0xFF,
};

// Message共通変数
struct MsgCommon {
  uint8_t MsgId;
  uint8_t Sender;
  uint8_t Recv0;
  uint8_t Recv1;
};

// ID:0x01 REQ_MOVE_DIR
// 方向指示移動
struct MSG_ReqMoveDir {
  MsgCommon cmn;
  uint32_t  u32_cmd;
  uint32_t  u32_time_ms;
  uint32_t  u32_speed;
};

// ID:0x02 REQ_MOVE_CONT_DIR
// 方向指示移動
struct MSG_ReqMoveContDir {
  MsgCommon cmn;
  float     fl_vel_x_mmps;
  float     fl_vel_y_mmps;
  float     fl_vel_th_radps;
  uint32_t  u32_time_ms;
};

enum REQ_MOVE_DIR_CMD {
  MOVE_STOP        = 0x00,
  GO_FORWARD       = 0x01,
  GO_BACK          = 0x02,
  GO_RIGHT         = 0x03,
  GO_LEFT          = 0x04,
  GO_RIGHT_FORWARD = 0x05,
  GO_LEFT_FORWARD  = 0x06,
  GO_RIGHT_BACK    = 0x07,
  GO_LEFT_BACK     = 0x08,
  ROT_RIGHT        = 0x09,
  ROT_LEFT         = 0x0A,
};

// Message共用体
union MSG_REQ {
  MsgCommon      common;
  MSG_ReqMoveDir move_dir;
  MSG_ReqMoveContDir move_cont_dir;
};

void prepare_task();
void main(void *params);

void get_status_now_vehicle_vel(float &_vx, float &_vy, float &_vr);
void send_req_msg(MSG_REQ *_msg);

}; // namespace VDT

#endif