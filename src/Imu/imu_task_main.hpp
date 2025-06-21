#ifndef IMU_TASK_MAIN_HPP_
#define IMU_TASK_MAIN_HPP_

#include "global_config.hpp"

namespace IMT {

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

// Message共用体
union MSG_REQ {
  MsgCommon      common;
  MSG_ReqMoveDir move_dir;
};

void prepare_task();
void main(void *params);


struct imu_data{
  bool is_error;
  float acc[3];
  float gyr[3];
  float mag[3];
  float ang[3];
  float qut[4];
};

void get_status_now_imu(imu_data &imu_d);
void send_req_msg(MSG_REQ *_msg);

}; // namespace IMT

#endif