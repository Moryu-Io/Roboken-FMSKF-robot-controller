#ifndef AD_TASK_MAIN_HPP_
#define AD_TASK_MAIN_HPP_

namespace ADT {

enum MSG_ID {
  REQ_CHANGE_MODE = 0x01,
  REQ_MOVE_POS    = 0x10,
  MSG_UNKNOWN     = 0xFF,
};

// Message共通変数
struct MsgCommon {
  uint8_t MsgId;
  uint8_t Sender;
  uint8_t Recv0;
  uint8_t Recv1;
};

// ID:0x01 REQ_CHANGE_MODE
// MODE変更指示
struct MSG_ReqChangeMode {
  MsgCommon cmn;
  uint32_t  u32_mode_id;
  uint8_t  u8_forced;  // 強制変更フラグ
  uint8_t  u8_resv;
};

enum MODE_ID {
  OFF,
  INIT,
  POSITIONING,
  ERROR
};

// ID:0x10 REQ_MOVE_POS
// Positioning
struct MSG_ReqMovePos {
  MsgCommon cmn;
  uint32_t  u32_id;
  uint32_t  u32_dt_ms;
  float     fl_pos[5];
};

// Message共用体
union MSG_REQ {
  MsgCommon         common;
  MSG_ReqChangeMode change_mode;
  MSG_ReqMovePos    move_pos;
};

void prepare_task();
void main(void *params);

void send_req_msg(MSG_REQ *_msg);

}; // namespace ADT

#endif