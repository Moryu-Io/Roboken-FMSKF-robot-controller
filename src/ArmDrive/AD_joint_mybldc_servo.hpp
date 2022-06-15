#ifndef AD_JOINT_MYBLDC_SERVO_SERVO_HPP_
#define AD_JOINT_MYBLDC_SERVO_SERVO_HPP_

#include "AD_joint_base.hpp"
#include "global_config.hpp"

namespace ADT {
/* 送信Message定義 */
#define CMD_ID_REQ_TORQUE_ON (0x0001)
struct REQ_TORQUE_ON {
  uint8_t u8_doInitialize;
  uint8_t u8_dummy[7];
};

#define CMD_ID_REQ_TORQUE_OFF (0x0002)
struct REQ_TORQUE_OFF {
  uint8_t u8_doTerminate;
  uint8_t u8_dummy[7];
};

#define CMD_ID_REQ_MOVE_ANGLE (0x0010)
struct REQ_MOVE_ANGLE {
  int32_t  s32_tgt_ang_deg_Q16;
  uint16_t u16_movetime_ms;
  uint16_t u16_currlim_A_Q8;
};

#define CMD_ID_REQ_ANGLE_INIT (0x0011)
struct REQ_ANGLE_INIT {
  uint8_t u8_set_angle_flag; // 0:現在位置を0とする, 1:指定した角度で初期化する
  uint8_t u8_dummy[3];
  int32_t s32_init_ang_deg_Q16;
};

union REQ_MESSAGE {
  uint8_t        u8_data[8];
  REQ_TORQUE_ON  reqTrqOn;
  REQ_TORQUE_OFF reqTrqOff;
  REQ_MOVE_ANGLE reqMvAng;
  REQ_ANGLE_INIT reqAngIni;
};

/* 受信Message定義 */
#define CMD_ID_RES_STATUS_SUMMARY (0x1000)
struct RES_STATUS_SUMMAY {
  // ビットフィールドもリトルエンディアン
  uint8_t b1_dummy_0_67         : 2;
  uint8_t b1_mcu_over_temp      : 1;
  uint8_t b1_mcu_fault          : 1;
  uint8_t b1_motor_torque_on    : 1;
  uint8_t b1_motor_over_curr    : 1;
  uint8_t b1_motor_over_temp    : 1;
  uint8_t b1_motor_driver_fault : 1;
  uint8_t u8_now_mode;
  int16_t s16_out_ang_deg_Q4;
  int8_t  s8_motor_curr_A_Q4;
  int8_t  s8_motor_vol_V_Q3;
  uint8_t u8_vm_V_Q3;
  int8_t  s8_motor_tempr_deg;
};

#define CMD_ID_RES_STATUS_MOVE_ANGLE (0x1001)
struct RES_STATUS_MOVE_ANGLE {
  int16_t  s16_out_ang_deg_Q4;
  int16_t  s16_tgt_ang_deg_Q4;
  uint16_t u16_movetime_ms;
  uint8_t  u8_dummy[2];
};

union RES_MESSAGE {
  uint8_t               u8_data[8];
  RES_STATUS_SUMMAY     resSummary;
  RES_STATUS_MOVE_ANGLE resStsMvAng;
};

class JointMyBldcServo : public JointBase {
public:
  JointMyBldcServo(ConstParams &_c, uint16_t can_dev_id) : JointBase(_c), u16_canid(can_dev_id){};

  void update() override;

  /* パラメータ設定 */

  /* CAN通信用 */
  uint16_t get_id() { return u16_canid; }

  bool get_cantx_data(uint8_t *_d, uint32_t &_id) {
    if(is_updated_txdata) {
      _id = (u16_canid << 18) | u32_txcmdid;
      memcpy(_d, txmsg.u8_data, 8);
      is_updated_txdata = false;
      return true;
    }
    return false;
  }
  void rx_callback(uint32_t cmdid, RES_MESSAGE *rxMsg, int16_t microsec_id);

private:
  bool        is_torque_on_prev;
  uint16_t    u16_canid;
  bool        is_updated_txdata;
  uint32_t    u32_txcmdid;
  REQ_MESSAGE txmsg;

  void rx_summary_status(RES_STATUS_SUMMAY &sts);
};

} // namespace ADT

#endif
