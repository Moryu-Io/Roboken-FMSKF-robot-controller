#ifndef AD_JOINT_MYBLDC_SERVO_SERVO_HPP_
#define AD_JOINT_MYBLDC_SERVO_SERVO_HPP_

#include "AD_joint_base.hpp"
#include "global_config.hpp"

namespace ADT {

/* 受信Message定義 */
#define CMD_ID_RES_STATUS_SUMMARY (0x1000)
struct RES_STATUS_SUMMAY {
  uint8_t b1_motor_driver_fault : 1;
  uint8_t b1_motor_over_temp    : 1;
  uint8_t b1_motor_over_curr    : 1;
  uint8_t b1_motor_torque_on    : 1;
  uint8_t b1_mcu_fault          : 1;
  uint8_t b1_mcu_over_temp      : 1;
  uint8_t b1_dummy_0_67         : 2;
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
  JointMyBldcServo(uint16_t can_dev_id) : JointBase(), u16_canid(can_dev_id){};

  void update() override;

  /* パラメータ設定 */

  /* CAN通信用 */
  uint16_t get_id() { return u16_canid; }

  bool get_cantx_data(uint8_t *_d) {
    if(is_updated_txdata) {
      memcpy(_d, txdata, 8);
      is_updated_txdata = false;
      return true;
    }
    return false;
  }
  void rx_callback(uint32_t cmdid, RES_MESSAGE *rxMsg, int16_t microsec_id);

private:
  uint16_t u16_canid;
  bool     is_updated_txdata;
  uint8_t  txdata[8];

  void rx_summary_status(RES_STATUS_SUMMAY& sts);
};

} // namespace ADT

#endif
