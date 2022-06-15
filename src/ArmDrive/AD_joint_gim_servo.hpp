#ifndef AD_JOINT_GIM_SERVO_HPP_
#define AD_JOINT_GIM_SERVO_HPP_

#include "AD_joint_base.hpp"
#include "global_config.hpp"

namespace ADT {

class JointGimServo : public JointBase {
public:
  // Teensy はビットフィールドがリトルエンディアン...
  struct GimMsgRx {
    uint8_t u8_host_id;
    uint8_t u8_pos_h;
    uint8_t u8_pos_l;
    uint8_t u8_vel_h;
    uint8_t u8_vel_l4_trq_h4;
    uint8_t u8_trq_l;
  };

  struct GimMsgTxParamsSet {
    uint8_t u8_pos_h;
    uint8_t u8_pos_l;
    uint8_t u8_vel_h;
    uint8_t u8_vel_l4_Kp_h4;
    uint8_t u8_Kp_l;
    uint8_t u8_Kd_h;
    uint8_t u8_Kd_l4_trq_h4;
    uint8_t u8_trq_l;
  };

  JointGimServo(ConstParams &_c) : JointBase(_c){};

  void update() override;

  /* パラメータ設定 */
  void set_gain(uint16_t kp, uint16_t kd){
      u16_Kp = kp;
      u16_Kd = kd;
  }

  /* CAN通信用 */
  bool get_cantx_data(uint8_t *_d) {
    if(is_updated_txdata) {
      memcpy(_d, txdata, 8);
      is_updated_txdata = false;
      return true;
    }
    return false;
  }
  void rx_callback(GimMsgRx *rxMsg, int16_t microsec_id);

private:
  bool is_torque_on_prev;
  bool is_updated_txdata;

  uint16_t u16_Kp;
  uint16_t u16_Kd;

  uint8_t txdata[8];
};

} // namespace ADT

#endif
