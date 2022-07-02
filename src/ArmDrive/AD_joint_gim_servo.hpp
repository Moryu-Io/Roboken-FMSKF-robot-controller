#ifndef AD_JOINT_GIM_SERVO_HPP_
#define AD_JOINT_GIM_SERVO_HPP_

#include "../Utility/util_controller.hpp"
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

  struct GimPosCtrlGain {
    float fl_pg;
    float fl_ig;
    float fl_dg;
    float fl_ilim;
    float fl_lpffr;
  };

  JointGimServo(ConstParams &_c) : JointBase(_c), pos_ctrl_(1.0f / _c.fl_ctrl_time_s, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f){};

  void init() override;
  void update() override;

  /* パラメータ設定 */
  void set_gain(uint16_t kp, uint16_t kd) {
    u16_Kp = kp;
    u16_Kd = kd;
  }

  void set_myctrl_gain(GimPosCtrlGain &ctlgain, GimPosCtrlGain &offgain) {
    pos_ctrl_params_     = ctlgain;
    pos_ctrl_params_off_ = offgain;
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

  uint16_t      u16_Kp;
  uint16_t      u16_Kd;
  UTIL::FF_PI_D pos_ctrl_;

  GimPosCtrlGain pos_ctrl_params_;
  GimPosCtrlGain pos_ctrl_params_off_;

  uint8_t txdata[8];

  void set_myctrl_gain_params(GimPosCtrlGain &ctlgain) {
    pos_ctrl_.set_PIDgain(ctlgain.fl_pg, ctlgain.fl_ig, ctlgain.fl_dg);
    pos_ctrl_.set_I_limit(ctlgain.fl_ilim);
    pos_ctrl_.set_FF_gain(0.0f);
    pos_ctrl_.set_FF_limit(0.0f);
    pos_ctrl_.set_VelLpf_CutOff(ctlgain.fl_lpffr);
  }
};

} // namespace ADT

#endif
