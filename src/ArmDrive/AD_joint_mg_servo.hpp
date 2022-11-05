#ifndef AD_JOINT_MG_SERVO_HPP_
#define AD_JOINT_MG_SERVO_HPP_

#include "../Utility/util_controller.hpp"
#include "../Utility/util_mymath.hpp"
#include "AD_joint_base.hpp"
#include "global_config.hpp"

namespace ADT {

class JointMgServo : public JointBase {
public:
  // Teensy はビットフィールドがリトルエンディアン...
  union MgMsgRx {
    uint8_t u8_d[8];
    struct rxcmd_summary {
      uint8_t u8_cmd;
      uint8_t u8_tempr;
      int16_t s16_iq;
      int16_t s16_vel;
      int16_t s16_enc;
      uint8_t u8_dummy;
    } summary;
    struct rxcmd_mlt_ang {
      uint8_t u8_cmd;
      uint8_t u8_ang[7];
    } mlt_ang;
  };

  // TxMessage
  union MgMsgTx {
    uint8_t u8_d[8];
    struct txcmd_generic {
      uint8_t u8_cmd;
      uint8_t u8_dummy[7];
    } cmd_generic;
    struct txcmd_writeram_pid {
      uint8_t u8_cmd;
      uint8_t u8_dummy;
      uint8_t u8_ang_kp;
      uint8_t u8_ang_ki;
      uint8_t u8_vel_kp;
      uint8_t u8_vel_ki;
      uint8_t u8_iq_kp;
      uint8_t u8_iq_ki;
    } w_ram_pid;
    struct txcmd_iqctrl {
      uint8_t u8_cmd;
      uint8_t u8_dummy0[3];
      int16_t s16_iq;
      uint8_t u8_dummy1[2];
    } iqctrl;
    struct txcmd_posctrl2 {
      uint8_t  u8_cmd;
      uint8_t  u8_dummy;
      uint16_t u16_vel_lim;
      int32_t  s32_ang;
    } posctrl2;
  };

  struct GimPosCtrlGain {
    float fl_pg;
    float fl_ig;
    float fl_dg;
    float fl_ilim;
    float fl_lpffr;
  };

  JointMgServo(ConstParams &_c) : JointBase(_c), pos_ctrl_(1.0f / _c.fl_ctrl_time_s, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f){};

  void init() override;
  void update() override;

  /* CAN通信用(Posコマンド) */
  bool get_cantx1_data(uint8_t *_d) {
    if(is_updated_txdata1) {
      memcpy(_d, tx1data, 8);
      is_updated_txdata1 = false;
      return true;
    }
    return false;
  }

  /* CAN通信用(Posコマンド) */
  bool get_cantx2_data(uint8_t *_d) {
    if(is_updated_txdata2) {
      memcpy(_d, tx2data, 8);
      is_updated_txdata2 = false;
      return true;
    }
    return false;
  }

  void rx_callback(MgMsgRx *rxMsg, int16_t microsec_id);

private:
  bool is_torque_on_prev;
  bool is_updated_txdata1;
  bool is_updated_txdata2;

  float fl_pre_raw_tgt_deg; // 前回の目標角度[deg] (オフセット未考慮)

  uint8_t tx1data[8];
  uint8_t tx2data[8]; // 基本ReadPos

  UTIL::PI_D pos_ctrl_;

  void subproc_shutdown();
  void subproc_torquectrl();
  void subproc_posctrl();

  void set_reqmsg_multipos();

  void set_myctrl_gain_params(GimPosCtrlGain &ctlgain) {
    pos_ctrl_.set_PIDgain(ctlgain.fl_pg, ctlgain.fl_ig, ctlgain.fl_dg);
    pos_ctrl_.set_I_limit(ctlgain.fl_ilim);
    pos_ctrl_.set_VelLpf_CutOff(ctlgain.fl_lpffr);
  }

  const double C_A = 0.0000057204;
  const double C_B = -0.0000485371;
  double       conv_raw_to_current(double raw) {
          if(raw >= 0) {
            return C_A * raw * raw + C_B * raw;
    } else {
            return -(C_A * raw * raw - C_B * raw);
    }
  }

  double conv_current_to_raw(double cur_a) {
    if(cur_a >= 0) {
      return (-C_B + UTIL::mymath::sqrtf(C_B * C_B + 4.0 * C_A * cur_a)) / (2.0 * C_A);
    } else {
      return (C_B - UTIL::mymath::sqrtf(C_B * C_B - 4.0 * C_A * cur_a)) / (2.0 * C_A);
    }
  }
};

} // namespace ADT

#endif
