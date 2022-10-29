#ifndef CG_ICS_SERVO_HPP_
#define CG_ICS_SERVO_HPP_

#include "IcsHardSerialClass.h"
#include "global_config.hpp"

namespace CGT {

class CGIcsServo {
public:
  CGIcsServo(){};

  void update();
  void init(IcsHardSerialClass *_p_ics_serial, uint8_t _id);

  void set_target_deg(float _tgt_deg){ fl_raw_tgt_deg = _tgt_deg; };
  void set_torque(bool _trq_on){ is_torque_on = _trq_on; };

  bool get_connect_status(){ return is_connected; };
  float get_now_angle_deg(){ return fl_raw_now_deg; };

private:
  IcsHardSerialClass *p_ics_serial;

  // 設定値
  uint8_t u8_id;

  // 指示値
  float fl_raw_tgt_deg; // 現在の目標角度[deg] (オフセット未考慮)

  // 状態値
  bool  is_connected;
  bool  is_torque_on;
  float fl_raw_now_deg; // 現在の実角度[deg] (オフセット未考慮)
  float fl_out_now_cur; // 現在の電流値[単位はサーボによる]
};

}; // namespace CGT

#endif