#ifndef AD_JOINT_BASE_HPP_
#define AD_JOINT_BASE_HPP_

#include "global_config.hpp"

namespace ADT {

class JointBase {
public:
  JointBase() : is_torque_on(false){};

  virtual void update() = 0;

  void set_torque_on(bool on) { is_torque_on = on; };
  void set_tgt_ang_deg(float tgt) { fl_raw_tgt_deg = tgt + fl_out_ofs_deg; };

  bool  get_connect_status() { return is_connected; };
  float get_now_deg() { return fl_raw_now_deg - fl_out_ofs_deg; }; // 現在の実角度[deg] (オフセット考慮)
  float get_now_cur() { return fl_out_now_cur; };

protected:
  // 設定値
  float fl_out_ofs_deg; // 原点オフセット角度[deg]

  // 指示値
  float fl_raw_tgt_deg; // 現在の目標角度[deg] (オフセット未考慮)

  // 状態値
  bool  is_connected;
  bool  is_torque_on;
  float fl_raw_now_deg; // 現在の実角度[deg] (オフセット未考慮)
  float fl_out_now_cur; // 現在の電流値[単位はサーボによる]
};

}; // namespace ADT

#endif