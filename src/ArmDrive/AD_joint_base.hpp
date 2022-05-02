#ifndef AD_JOINT_BASE_HPP_
#define AD_JOINT_BASE_HPP_

#include "global_config.hpp"

namespace ADT {

class JointBase {
public:
  JointBase() : is_torque_on(false){};

  virtual void update() = 0;

  void set_torque_on(bool on) { is_torque_on = on; };
  void set_tgt_ang_deg(float tgt) { fl_out_tgt_deg = tgt; };

  float get_now_deg() { return fl_out_now_deg; };
  float get_now_cur() { return fl_out_now_cur; };

protected:
  // 設定値
  float fl_out_ofs_deg; // 原点オフセット角度[deg]

  // 指示値
  float fl_out_tgt_deg; // 現在の目標角度[deg] (オフセット考慮)

  // 状態値
  bool  is_torque_on;
  float fl_out_now_deg; // 現在の実角度[deg] (オフセット考慮)
  float fl_out_now_cur; // 現在の電流値[単位はサーボによる]
};

}; // namespace ADT

#endif