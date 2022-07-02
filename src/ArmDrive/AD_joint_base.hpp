#ifndef AD_JOINT_BASE_HPP_
#define AD_JOINT_BASE_HPP_

#include "global_config.hpp"

namespace ADT {

enum JointAxis {
  J0_YAW,
  J1_PITCH,
  J2_PITCH,
  J3_ROLL,
  J4_PITCH,
  J_NUM
};

class JointBase {
public:
  struct ConstParams {
    float fl_ctrl_time_s;      // 制御周期[s]
    float fl_gear_ratio;       // 減速比(減速比はCANとの界面で考慮する, モータ回転量xに対して出力1のxを入力)
    float fl_motor_dir;        // 回転方向
    float fl_curlim_default_A; // 通常駆動時電流制限[A]
    float fl_mechend_pos_deg;  // 端をこの角度として角度オフセットを定める[deg]
    float fl_vel_init_degps;   // 初期化基準端当て用速度[deg/s]
    float fl_curlim_init_A;    // 初期化基準端当て用電流制限[A]
    float fl_initpos_deg;      // 初期姿勢角度[deg]
  };

public:
  JointBase(ConstParams &_c) : c_params(_c), is_torque_on(false){};

  virtual void init(){};
  virtual void update() = 0;
  virtual void mech_reset_pos(){
    fl_out_ofs_deg = fl_raw_now_deg - c_params.fl_mechend_pos_deg;
  };

  virtual void set_torque_on(bool on) { is_torque_on = on; };
  void         set_force_current(float cur) { fl_force_cur_A = cur; };
  virtual void set_tgt_ang_deg(float tgt) { fl_raw_tgt_deg = tgt + fl_out_ofs_deg; };
  virtual void set_curlim_A(float lim) { fl_curlim_A = lim; };
  virtual void set_ofs_ang_deg(float ofs) { fl_out_ofs_deg = ofs; };

  bool          get_connect_status() { return is_connected; };
  virtual float get_tgt_deg() { return fl_raw_tgt_deg - fl_out_ofs_deg; }; // 現在の目標角度[deg] (オフセット考慮)
  virtual float get_now_deg() { return fl_raw_now_deg - fl_out_ofs_deg; }; // 現在の実角度[deg] (オフセット考慮)
  virtual float get_raw_deg() { return fl_raw_now_deg; };                  // 現在の実角度[deg] (オフセット未考慮)
  float         get_now_cur() { return fl_out_now_cur; };
  float         get_curlim_default_A() { return c_params.fl_curlim_default_A; };

  // 初期化用
  float get_mechend_pos_deg() { return c_params.fl_mechend_pos_deg; };
  float get_vel_for_init_degps() { return c_params.fl_vel_init_degps; };
  float get_cur_for_init_A() { return c_params.fl_curlim_init_A; };
  float get_initpos_deg() { return c_params.fl_initpos_deg; };

protected:
  // 設定値
  ConstParams &c_params;
  float        fl_out_ofs_deg; // 原点オフセット角度[deg]

  // 指示値
  float fl_raw_tgt_deg; // 現在の目標角度[deg] (オフセット未考慮)
  float fl_curlim_A;    // 現在の電流制限値[A]
  float fl_force_cur_A; // 現在の強制電流値[A]

  // 状態値
  bool  is_connected;
  bool  is_torque_on;
  float fl_raw_now_deg; // 現在の実角度[deg] (オフセット未考慮)
  float fl_out_now_cur; // 現在の電流値[単位はサーボによる]
};

}; // namespace ADT

#endif