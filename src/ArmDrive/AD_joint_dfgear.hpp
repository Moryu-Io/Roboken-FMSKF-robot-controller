#ifndef AD_JOINT_DFGEAR_HPP_
#define AD_JOINT_DFGEAR_HPP_

#include "AD_joint_base.hpp"
#include "AD_joint_mybldc_servo.hpp"
#include "global_config.hpp"

namespace ADT {

/**
 * @brief 差動軸の左右への振り分けクラス
 * 
 */
class JointDfGearVirtual {
public:
  JointDfGearVirtual(JointMyBldcServo &_lm, JointMyBldcServo &_rm)
      : left_m_(_lm), right_m_(_rm){};
    
  void set_P_tgt_ang_deg(float tgt_P){
    fl_rawP_tgt_deg_ = tgt_P;
    left_m_.set_tgt_ang_deg(fl_rawP_tgt_deg_  + fl_rawR_tgt_deg_);
    right_m_.set_tgt_ang_deg(fl_rawP_tgt_deg_  - fl_rawR_tgt_deg_);
  };

  void set_R_tgt_ang_deg(float tgt_R){
    fl_rawR_tgt_deg_ = tgt_R;
    left_m_.set_tgt_ang_deg(fl_rawP_tgt_deg_  + fl_rawR_tgt_deg_);
    right_m_.set_tgt_ang_deg(fl_rawP_tgt_deg_  - fl_rawR_tgt_deg_);
  };

  JointMyBldcServo &left_m_;
  JointMyBldcServo &right_m_;

protected:
  float fl_rawP_tgt_deg_;
  float fl_rawR_tgt_deg_;
};

/**
 * @brief 差動軸PitchのIF用
 * 
 */
class JointDfGearPitch : public JointBase {
public:
  JointDfGearPitch(ConstParams &_c, JointDfGearVirtual &_df_pr)
      : JointBase(_c), df_parent_(_df_pr){};

  void update() override{};

  void set_torque_on(bool on) override {
    df_parent_.left_m_.set_torque_on(on);
    df_parent_.right_m_.set_torque_on(on);
  };

  void set_curlim_A(float lim) override {
    df_parent_.left_m_.set_curlim_A(lim);
    df_parent_.right_m_.set_curlim_A(lim);
  };

  void set_tgt_ang_deg(float tgt) {
    fl_raw_tgt_deg = tgt + fl_out_ofs_deg;
    df_parent_.set_P_tgt_ang_deg(fl_raw_tgt_deg * c_params.fl_gear_ratio);  // 渡すのはギア比考慮
  };

  float get_raw_deg() override { return (df_parent_.left_m_.get_now_deg() + df_parent_.right_m_.get_now_deg()) * 0.5f / c_params.fl_gear_ratio; };
  float get_now_deg() override { return get_raw_deg() - fl_out_ofs_deg; };

protected:
  JointDfGearVirtual &df_parent_;
};

/**
 * @brief 差動軸Roll用
 * @note Pitchから異なるもののみOverride
 * 
 */
class JointDfGearRoll : public JointDfGearPitch {
public:
  JointDfGearRoll(ConstParams &_c, JointDfGearVirtual &_df_pr)
      : JointDfGearPitch(_c, _df_pr){};

  void set_tgt_ang_deg(float tgt) {
    fl_raw_tgt_deg = tgt + fl_out_ofs_deg;
    df_parent_.set_R_tgt_ang_deg(fl_raw_tgt_deg * c_params.fl_gear_ratio);  // 渡すのはギア比考慮
  };

  float get_raw_deg() override { return (-df_parent_.left_m_.get_now_deg() + df_parent_.right_m_.get_now_deg()) * 0.5f / c_params.fl_gear_ratio; };

};

}; // namespace ADT

#endif