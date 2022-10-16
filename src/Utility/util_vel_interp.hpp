#ifndef UTIL_VEL_INTERP_HPP_
#define UTIL_VEL_INTERP_HPP_

#include "arm_math.h"
#include "global_config.hpp"

namespace UTIL {

class VelInterp {
public:
  VelInterp(){};
  virtual void  set_target_params(float v_t, float a_m, float jrk) { vel_now_ = v_t; };
  virtual float update() { return vel_now_; };
  virtual void  reset() { vel_now_ = acl_now_ = 0.0f; };

  float get_now_vel() { return vel_now_; };
  float get_now_accel() { return acl_now_; };

protected:
  float vel_now_;
  float acl_now_;

};

class VelInterpConstJerk : public VelInterp {
private:
  struct StatusBuf {
    /* 目標設定値 */
    float vel_tgt_;
    float acl_max_;
    float jerk_p_; // 加加速時Jerk
    float jerk_m_; // 減加速時Jerk
    float dt1_;
    float dt2_;
    float dt3_;
    float vel_ini_;
    float acl_ini_;
    float dt_;
  };

public:
  VelInterpConstJerk(float sample_time) : VelInterp(), ts_(sample_time) {
    u8_now_use_ = 0;
  };

  /**
   * @brief Set the target params object
   *
   * @param v_t : 目標速度(符号有り)
   * @param a_m : 最大加速度(符号無し)
   * @param jrk : 躍度(符号無し)
   */
  void set_target_params(float v_t, float a_m, float jrk) override {
    StatusBuf *w_s = &sts[u8_now_use_ ^ 1]; // 書き込むStatusBuffer

    /* 設定値をコピー */
    w_s->vel_tgt_ = v_t;
    w_s->acl_max_ = a_m;

    /* 現速度/加速度をlatch */
    w_s->vel_ini_ = vel_now_;
    w_s->acl_ini_ = acl_now_;

    /* 方向に応じて符号を変える */
    if((w_s->vel_tgt_ - w_s->vel_ini_) < 0) {
      w_s->acl_max_ = -a_m;
    }

    /* 最大加速:正の時、減加速時jerkは負値 */
    w_s->jerk_m_      = (w_s->acl_max_ >= 0) ? -jrk : jrk;
    float _jerk_m_inv = 1.0f / w_s->jerk_m_;

    /* (最大加速-現加速):正の時、加加速時jerkは正値 */
    w_s->jerk_p_      = (w_s->acl_max_ - w_s->acl_ini_ >= 0) ? jrk : -jrk;
    float _jerk_p_inv = 1.0f / w_s->jerk_p_;

    w_s->dt1_ = (w_s->acl_max_ - w_s->acl_ini_) * _jerk_p_inv;
    w_s->dt3_ = w_s->acl_max_ * (-_jerk_m_inv);
    w_s->dt2_ = 1.0f / w_s->acl_max_                  //
                * (w_s->vel_tgt_                      //
                   - w_s->vel_ini_                    //
                   - w_s->acl_ini_ * w_s->dt1_ * 0.5f //
                   - w_s->acl_max_ * (w_s->dt1_ + w_s->dt3_) * 0.5f);

    /* 等加速区間がない場合 */
    if(w_s->dt2_ < 0.0f) {
      float _sqrt_res = 0.0f;
      float _sqrt_in  = (w_s->acl_ini_ * _jerk_p_inv) * (w_s->acl_ini_ * _jerk_p_inv) * 0.5f //
                       + (w_s->vel_tgt_ - w_s->vel_ini_) * _jerk_p_inv;
      arm_sqrt_f32(_sqrt_in, &_sqrt_res);
      w_s->dt1_ = _sqrt_res - w_s->acl_ini_ * _jerk_p_inv;

      /* 最大加速区間がないので、MAX加速度を再計算 */
      w_s->acl_max_ = w_s->acl_ini_ + w_s->jerk_p_ * w_s->dt1_;

      w_s->dt2_ = 0.0f;
      w_s->dt3_ = w_s->acl_max_ * (-_jerk_m_inv);
    }

    /* FailSafe */
    w_s->dt1_ = (w_s->dt1_ < 0.0f) ? 0.0f : w_s->dt1_;
    // w_s->dt2_ = (w_s->dt2_ < 0.0f) ? 0.0f : w_s->dt2_;    // dt2_は↑が実質FailSafeになっている
    w_s->dt3_ = (w_s->dt3_ < 0.0f) ? 0.0f : w_s->dt3_;

    /* 最後にバッファ面切り替え */
    w_s->dt_    = 0.0f;
    u8_now_use_ = u8_now_use_ ^ 1;
  }

  float update() override {
    StatusBuf *r_s = &sts[u8_now_use_]; // 参照statusBuffer

    if(r_s->dt_ <= r_s->dt1_ + ts_) {
      /* 加加速区間 */
      acl_now_ = r_s->acl_ini_ + r_s->jerk_p_ * r_s->dt_;
      vel_now_ = r_s->vel_ini_ + (r_s->acl_ini_ + acl_now_) * r_s->dt_ * 0.5f;
      r_s->dt_ = r_s->dt_ + ts_;
    } else if(r_s->dt_ <= r_s->dt1_ + r_s->dt2_ + ts_) {
      /* 等加速区間 */
      acl_now_ = r_s->acl_max_;
      vel_now_ = vel_now_ + acl_now_ * ts_;
      r_s->dt_ = r_s->dt_ + ts_;
    } else if(r_s->dt_ <= r_s->dt1_ + r_s->dt2_ + r_s->dt3_ + ts_) {
      /* 減加速区間*/
      acl_now_ = r_s->acl_max_ + r_s->jerk_m_ * (r_s->dt_ - r_s->dt1_ - r_s->dt2_);
      vel_now_ = vel_now_ + acl_now_ * ts_;
      r_s->dt_ = r_s->dt_ + ts_;
    } else {
      /* 加減速終了(等速区間) */
      /* FailSafe的に現行のStatusBufferの目標速度に設定しておく */
      acl_now_ = 0.0f;
      vel_now_ = r_s->vel_tgt_;
    }

    return vel_now_;
  }

  void reset() override {
    vel_now_ = acl_now_ = 0.0f;
    StatusBuf stszero   = {};
    sts[0]              = stszero;
    sts[1]              = stszero;
  };

private:
  /* 設定値 */
  float ts_;

  /* 状態値 */
  StatusBuf sts[2];      // 設定値2面バッファ
  uint8_t   u8_now_use_; // 現在制御側で使っているページ
};

} // namespace UTIL

#endif