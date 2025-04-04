#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "util_iir.hpp"

namespace UTIL {

class controller {
public:
  controller(float _c_freq) : freq_(_c_freq), dt_(1.0f / _c_freq){};

  virtual float update(float _nowval) = 0;
  virtual void  reset()               = 0;

  void  set_target(float _tgtval) { now_tgt_ = _tgtval; };
  float get_target() { return now_tgt_; };
  float get_now_val() { return now_val_; };

protected:
  float freq_ = 1.0f;
  float dt_   = 1.0f;

  float now_val_  = 0.0f;
  float prev_val_ = 0.0f;
  float now_tgt_  = 0.0f;

  float now_error_  = 0.0f;
  float prev_error_ = 0.0f;

  float now_ctrl_ = 0.0f;
};

class PID : public controller {
public:
  PID(float _c_freq, float _p_gain, float _i_gain, float _d_gain,
      float _i_limit) : controller(_c_freq), Pgain_(_p_gain), Igain_(_i_gain), Dgain_(_d_gain),
                        I_limit_(_i_limit){};

  float update(float _nowval) override {
    now_val_   = _nowval;
    now_error_ = now_tgt_ - now_val_;

    // 積分計算
    Integ_ += Igain_ * dt_ * now_error_;
    Integ_ = (Integ_ >= I_limit_) ? I_limit_ : ((Integ_ <= -I_limit_) ? -I_limit_ : Integ_);

    // 制御量計算
    now_ctrl_ = Pgain_ * now_error_ + Integ_;

    prev_val_   = now_val_;
    prev_error_ = now_error_;

    return now_ctrl_;
  };

  void reset() override {
    Integ_ = 0.0f;

    now_val_    = 0.0f;
    prev_val_   = 0.0f;
    now_tgt_    = 0.0f;
    now_error_  = 0.0f;
    prev_error_ = 0.0f;

    now_ctrl_ = 0.0f;
  };

  void set_PIDgain(float _pg, float _ig, float _dg) {
    Pgain_ = _pg;
    Igain_ = _ig;
    Dgain_ = _dg;
  }
  void set_I_limit(float _i_lim) { I_limit_ = _i_lim; };

private:
  float Pgain_ = 0.0f;
  float Igain_ = 0.0f;
  float Dgain_ = 0.0f;

  float Integ_   = 0.0f;
  float I_limit_ = 0.0f;
};

/**
 * @brief 微分先行型PID
 *
 */
class PI_D : public controller {
public:
  PI_D(float _c_freq, float _p_gain, float _i_gain, float _d_gain, float _i_limit, float _lpf_freq)
      : controller(_c_freq), Pgain_(_p_gain), Igain_(_i_gain), Dgain_(_d_gain), I_limit_(_i_limit),
        velLpf_((2.0f * _c_freq - _lpf_freq) / (2.0f * _c_freq + _lpf_freq), _lpf_freq / (2.0f * _c_freq + _lpf_freq), _lpf_freq / (2.0f * _c_freq + _lpf_freq)){};

  float update(float _nowval) override {
    now_val_   = _nowval;
    now_error_ = now_tgt_ - now_val_;
    velLpf_.update((now_val_ - prev_val_) * freq_);

    // 積分計算
    Integ_ += Igain_ * dt_ * now_error_;
    Integ_ = (Integ_ >= I_limit_) ? I_limit_ : ((Integ_ <= -I_limit_) ? -I_limit_ : Integ_);

    // 制御量計算
    now_ctrl_ = Pgain_ * now_error_ + Integ_ - Dgain_ * velLpf_.get_output();

    prev_val_   = now_val_;
    prev_error_ = now_error_;

    return now_ctrl_;
  };

  void reset() override {
    Integ_ = 0.0f;

    now_val_    = 0.0f;
    prev_val_   = 0.0f;
    now_tgt_    = 0.0f;
    now_error_  = 0.0f;
    prev_error_ = 0.0f;

    now_ctrl_ = 0.0f;

    velLpf_.reset();
  };

  void set_PIDgain(float _pg, float _ig, float _dg) {
    Pgain_ = _pg;
    Igain_ = _ig;
    Dgain_ = _dg;
  }
  void set_I_limit(float _i_lim) { I_limit_ = _i_lim; };

  void set_VelLpf_CutOff(float _lpf_frq) {
    velLpf_.set_coefs((2.0f * freq_ - _lpf_frq) / (2.0f * freq_ + _lpf_frq),
                      _lpf_frq / (2.0f * freq_ + _lpf_frq),
                      _lpf_frq / (2.0f * freq_ + _lpf_frq));
  };

private:
  float Pgain_ = 0.0f;
  float Igain_ = 0.0f;
  float Dgain_ = 0.0f;

  float Integ_   = 0.0f;
  float I_limit_ = 0.0f;

  IIR1 velLpf_;
};

/**
 * @brief 微分先行型PID+FF
 *
 */
class FF_PI_D : public PI_D {
public:
  FF_PI_D(float _c_freq, float _ff_gain, float _p_gain, float _i_gain, float _d_gain, float _i_limit, float _lpf_freq)
      : PI_D(_c_freq, _p_gain, _i_gain, _d_gain, _i_limit, _lpf_freq), FFgain_(_ff_gain){};

  float update(float _nowval) override {
    PI_D::update(_nowval);
    float _ff_val = now_tgt_ * FFgain_;
    _ff_val = (_ff_val >= FFlim_) ? FFlim_ : ((_ff_val <= -FFlim_) ? -FFlim_ : _ff_val);
    now_ctrl_ = now_ctrl_ + _ff_val;
    return now_ctrl_;
  };

  void set_FF_gain(float _ff) { FFgain_ = _ff; };
  void set_FF_limit(float _i_lim) { FFlim_ = _i_lim; };

private:
  float FFgain_ = 0.0f;
  float FFlim_ = 1.0f;
};

}; // namespace UTIL

#endif