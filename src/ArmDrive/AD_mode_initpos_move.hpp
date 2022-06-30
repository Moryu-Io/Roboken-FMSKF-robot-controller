#ifndef AD_MODE_INITPOS_MOVE_HPP_
#define AD_MODE_INITPOS_MOVE_HPP_

#include "AD_mode_base.hpp"
#include "global_config.hpp"

namespace ADT {

class ADTModeInitPosMove : public ADTModeBase {
public:
  ADTModeInitPosMove(){};

  void doInit() override;
  void update() override;
  void end() override{};

  enum State {
    INIT,          // 初期
    TORQUE_ON,     // トルクON
    MOVE_INIT_POS, // 初期位置移動
    COMPLETED
  };

private:
  /* 状態ごとの処理関数 */
  void exec_init();
  void exec_torqueon();
  void exec_move_initpos();

  State nowState;

  uint16_t u16_wait_cnt_;
  float fl_move_vel_dir_[JointAxis::J_NUM];

  const uint16_t c_u16_toque_on_waitcnt = 100;
  const uint16_t c_u16_move_end_waitcnt = 300;
};

}; // namespace ADT

#endif