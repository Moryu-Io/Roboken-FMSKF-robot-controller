#ifndef AD_MODE_INITIALIZING_HPP_
#define AD_MODE_INITIALIZING_HPP_

#include "AD_mode_base.hpp"
#include "global_config.hpp"

namespace ADT {

class ADTModeInitialize : public ADTModeBase {
public:
  ADTModeInitialize(){};

  void doInit() override;
  void update() override;
  void end() override{};

  enum State {
    INIT,          // 初期
    TORQUE_ON,     // トルクON
    MOVE_MECH_END, // 端当て(差動軸除く)
    RESET_ANGLE,   // 角度情報初期化
    MOVE_INIT_POS, // 初期位置移動
    COMPLETED
  };

private:
  /* 状態ごとの処理関数 */
  void exec_init();
  void exec_torqueon();
  void exec_move_mechend();
  void exec_resetangle();
  void exec_move_initpos();

  /* SubRoutine */
  void ax_move_mechend(JointAxis ax);
  void ax_reset_angle(JointAxis ax);

  State nowState;

  uint16_t u16_wait_cnt_;
  bool is_move_end_comp[JointAxis::J_NUM];

  const uint16_t c_u16_toque_on_waitcnt = 100;
  const uint16_t c_u16_move_end_waitcnt = 500;
};

}; // namespace ADT

#endif