#include "AD_mode_initialize.hpp"

namespace ADT {

void ADTModeInitialize::doInit() {
  /* 変数初期化 */
  nowState      = INIT;
  u16_wait_cnt_ = 0;
  for(int i = 0; i < JointAxis::J_NUM; i++) {
    is_move_end_comp[i] = false;
  }
}

void ADTModeInitialize::update() {
  switch(nowState) {
  case INIT:
    exec_init();
    break;
  case TORQUE_ON:
    exec_torqueon();
    break;
  case MOVE_MECH_END:
    exec_move_mechend();
    break;
  case RESET_ANGLE:
    exec_resetangle();
    break;
  case MOVE_INIT_POS:
    exec_move_initpos();
    break;
  case COMPLETED:
    is_comp = true;
    break;
  default:
    break;
  };
}

/**
 * @brief 初期化実行(通信初期化など)
 *
 */
void ADTModeInitialize::exec_init() {
  for(int i = 0; i < JointAxis::J_NUM; i++) {
    ADTModeBase::P_JOINT_[i]->init();
  }

  nowState = TORQUE_ON;
}

/**
 * @brief トルクON実行
 *
 */
void ADTModeInitialize::exec_torqueon() {
  if(u16_wait_cnt_ == 0) {
    /* 初回実行時トルクON */
    for(int i = 0; i < JointAxis::J_NUM; i++) {
      ADTModeBase::P_JOINT_[i]->set_torque_on(true);
    }
    u16_wait_cnt_++;
  } else if(u16_wait_cnt_ == c_u16_toque_on_waitcnt) {
    /* 次Stateへ */
    nowState      = MOVE_MECH_END;
    u16_wait_cnt_ = 0;
  } else {
    /* 安定化待ち */
    u16_wait_cnt_++;
  }
}

/**
 * @brief 角度リセット用の基準端まで移動
 * @note 端当たり判定を追加したい
 *
 */
void ADTModeInitialize::exec_move_mechend() {
  if(u16_wait_cnt_ < c_u16_move_end_waitcnt) {
    // ax_move_mechend(JointAxis::J0_YAW);  // Yaw軸は不要
    ax_move_mechend(JointAxis::J1_PITCH);
    // ax_move_mechend(JointAxis::J2_PITCH);  // 差動軸は別で行う
    // ax_move_mechend(JointAxis::J3_ROLL);  // 差動軸は別で行う
    ax_move_mechend(JointAxis::J4_PITCH);

    u16_wait_cnt_++;
  } else if(u16_wait_cnt_ == c_u16_move_end_waitcnt) {
    /* 次Stateへ */
    nowState      = RESET_ANGLE;
    u16_wait_cnt_ = 0;
  }
}

/**
 * @brief 現在の位置を基準端として角度オフセット値をリセット
 *
 */
void ADTModeInitialize::exec_resetangle() {
  // ax_reset_angle(JointAxis::J0_YAW);  // Yaw軸は不要
  ax_reset_angle(JointAxis::J1_PITCH);
  ax_reset_angle(JointAxis::J2_PITCH);  // 差動軸はその場リセット
  ax_reset_angle(JointAxis::J3_ROLL);  // 差動軸はその場リセット
  ax_reset_angle(JointAxis::J4_PITCH);

  /* 次Stateへ */
  nowState = MOVE_INIT_POS;
}

/**
 * @brief 初期角度へ移動
 *
 */
void ADTModeInitialize::exec_move_initpos() {
  bool _isComp = false;

  for(int i = 0; i < JointAxis::J_NUM; i++) {
    float vel     = -ADTModeBase::P_JOINT_[i]->get_vel_for_init_degps(); // 逆方向
    float initpos = ADTModeBase::P_JOINT_[i]->get_initpos_deg();
    // float nowpos  = ADTModeBase::P_JOINT_[i]->get_now_deg();
    float nowpos = ADTModeBase::P_JOINT_[i]->get_tgt_deg();
    float tgtpos = nowpos + vel * ADTModeBase::FL_CYCLE_TIME_S;

    if(((vel > 0) && (tgtpos > initpos)) || ((vel < 0) && (tgtpos < initpos))) {
      tgtpos  = initpos;
      _isComp = true;
    } else {
      if((i == JointAxis::J2_PITCH) || (i == JointAxis::J3_ROLL)) {
        // 差動軸は別で行う
      } else {
        _isComp = false;
      }
    }

    ADTModeBase::P_JOINT_[i]->set_tgt_ang_deg(tgtpos);
    ADTModeBase::P_JOINT_[i]->set_curlim_A(ADTModeBase::P_JOINT_[i]->get_curlim_default_A());
    ADTModeBase::P_JOINT_[i]->set_force_current(0.0f);
  }

  // 全ての軸がリセット完了した場合
  if(_isComp) {
    DEBUG_PRINT_STR_ADT("[ADT]INIT Comp\n");
    nowState = COMPLETED;
  }
}

/**
 * @brief 単軸を基準端へ押しあてる動作
 *
 * @param ax : JointAxis
 */
void ADTModeInitialize::ax_move_mechend(JointAxis ax) {
  float vel    = ADTModeBase::P_JOINT_[ax]->get_vel_for_init_degps();
  float curlim = ADTModeBase::P_JOINT_[ax]->get_cur_for_init_A();
  float nowpos = ADTModeBase::P_JOINT_[ax]->get_now_deg();
  float tgtpos = ADTModeBase::P_JOINT_[ax]->get_tgt_deg();

  ADTModeBase::P_JOINT_[ax]->set_tgt_ang_deg(tgtpos + vel * ADTModeBase::FL_CYCLE_TIME_S);
  ADTModeBase::P_JOINT_[ax]->set_curlim_A(curlim);
  ADTModeBase::P_JOINT_[ax]->set_force_current(0.0f);
}

/**
 * @brief 単軸角度をリセットする
 *
 * @param ax : JointAxis
 */
void ADTModeInitialize::ax_reset_angle(JointAxis ax) {
  ADTModeBase::P_JOINT_[ax]->mech_reset_pos();

  // 現在値でターゲットを上書き
  ADTModeBase::P_JOINT_[ax]->set_tgt_ang_deg(ADTModeBase::P_JOINT_[ax]->get_now_deg());
}

}; // namespace ADT
