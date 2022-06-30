#include "AD_mode_initpos_move.hpp"

namespace ADT {

void ADTModeInitPosMove::doInit() {
  /* 変数初期化 */
  nowState      = INIT;
  u16_wait_cnt_ = 0;
  for(int i = 0; i < JointAxis::J_NUM; i++) {
    fl_move_vel_dir_[i] = 0.0f;
  }
}

void ADTModeInitPosMove::update() {
  switch(nowState) {
  case INIT:
    exec_init();
    break;
  case TORQUE_ON:
    exec_torqueon();
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
 * @brief
 *
 */
void ADTModeInitPosMove::exec_init() {
  /* Targetを現在位置で上書き */
  for(int i = 0; i < JointAxis::J_NUM; i++) {
    ADTModeBase::P_JOINT_[i]->set_tgt_ang_deg(ADTModeBase::P_JOINT_[i]->get_now_deg());
    fl_move_vel_dir_[i] = (ADTModeBase::P_JOINT_[i]->get_initpos_deg() >= ADTModeBase::P_JOINT_[i]->get_now_deg()) ? 1.0f : -1.0f;
  }

  nowState = TORQUE_ON;
}

/**
 * @brief トルクON実行
 *
 */
void ADTModeInitPosMove::exec_torqueon() {
  if(u16_wait_cnt_ == 0) {
    /* 初回実行時トルクON */
    for(int i = 0; i < JointAxis::J_NUM; i++) {
      ADTModeBase::P_JOINT_[i]->set_torque_on(true);
    }
    u16_wait_cnt_++;
  } else if(u16_wait_cnt_ == c_u16_toque_on_waitcnt) {
    /* 次Stateへ */
    nowState      = MOVE_INIT_POS;
    u16_wait_cnt_ = 0;
  } else {
    /* 安定化待ち */
    u16_wait_cnt_++;
  }
}

/**
 * @brief 初期角度へ移動
 *
 */
void ADTModeInitPosMove::exec_move_initpos() {
  bool _isComp = true;

  for(int i = 0; i < JointAxis::J_NUM; i++) {
    float initpos = ADTModeBase::P_JOINT_[i]->get_initpos_deg();
    float vel     = fl_move_vel_dir_[i] * fabsf(ADTModeBase::P_JOINT_[i]->get_vel_for_init_degps());
    float tgtpos = ADTModeBase::P_JOINT_[i]->get_tgt_deg() + vel * ADTModeBase::FL_CYCLE_TIME_S;

    if(((vel > 0) && (tgtpos > initpos)) || ((vel < 0) && (tgtpos < initpos))) {
      tgtpos  = initpos;
      _isComp = _isComp & true;
    } else {
       _isComp = false;
    }

    ADTModeBase::P_JOINT_[i]->set_tgt_ang_deg(tgtpos);
    ADTModeBase::P_JOINT_[i]->set_curlim_A(ADTModeBase::P_JOINT_[i]->get_curlim_default_A());
    ADTModeBase::P_JOINT_[i]->set_force_current(0.0f);
  }

  // 全ての軸がリセット完了した場合
  if(_isComp) {
    DEBUG_PRINT_STR_ADT("[ADT]INIT PosMove Comp\n");
    nowState = COMPLETED;
  }
}

}; // namespace ADT
