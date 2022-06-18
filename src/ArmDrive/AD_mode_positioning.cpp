#include "AD_mode_positioning.hpp"

namespace ADT {

void ADTModePositioning::doInit() {
  nowState = State::STANDBY;
}

void ADTModePositioning::update() {
  switch(nowState) {
  case State::STANDBY:
    exec_standby();
    break;
  case State::MOVING:
    exec_moving();
    break;
  default:
    break;
  }
}

/**
 * @brief 待機
 *
 */
void ADTModePositioning::exec_standby() {
  is_comp = true;

  if(!cmd_q_.empty()) {
    /* キューにコマンドが存在する場合 */
    now_cmd_ = cmd_q_.front();
    cmd_q_.pop_front();

    /* 移動量の算出 */
    u32_move_cnt_ = (int)((float)now_cmd_.u32_dt_ms * 0.001f / ADTModeBase::FL_CYCLE_TIME_S);
    u32_move_cnt_ = (u32_move_cnt_ == 0) ? 1 : u32_move_cnt_;

    fl_move_deg_[0] = (now_cmd_.fl_tgt_pos_deg[0] - ADTModeBase::P_JOINT_[0]->get_now_deg()) / (float)u32_move_cnt_;
    fl_move_deg_[1] = (now_cmd_.fl_tgt_pos_deg[1] - ADTModeBase::P_JOINT_[1]->get_now_deg()) / (float)u32_move_cnt_;
    fl_move_deg_[4] = (now_cmd_.fl_tgt_pos_deg[4] - ADTModeBase::P_JOINT_[3]->get_now_deg()) / (float)u32_move_cnt_; // 3:J4_PITCH

    /* 状態遷移 */
    u32_cycle_counter_ = 0;
    is_comp            = false; // Moving中はFalse
    nowState           = State::MOVING;

    DEBUG_PRINT_ADT("[ADT]Positioning:CMDID:%d\n", now_cmd_.u32_id);
  }
}

/**
 * @brief 駆動実行
 *
 */
void ADTModePositioning::exec_moving() {
  float _fl_tgt_pos = 0.0f;

  /* 目標位置の設定 */
  _fl_tgt_pos = now_cmd_.fl_tgt_pos_deg[0] - fl_move_deg_[0] * (float)(u32_move_cnt_ - u32_cycle_counter_);
  ADTModeBase::P_JOINT_[0]->set_tgt_ang_deg(_fl_tgt_pos);

  _fl_tgt_pos = now_cmd_.fl_tgt_pos_deg[1] - fl_move_deg_[1] * (float)(u32_move_cnt_ - u32_cycle_counter_);
  ADTModeBase::P_JOINT_[1]->set_tgt_ang_deg(_fl_tgt_pos);

  // 3:J4_PITCH
  _fl_tgt_pos = now_cmd_.fl_tgt_pos_deg[4] - fl_move_deg_[4] * (float)(u32_move_cnt_ - u32_cycle_counter_);
  ADTModeBase::P_JOINT_[3]->set_tgt_ang_deg(_fl_tgt_pos);

  if(u32_move_cnt_ <= u32_cycle_counter_) {
    // 過去ID保存
    u32_prev_cmd_id_[1] = u32_prev_cmd_id_[0];
    u32_prev_cmd_id_[0] = now_cmd_.u32_id;
    // 駆動終了のためSTANDBYに戻る
    nowState = State::STANDBY;

    DEBUG_PRINT_ADT("[ADT]PositioningComp:CMDID:%d\n", now_cmd_.u32_id);
  } else {
    u32_cycle_counter_++;
  }
}

/**
 * @brief コマンドPush
 * 
 * @param _cmd 
 */
void ADTModePositioning::push_cmd(PosCmd &_cmd) { 
  if(cmd_q_.size() >= CU32_Q_MAX_NUM){
    cmd_q_.pop_front();
  }
  cmd_q_.push_back(_cmd);

  DEBUG_PRINT_ADT("[ADT]PushCmd:%d\n", _cmd.u32_id);
};


/**
 * @brief 指定されたコマンドIDがどのような状態かを検索する
 *
 *
 * @param _id : 状態を探すコマンドのID
 * @return int32_t : 状態
 */
int32_t ADTModePositioning::get_q_cmd_status(uint32_t _id) {
  int32_t _cmd_sts = (int32_t)CmdStatus::NO_DATA;

  /* Queue内のコマンド確認 */
  for(auto it = cmd_q_.cbegin(); it != cmd_q_.cend(); ++it) {
    if(_id == (*it).u32_id) _cmd_sts = (int32_t)CmdStatus::PROCESSING;
  }

  /* 過去コマンド確認 */
  for(uint32_t pre_id : u32_prev_cmd_id_) {
    if(_id == pre_id) _cmd_sts = (int32_t)CmdStatus::DONE;
  }

  return _cmd_sts;
}

} // namespace ADT