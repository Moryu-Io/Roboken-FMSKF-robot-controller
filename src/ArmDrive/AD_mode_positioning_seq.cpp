#include "AD_mode_positioning_seq.hpp"

namespace ADT {

void ADTModePositioningSeq::doInit() {
  nowState            = State::STANDBY;
  u16_seq_exec_idx_   = CMD_SEQ_BUF_LEN - 1;
  u16_seq_write_head_ = CMD_SEQ_BUF_LEN - 1;
}

void ADTModePositioningSeq::update() {
  // 遷移時はすぐに次の処理をしたいのでif文で分岐を記述
  if(nowState == State::STANDBY) exec_standby();
  if(nowState == State::MOVE_START) exec_move_start();
  if(nowState == State::MOVING) exec_moving();
}

/**
 * @brief 待機
 *
 */
void ADTModePositioningSeq::exec_standby() {
  is_comp = true;

  if(u16_seq_exec_idx_ != u16_seq_write_head_) {
    /* BufferにCommandSeqが存在する場合 */
    u16_seq_exec_idx_++;
    u16_seq_exec_idx_ = (u16_seq_exec_idx_ >= CMD_SEQ_BUF_LEN) ? 0 : u16_seq_exec_idx_;

    /* CommandIndex初期化 */
    u8_nowcmd_idx_     = 0;
    u32_total_move_ms_ = 0;

    /* 駆動開始Stateへ */
    nowState = State::MOVE_START;
  }
}

/**
 * @brief 駆動開始処理
 *
 */
void ADTModePositioningSeq::exec_move_start() {

  if(u8_nowcmd_idx_ >= cmd_seq_[u16_seq_exec_idx_].u8_cmd_seq_len) {
    /* このシーケンスは終了 */
    nowState = State::STANDBY;
    DEBUG_PRINT_ADT("[ADT]PosCmdSeq:%d\n", cmd_seq_[u16_seq_exec_idx_].u32_id);
  } else {
    now_cmd_ = cmd_seq_[u16_seq_exec_idx_].cmd_seq[u8_nowcmd_idx_];

    /* 移動量の算出 */
    u32_move_cnt_ = (int)((float)(now_cmd_.u32_dt_ms - u32_total_move_ms_) * 0.001f / ADTModeBase::FL_CYCLE_TIME_S);
    u32_move_cnt_ = (u32_move_cnt_ <= 0) ? 1 : u32_move_cnt_;

#if 0 // 現在位置から移動量を作り直す
    fl_move_deg_[0] = (now_cmd_.fl_tgt_pos_deg[0] - ADTModeBase::P_JOINT_[JointAxis::J0_YAW]->get_now_deg()) / (float)u32_move_cnt_;
    fl_move_deg_[1] = (now_cmd_.fl_tgt_pos_deg[1] - ADTModeBase::P_JOINT_[JointAxis::J1_PITCH]->get_now_deg()) / (float)u32_move_cnt_;
    fl_move_deg_[2] = (now_cmd_.fl_tgt_pos_deg[2] - ADTModeBase::P_JOINT_[JointAxis::J2_PITCH]->get_now_deg()) / (float)u32_move_cnt_;
    fl_move_deg_[3] = (now_cmd_.fl_tgt_pos_deg[3] - ADTModeBase::P_JOINT_[JointAxis::J3_ROLL]->get_now_deg()) / (float)u32_move_cnt_;
    fl_move_deg_[4] = (now_cmd_.fl_tgt_pos_deg[4] - ADTModeBase::P_JOINT_[JointAxis::J4_PITCH]->get_now_deg()) / (float)u32_move_cnt_;
#else // 現在Targetから移動量を作成
    fl_move_deg_[0] = (now_cmd_.fl_tgt_pos_deg[0] - ADTModeBase::P_JOINT_[JointAxis::J0_YAW]->get_tgt_deg()) / (float)u32_move_cnt_;
    fl_move_deg_[1] = (now_cmd_.fl_tgt_pos_deg[1] - ADTModeBase::P_JOINT_[JointAxis::J1_PITCH]->get_tgt_deg()) / (float)u32_move_cnt_;
    fl_move_deg_[2] = (now_cmd_.fl_tgt_pos_deg[2] - ADTModeBase::P_JOINT_[JointAxis::J2_PITCH]->get_tgt_deg()) / (float)u32_move_cnt_;
    fl_move_deg_[3] = (now_cmd_.fl_tgt_pos_deg[3] - ADTModeBase::P_JOINT_[JointAxis::J3_ROLL]->get_tgt_deg()) / (float)u32_move_cnt_;
    fl_move_deg_[4] = (now_cmd_.fl_tgt_pos_deg[4] - ADTModeBase::P_JOINT_[JointAxis::J4_PITCH]->get_tgt_deg()) / (float)u32_move_cnt_;
#endif

    /* 状態遷移 */
    u32_total_move_ms_ = now_cmd_.u32_dt_ms; // 次Cmd用に時間を保存
    u32_cycle_counter_ = 0;
    is_comp            = false; // Moving中はFalse
    nowState           = State::MOVING;

    DEBUG_PRINT_ADT("[ADT]PosCmd:%d\n", u8_nowcmd_idx_);
  }
}

/**
 * @brief 駆動実行
 *
 */
void ADTModePositioningSeq::exec_moving() {
  float _fl_tgt_pos = 0.0f;

  /* 目標位置の設定 */
  _fl_tgt_pos = now_cmd_.fl_tgt_pos_deg[0] - fl_move_deg_[0] * (float)(u32_move_cnt_ - u32_cycle_counter_);
  ADTModeBase::P_JOINT_[JointAxis::J0_YAW]->set_tgt_ang_deg(_fl_tgt_pos);

  _fl_tgt_pos = now_cmd_.fl_tgt_pos_deg[1] - fl_move_deg_[1] * (float)(u32_move_cnt_ - u32_cycle_counter_);
  ADTModeBase::P_JOINT_[JointAxis::J1_PITCH]->set_tgt_ang_deg(_fl_tgt_pos);

  _fl_tgt_pos = now_cmd_.fl_tgt_pos_deg[2] - fl_move_deg_[2] * (float)(u32_move_cnt_ - u32_cycle_counter_);
  ADTModeBase::P_JOINT_[JointAxis::J2_PITCH]->set_tgt_ang_deg(_fl_tgt_pos);

  _fl_tgt_pos = now_cmd_.fl_tgt_pos_deg[3] - fl_move_deg_[3] * (float)(u32_move_cnt_ - u32_cycle_counter_);
  ADTModeBase::P_JOINT_[JointAxis::J3_ROLL]->set_tgt_ang_deg(_fl_tgt_pos);

  _fl_tgt_pos = now_cmd_.fl_tgt_pos_deg[4] - fl_move_deg_[4] * (float)(u32_move_cnt_ - u32_cycle_counter_);
  ADTModeBase::P_JOINT_[JointAxis::J4_PITCH]->set_tgt_ang_deg(_fl_tgt_pos);

  DEBUG_PRINT_ADT("[ADT]PosCmdMoveCnt:%d,%d,%d,%d,%d,%d\n", u32_cycle_counter_, (int)ADTModeBase::P_JOINT_[JointAxis::J0_YAW]->get_tgt_deg(), (int)ADTModeBase::P_JOINT_[JointAxis::J1_PITCH]->get_tgt_deg(), (int)ADTModeBase::P_JOINT_[JointAxis::J2_PITCH]->get_tgt_deg(), (int)ADTModeBase::P_JOINT_[JointAxis::J3_ROLL]->get_tgt_deg(), (int)ADTModeBase::P_JOINT_[JointAxis::J4_PITCH]->get_tgt_deg());

  if(u32_move_cnt_ <= u32_cycle_counter_) {
    // 駆動終了のためMOVE_STARTに戻る
    u8_nowcmd_idx_++;
    nowState = State::MOVE_START;
  } else {
    u32_cycle_counter_++;
  }
}

/**
 * @brief コマンドPush
 *
 * @param _cmd
 */
void ADTModePositioningSeq::push_cmdseq(PosCmdSeq &_cmd) {
  uint16_t _new_write_idx = u16_seq_write_head_ + 1;

  _new_write_idx = (_new_write_idx >= CMD_SEQ_BUF_LEN) ? 0 : _new_write_idx;

  if(_new_write_idx == u16_seq_exec_idx_) {
    // 書き込み位置が現在実行中の場合は書き込まない(Buffer1周している)
    return;
  }
  cmd_seq_[_new_write_idx] = _cmd;
  u16_seq_write_head_      = _new_write_idx;

  DEBUG_PRINT_ADT("[ADT]PushCmdSeq:%d\n", _cmd.u32_id);
};

/**
 * @brief 指定されたコマンドIDがどのような状態かを検索する
 *
 *
 * @param _id : 状態を探すコマンドのID
 * @return int32_t : 状態
 */
int32_t ADTModePositioningSeq::get_q_cmdseq_status(uint32_t _id) {
  int32_t _cmd_sts = (int32_t)CmdStatus::NO_DATA;

  for(int i = 0; i < CMD_SEQ_BUF_LEN; i++) {
    if(cmd_seq_[i].u32_id == _id) {
      // Buffer内に同一IDがある場合、実行中or前なのか終了したIDなのかを判定する

      if(u16_seq_exec_idx_ == u16_seq_write_head_) {
        // 実行Stateと書き込み先頭が一致している場合、
        // Queueに1つしかCmdSeqが存在していない
        // その場合はCmdIndexの数で完了を確認する
        if(u8_nowcmd_idx_ >= cmd_seq_[u16_seq_exec_idx_].u8_cmd_seq_len) {
          _cmd_sts = (int32_t)CmdStatus::DONE;
        } else {
          _cmd_sts = (int32_t)CmdStatus::PROCESSING;
        }
      } else if(u16_seq_exec_idx_ < u16_seq_write_head_) {
        if((u16_seq_exec_idx_ <= i) && (i <= u16_seq_write_head_)) {
          _cmd_sts = (int32_t)CmdStatus::PROCESSING;
        } else {
          _cmd_sts = (int32_t)CmdStatus::DONE;
        }
      } else {
        // 逆転している場合、Writeが一周している
        if(((u16_seq_exec_idx_ <= i) && (i < CMD_SEQ_BUF_LEN)) || (i <= u16_seq_write_head_)) {
          _cmd_sts = (int32_t)CmdStatus::PROCESSING;
        } else {
          _cmd_sts = (int32_t)CmdStatus::DONE;
        }
      }
    }
  }

  return _cmd_sts;
}

} // namespace ADT