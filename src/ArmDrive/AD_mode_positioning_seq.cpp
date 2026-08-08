#include "AD_mode_positioning_seq.hpp"

#include "../Utility/util_mymath.hpp"
#include "AD_task_main.hpp" // 補正入力(get_yaw_corr_deg / get_height_err ...)

namespace ADT {

namespace {
// ===== 補正チューニング定数(実機で調整前提) =====
constexpr float CTRL_FREQ_HZ = 100.0f; // ADTタスク制御周期[Hz]

// --- 面向けYaw補正(絶対オフセット, 常時適用) ---
constexpr float YAW_CORR_MAX_DEG = 15.0f; // Yaw補正のクランプ[deg]
constexpr float YAW_LPF_CUTOFF   = 10.0f; // Yaw補正LPFカットオフ(双一次)
constexpr float YAW_LPF_A1 = (2.0f * CTRL_FREQ_HZ - YAW_LPF_CUTOFF) / (2.0f * CTRL_FREQ_HZ + YAW_LPF_CUTOFF);
constexpr float YAW_LPF_B  = YAW_LPF_CUTOFF / (2.0f * CTRL_FREQ_HZ + YAW_LPF_CUTOFF);

// --- 胴向け高さ補正(誤差をMCUで毎周期積分・累積) ---
constexpr float    HEIGHT_K_CLIK        = 0.02f; // 高さ誤差→補正の積分ゲイン[deg/(誤差単位)/周期]
constexpr float    HEIGHT_CORR_MAX_DEG  = 20.0f; // 高さ補正累積のクランプ[deg](アンチワインドアップ兼可動域保護)
constexpr float    HEIGHT_DQ_SLEW_DEG   = 0.5f;  // 1周期あたりの補正変化量制限[deg/周期](=50deg/s@100Hz)
constexpr uint32_t HEIGHT_ERR_STALE_MS  = 100;   // 誤差がこの時間更新されなければ積分を凍結(視覚系停止時の暴走防止)
constexpr float    HEIGHT_LPF_CUTOFF    = 10.0f; // 高さ誤差LPFカットオフ(双一次)
constexpr float    HEIGHT_LPF_A1 = (2.0f * CTRL_FREQ_HZ - HEIGHT_LPF_CUTOFF) / (2.0f * CTRL_FREQ_HZ + HEIGHT_LPF_CUTOFF);
constexpr float    HEIGHT_LPF_B  = HEIGHT_LPF_CUTOFF / (2.0f * CTRL_FREQ_HZ + HEIGHT_LPF_CUTOFF);

// 高さ補正の関節配分 {J0_YAW, J1_PITCH, J2_PITCH, J3_ROLL, J4_PITCH}
// 既定は肩ピッチ(J1)のみ。先端姿勢維持のためJ1/J4分散も可(要調整)。J0/J3は0。
constexpr float HEIGHT_CORR_W[5] = {0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
} // namespace

ADTModePositioningSeq::ADTModePositioningSeq()
    : yaw_lpf_(YAW_LPF_A1, YAW_LPF_B, YAW_LPF_B),
      height_err_lpf_(HEIGHT_LPF_A1, HEIGHT_LPF_B, HEIGHT_LPF_B) {}

void ADTModePositioningSeq::doInit() {
  nowState            = State::STANDBY;
  u16_seq_exec_idx_   = CMD_SEQ_BUF_LEN - 1;
  u16_seq_write_head_ = CMD_SEQ_BUF_LEN - 1;

  isModeFirstCall = true;

  /* 補正状態の初期化 */
  fl_height_corr_deg_ = 0.0f;
  yaw_lpf_.reset();
  height_err_lpf_.reset();
  /* ベース目標を現在の指令目標でシード(区間傾きの起点) */
  for(int i = 0; i < JointAxis::J_NUM; i++) {
    fl_base_tgt_deg_[i] = ADTModeBase::P_JOINT_[i]->get_tgt_deg();
  }
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

    /* 振り始め: 高さ補正アキュムレータをリセット(summary "振り始めで dq=0") */
    fl_height_corr_deg_ = 0.0f;
    height_err_lpf_.reset();

    /* 駆動開始Stateへ */
    nowState = State::MOVE_START;

    /* 駆動開始したので初回フラグを落とす */
    isModeFirstCall = false;
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
    s32_move_cnt_ = (int)((float)(now_cmd_.u32_dt_ms - u32_total_move_ms_) * 0.001f / ADTModeBase::FL_CYCLE_TIME_S);
    s32_move_cnt_ = (s32_move_cnt_ <= 0) ? 1 : s32_move_cnt_;

#if 0 // 現在位置から移動量を作り直す
    fl_move_deg_[0] = (now_cmd_.fl_tgt_pos_deg[0] - ADTModeBase::P_JOINT_[JointAxis::J0_YAW]->get_now_deg()) / (float)s32_move_cnt_;
    fl_move_deg_[1] = (now_cmd_.fl_tgt_pos_deg[1] - ADTModeBase::P_JOINT_[JointAxis::J1_PITCH]->get_now_deg()) / (float)s32_move_cnt_;
    fl_move_deg_[2] = (now_cmd_.fl_tgt_pos_deg[2] - ADTModeBase::P_JOINT_[JointAxis::J2_PITCH]->get_now_deg()) / (float)s32_move_cnt_;
    fl_move_deg_[3] = (now_cmd_.fl_tgt_pos_deg[3] - ADTModeBase::P_JOINT_[JointAxis::J3_ROLL]->get_now_deg()) / (float)s32_move_cnt_;
    fl_move_deg_[4] = (now_cmd_.fl_tgt_pos_deg[4] - ADTModeBase::P_JOINT_[JointAxis::J4_PITCH]->get_now_deg()) / (float)s32_move_cnt_;
#else // 現在のベース目標から移動量を作成(補正を含めない)
    for(int i = 0; i < JointAxis::J_NUM; i++) {
      fl_move_deg_[i] = (now_cmd_.fl_tgt_pos_deg[i] - fl_base_tgt_deg_[i]) / (float)s32_move_cnt_;
    }
#endif

    /* 状態遷移 */
    u32_total_move_ms_ = now_cmd_.u32_dt_ms; // 次Cmd用に時間を保存
    s32_cycle_counter_ = 0;
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
  /* ---- 補正量の算出(この周期で全関節共通) ---- */
  // 面向けYaw補正: 絶対オフセットをクランプ→LPFして常時適用
  float _yaw_corr = UTIL::mymath::satf(get_yaw_corr_deg(), YAW_CORR_MAX_DEG, -YAW_CORR_MAX_DEG);
  _yaw_corr       = yaw_lpf_.update(_yaw_corr);

  // 胴向け高さ補正: 高さ誤差をMCUで積分・累積(stale凍結→LPF→slew制限→クランプ)
  float _step = 0.0f;
  if(get_height_err_age_ms() > HEIGHT_ERR_STALE_MS) {
    // 誤差が古い場合はLPF状態ごと完全に凍結(視覚系停止時の暴走防止)
    height_err_lpf_.reset();
  } else {
    float _err = height_err_lpf_.update(get_height_err());
    _step      = UTIL::mymath::satf(HEIGHT_K_CLIK * _err, HEIGHT_DQ_SLEW_DEG, -HEIGHT_DQ_SLEW_DEG);
  }
  fl_height_corr_deg_ = UTIL::mymath::satf(fl_height_corr_deg_ + _step, HEIGHT_CORR_MAX_DEG, -HEIGHT_CORR_MAX_DEG);

  /* ---- 各関節: プリセット補間(ベース) + 補正 を指令 ---- */
  for(int i = 0; i < JointAxis::J_NUM; i++) {
    float _fl_base      = now_cmd_.fl_tgt_pos_deg[i] - fl_move_deg_[i] * (float)(s32_move_cnt_ - s32_cycle_counter_);
    fl_base_tgt_deg_[i] = _fl_base; // 補正抜きベースを保存(次区間の傾き基準)

    float _fl_cmd = _fl_base;
    if(i == JointAxis::J0_YAW) {
      _fl_cmd += _yaw_corr;                              // 面向けYaw補正
    } else {
      _fl_cmd += HEIGHT_CORR_W[i] * fl_height_corr_deg_; // 胴向け高さ補正(ピッチ関節へ配分)
    }
    ADTModeBase::P_JOINT_[i]->set_tgt_ang_deg(_fl_cmd);
  }

  DEBUG_PRINT_ADT("[ADT]PosCmdMoveCnt:%d,%d,%d,%d,%d,%d\n", s32_cycle_counter_, (int)ADTModeBase::P_JOINT_[JointAxis::J0_YAW]->get_tgt_deg(), (int)ADTModeBase::P_JOINT_[JointAxis::J1_PITCH]->get_tgt_deg(), (int)ADTModeBase::P_JOINT_[JointAxis::J2_PITCH]->get_tgt_deg(), (int)ADTModeBase::P_JOINT_[JointAxis::J3_ROLL]->get_tgt_deg(), (int)ADTModeBase::P_JOINT_[JointAxis::J4_PITCH]->get_tgt_deg());

  if(s32_move_cnt_ <= s32_cycle_counter_) {
    // 駆動終了のためMOVE_STARTに戻る
    u8_nowcmd_idx_++;
    nowState = State::MOVE_START;
  } else {
    s32_cycle_counter_++;
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

  if(isModeFirstCall && (_id == 0)){
    /* 初回 かつ ID==0の場合は問答無用でNO_DATAを返す*/
    return CmdStatus::NO_DATA;
  }

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