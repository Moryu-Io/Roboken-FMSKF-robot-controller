#include <string.h>

#include "../Utility/util_mymath.hpp"
#include "AD_joint_mg_servo.hpp"

namespace ADT {

// コマンド
const uint8_t MG4005_WRITE_RAM_PID_CMD          = 0x31;
const uint8_t MG4005_SHUTDOWN_MOTOR_CMD         = 0x80;
const uint8_t MG4005_READ_MULTI_ANGLE_CMD       = 0x92;
const uint8_t MG4005_READ_MOTOR_STATE_2_CMD     = 0x9C;
const uint8_t MG4005_WRITE_TORQUE_CTRL_CMD      = 0xA1;
const uint8_t MG4005_WRITE_MULTI_POS_CTRL_2_CMD = 0xA4;

// 定数
constexpr double DB_ANG_RAW_TO_DEG = -1.0f / 100.0f / 10.0f / 256.0f; // 算術シフト分も考慮. 出力段1回転360°に変換
constexpr float  FL_ANG_DEG_TO_RAW = -100.0f * 10.0f;
constexpr float  FL_VEL_DPS_TO_RAW = -10.0f; // 減速比分のみ考慮
constexpr float  FL_CURR_DIR  = -1.0f;

// 制御パラメータ
JointMgServo::GimPosCtrlGain InitGain = {
      .fl_pg    = 0.01f,
      .fl_ig    = 0.0f,
      .fl_dg    = 0.0f,
      .fl_ilim  = 0.0f,
      .fl_lpffr = 10.0f,
};
JointMgServo::GimPosCtrlGain PosGain = {
      .fl_pg    = 0.01f,
      .fl_ig    = 0.00f,
      .fl_dg    = 0.00f,
      .fl_ilim  = 0.1f,
      .fl_lpffr = 10.0f,
};

void JointMgServo::init() {
  is_torque_on_prev = false;
  is_torque_on      = false;
  is_connected      = true;

  /* 初期化用パラメータ */
  set_myctrl_gain_params(InitGain);

  /* Pos要求 */
  set_reqmsg_multipos();
}

void JointMgServo::update() {

  if(is_torque_on_prev && !is_torque_on) {
    /* トルクON->トルクOFF遷移時 */
    // subproc_shutdown(); // shutdownはしない
    pos_ctrl_.reset();
  } else if(!is_initialized && is_torque_on) {
    /* Pos初期化されていないけどトルクON時 */
    subproc_torquectrl();
  } else if(is_torque_on) {
    /* トルクON時＆Position制御 */
    // set_myctrl_gain_params(PosGain);
    // subproc_torquectrl();
    subproc_posctrl();
  } else if(!is_torque_on){
    /* トルクOFF時＆Pos初期化時 */
    set_myctrl_gain_params(InitGain);
    subproc_torquectrl();
  }

  set_reqmsg_multipos();
  is_torque_on_prev  = is_torque_on;
  fl_pre_raw_tgt_deg = fl_raw_tgt_deg; // 目標速度用に前の速度を残しておく
}

void JointMgServo::rx_callback(MgMsgRx *rxMsg, int16_t microsec_id) {
  if(rxMsg->u8_d[0] == 0x92) {
    /* read multi-loop angle */
    uint64_t u64_ang = 0;
    for(int i = 0; i < 7; i++) {
      u64_ang = u64_ang | (rxMsg->mlt_ang.u8_ang[i] << (i * 8));
    }
    fl_raw_now_deg = (float)((double)((int64_t)(u64_ang << 8)) * DB_ANG_RAW_TO_DEG);

    /* トルクOFF時は現目標位置を現在位置で上書き */
    if(!is_torque_on) fl_raw_tgt_deg = fl_raw_now_deg;

  } else if(rxMsg->u8_d[0] == 0x9C || rxMsg->u8_d[0] == 0xA1) {
    /* summary */
    /* 電流情報の保存 */
    fl_out_now_cur = FL_CURR_DIR*(float)conv_raw_to_current((double)rxMsg->summary.s16_iq);
  }
}

void JointMgServo::subproc_shutdown() {
  /* Motor Shutdown処理 */
  MgMsgTx *p_txparams = (MgMsgTx *)tx1data;

  /* コマンド作成 */
  p_txparams->cmd_generic.u8_cmd = MG4005_SHUTDOWN_MOTOR_CMD;
  memset(p_txparams->cmd_generic.u8_dummy, 0, 7);
  is_updated_txdata1 = true;
}

void JointMgServo::subproc_torquectrl() {
  MgMsgTx *p_txparams = (MgMsgTx *)tx1data;

  /* 速度情報の作成 */
  pos_ctrl_.set_target(fl_raw_tgt_deg);
  float _fl_iq    = pos_ctrl_.update(fl_raw_now_deg);

  // 絶対角度に応じたFFトルク値 90deg → 50mA
  // 初期化されていない場合はこの演算を飛ばす
  if(is_initialized){
    _fl_iq -= 0.05f*UTIL::mymath::sinf(UTIL::mymath::deg2rad(get_now_deg()));
  }

  _fl_iq          = (_fl_iq > fl_curlim_A) ? fl_curlim_A : ((_fl_iq < -fl_curlim_A) ? -fl_curlim_A : _fl_iq);
  int16_t _s16_iq = (int16_t)(FL_CURR_DIR*conv_current_to_raw((double)_fl_iq));

  // 最終リミット(念のため)
  const int16_t C_S16_FINAL_LIM = 450;
  _s16_iq = (_s16_iq > C_S16_FINAL_LIM) ? C_S16_FINAL_LIM : ((_s16_iq < -C_S16_FINAL_LIM) ? -C_S16_FINAL_LIM : _s16_iq);


  /* コマンド作成 */
  p_txparams->iqctrl.u8_cmd       = MG4005_WRITE_TORQUE_CTRL_CMD;
  p_txparams->iqctrl.u8_dummy0[0] = 0;
  p_txparams->iqctrl.u8_dummy0[1] = 0;
  p_txparams->iqctrl.u8_dummy0[2] = 0;
  p_txparams->iqctrl.s16_iq       = _s16_iq;
  p_txparams->iqctrl.u8_dummy1[0] = 0;
  p_txparams->iqctrl.u8_dummy1[1] = 0;
  is_updated_txdata1              = true;
}

void JointMgServo::subproc_posctrl() {
  MgMsgTx *p_txparams = (MgMsgTx *)tx1data;

  /* 速度情報の作成 */
  float    fl_tgt_vel  = fabsf((fl_raw_tgt_deg - fl_pre_raw_tgt_deg) / c_params.fl_ctrl_time_s * FL_VEL_DPS_TO_RAW);
  uint16_t u16_tgt_vel = (uint16_t)((fl_tgt_vel > 1800) ? 1800 : fl_tgt_vel);

  /* コマンド作成 */
  p_txparams->posctrl2.u8_cmd      = MG4005_WRITE_MULTI_POS_CTRL_2_CMD;
  p_txparams->posctrl2.u8_dummy    = 0;
  p_txparams->posctrl2.u16_vel_lim = u16_tgt_vel;
  p_txparams->posctrl2.s32_ang     = (int32_t)(fl_raw_tgt_deg * FL_ANG_DEG_TO_RAW);
  is_updated_txdata1               = true;
}

/**
 * @brief Tx2にPosRead要求を設定
 *
 */
void JointMgServo::set_reqmsg_multipos() {
  MgMsgTx *p_txparams            = (MgMsgTx *)tx2data;
  p_txparams->cmd_generic.u8_cmd = MG4005_READ_MULTI_ANGLE_CMD;
  memset(p_txparams->cmd_generic.u8_dummy, 0, 7);
  is_updated_txdata2 = true;
}

} // namespace ADT