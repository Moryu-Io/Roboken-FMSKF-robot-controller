#include <string.h>

#include "../Utility/util_mymath.hpp"
#include "AD_joint_gim_servo.hpp"

namespace ADT {

const uint8_t GIM_START_CMD[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
const uint8_t GIM_STOP_CMD[8]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

// constexpr float FL_ANG_RAW_TO_DEG = 1.0f / 32768.0f * 12.5f * 180.0f / 3.141592f;
constexpr float FL_ANG_RAW_TO_DEG = 1.0f / 32768.0f * 180.0f;
constexpr float FL_ANG_DEG_TO_RAW = 32768.0f / 180.0f;
constexpr float FL_CURR_RAW_TO_A  = 1.0f / 2048.0f * 4.0f;
constexpr float FL_CURR_A_TO_RAW  = 2048.0f / 4.0f;

void JointGimServo::init() {
  is_torque_on_prev = false;
  is_torque_on      = false;
  is_connected      = true;
  pos_ctrl_.reset();

  memcpy(txdata, GIM_START_CMD, 8);
  is_updated_txdata = true;
}

void JointGimServo::update() {

  if(is_torque_on_prev && !is_torque_on) {
    /* トルクON->トルクOFF時 */
    /* STOP処理 */
    GimMsgTxParamsSet *p_txparams  = (GimMsgTxParamsSet *)txdata;
    uint16_t           u16_tgt_trq = 2048;
    p_txparams->u8_pos_h           = 0;
    p_txparams->u8_pos_l           = 0;
    p_txparams->u8_vel_h           = 0;
    p_txparams->u8_vel_l4_Kp_h4    = (u16_Kp / 2) >> 8;
    p_txparams->u8_Kp_l            = (u16_Kp / 2) & 0x00FF;
    p_txparams->u8_Kd_h            = u16_Kd >> 4;
    p_txparams->u8_Kd_l4_trq_h4    = ((u16_Kd & 0x000F) << 4) | ((u16_tgt_trq >> 8) & 0x000F);
    p_txparams->u8_trq_l           = u16_tgt_trq & 0x00FF;
  
    set_myctrl_gain_params(pos_ctrl_params_off_);
    pos_ctrl_.reset();

    is_updated_txdata = true;

  } else if(is_torque_on) {
    /* 駆動処理 */
    GimMsgTxParamsSet *p_txparams  = (GimMsgTxParamsSet *)txdata;
    uint16_t           u16_tgt_trq = 0;
  
    set_myctrl_gain_params(pos_ctrl_params_);
    pos_ctrl_.set_target(fl_raw_tgt_deg);
    float _fl_tgt_trq = pos_ctrl_.update(fl_raw_now_deg);
    _fl_tgt_trq       = UTIL::mymath::satf(_fl_tgt_trq, fl_curlim_A, -fl_curlim_A);
    u16_tgt_trq       = (uint16_t)(FL_CURR_A_TO_RAW * _fl_tgt_trq + 2048.0f);

    // debug_printf("%d,%d\n", u16_tgt_trq, (int)(fl_out_now_cur / FL_CURR_RAW_TO_A) + 2048);

    /* データ作成 */
    p_txparams->u8_pos_h        = 0;
    p_txparams->u8_pos_l        = 0;
    p_txparams->u8_vel_h        = 0;
    p_txparams->u8_vel_l4_Kp_h4 = u16_Kp >> 8;
    p_txparams->u8_Kp_l         = u16_Kp & 0x00FF;
    p_txparams->u8_Kd_h         = u16_Kd >> 4;
    p_txparams->u8_Kd_l4_trq_h4 = ((u16_Kd & 0x000F) << 4) | ((u16_tgt_trq >> 8) & 0x000F);
    p_txparams->u8_trq_l        = u16_tgt_trq & 0x00FF;

    is_updated_txdata = true;
  } else if(is_connected && !is_updated_txdata){
    /* 駆動処理 */
    GimMsgTxParamsSet *p_txparams  = (GimMsgTxParamsSet *)txdata;
    uint16_t           u16_tgt_trq = 0;

    set_myctrl_gain_params(pos_ctrl_params_off_);
    pos_ctrl_.set_target(fl_raw_tgt_deg);
    float _fl_tgt_trq = pos_ctrl_.update(fl_raw_now_deg);
    _fl_tgt_trq       = UTIL::mymath::satf(_fl_tgt_trq, fl_curlim_A, -fl_curlim_A);
    u16_tgt_trq       = (uint16_t)(FL_CURR_A_TO_RAW * _fl_tgt_trq + 2048.0f);

    // debug_printf("%d,%d\n", u16_tgt_trq, (int)(fl_out_now_cur / FL_CURR_RAW_TO_A) + 2048);

    /* データ作成 */
    p_txparams->u8_pos_h        = 0;
    p_txparams->u8_pos_l        = 0;
    p_txparams->u8_vel_h        = 0;
    p_txparams->u8_vel_l4_Kp_h4 = (u16_Kp / 2)  >> 8;
    p_txparams->u8_Kp_l         = (u16_Kp / 2)  & 0x00FF;
    p_txparams->u8_Kd_h         = u16_Kd >> 4;
    p_txparams->u8_Kd_l4_trq_h4 = ((u16_Kd & 0x000F) << 4) | ((u16_tgt_trq >> 8) & 0x000F);
    p_txparams->u8_trq_l        = u16_tgt_trq & 0x00FF;

    is_updated_txdata = true;
  }

  is_torque_on_prev = is_torque_on;
}

void JointGimServo::rx_callback(GimMsgRx *rxMsg, int16_t microsec_id) {
  /* 角度情報の保存 */
  int32_t s32_rx_ang_raw = (int32_t)(rxMsg->u8_pos_h << 8 | rxMsg->u8_pos_l);
  fl_raw_now_deg         = (float)(s32_rx_ang_raw - 32768) * FL_ANG_RAW_TO_DEG;

  /* 電流情報の保存 */
  int16_t s16_rx_curr_raw = (int16_t)((rxMsg->u8_vel_l4_trq_h4 & 0x0F) << 8 | rxMsg->u8_trq_l) - 2048;
  fl_out_now_cur          = (float)s16_rx_curr_raw * FL_CURR_RAW_TO_A;

  /* トルクOFF時は現目標位置を現在位置で上書き */
  if(!is_torque_on) fl_raw_tgt_deg = fl_raw_now_deg;
}

} // namespace ADT