#include <string.h>

#include "AD_joint_mybldc_servo.hpp"

namespace ADT {

void JointMyBldcServo::update() {
  if(!is_torque_on) {
    /* トルクOFF時 */
    /* STOP処理 */
    memset(txmsg.u8_data, 0, 8);
    u32_txcmdid = CMD_ID_REQ_TORQUE_OFF;
    is_updated_txdata = true;

  } else if(!is_torque_on_prev && is_torque_on) {
    /*トルクOFF→ON*/
    memset(txmsg.u8_data, 0, 8);
    u32_txcmdid = CMD_ID_REQ_TORQUE_ON;
    is_updated_txdata = true;

  } else if(is_torque_on) {
    /* 駆動処理 */
    int32_t s32_tgt_pos_q16 = (int32_t)(fl_raw_tgt_deg * c_params.fl_gear_ratio * c_params.fl_motor_dir * 65536.0f);

    /* データ作成 */
    txmsg.reqMvAng.s32_tgt_ang_deg_Q16 = s32_tgt_pos_q16;
    txmsg.reqMvAng.u16_movetime_ms = (uint16_t)(c_params.fl_ctrl_time_s * 1000.0f);
    txmsg.reqMvAng.u16_currlim_A_Q8 = (uint16_t)(fl_curlim_A*256.0f);
    u32_txcmdid = CMD_ID_REQ_MOVE_ANGLE;

    is_updated_txdata = true;
  }

  is_torque_on_prev = is_torque_on;
  
}

/**
 * @brief 
 * 
 * @param cmdid : デバイスIDを除いたコマンドID
 * @param rxMsg 
 * @param microsec_id 
 */
void JointMyBldcServo::rx_callback(uint32_t cmdid, RES_MESSAGE *rxMsg, int16_t microsec_id){

    switch (cmdid)
    {
    case CMD_ID_RES_STATUS_SUMMARY:
        /* SUMMARY STATUSの受信 */
        rx_summary_status(rxMsg->resSummary);
        break;
    default:
        break;
    }

}

/**
 * @brief 
 * 
 * @param sts 
 */
void JointMyBldcServo::rx_summary_status(RES_STATUS_SUMMAY& sts){
  /* 角度情報の保存 */
  fl_raw_now_deg = (float)sts.s16_out_ang_deg_Q4 / 16.0f / c_params.fl_gear_ratio * c_params.fl_motor_dir;

  /* 電流情報の保存 */
  fl_out_now_cur = (float)sts.s8_motor_curr_A_Q4 / 16.0f * c_params.fl_motor_dir;

  /* トルクOFF時は現目標位置を現在位置で上書き */
  if(!is_torque_on) fl_raw_tgt_deg = fl_raw_now_deg;
}

} // namespace ADT