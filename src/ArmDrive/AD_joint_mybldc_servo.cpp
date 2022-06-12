#include <string.h>

#include "AD_joint_mybldc_servo.hpp"

namespace ADT {

void JointMyBldcServo::update() {

  
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
  fl_raw_now_deg = (float)sts.s16_out_ang_deg_Q4 / 16.0f;

  /* 電流情報の保存 */
  fl_out_now_cur = (float)sts.s8_motor_curr_A_Q4 / 16.0f;

  /* トルクOFF時は現目標位置を現在位置で上書き */
  if(!is_torque_on) fl_raw_tgt_deg = fl_raw_now_deg;
}

} // namespace ADT