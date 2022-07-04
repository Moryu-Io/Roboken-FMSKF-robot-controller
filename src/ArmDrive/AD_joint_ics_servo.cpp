#include "AD_joint_ics_servo.hpp"

namespace ADT {

void JointIcsServo::update() {
  if(!is_connected) {
    return;
  }

  // 角度値の更新
  int32_t s32_servo_tgt_pos = p_ics_serial->degPos100((int32_t)(fl_raw_tgt_deg * 100.0f));

  if(s32_servo_tgt_pos == -1) {
    return;
  }

  int32_t s32_servo_now_pos = 0;
  if(is_torque_on) {
    s32_servo_now_pos = p_ics_serial->setPos(u8_id, (uint32_t)s32_servo_tgt_pos);
  } else {
    s32_servo_now_pos = p_ics_serial->setFree(u8_id);
  }

  fl_raw_now_deg = (float)p_ics_serial->posDeg100(s32_servo_now_pos) * 0.01f;

  // 電流値の更新
  //int8_t s8_now_cur = (int8_t)p_ics_serial->getCur(u8_id);
  //fl_out_now_cur    = s8_now_cur;
}

/**
 * @brief 初期設定関数
 *
 */
void JointIcsServo::init() {
  if(p_ics_serial == NULL) return;

  p_ics_serial->begin();

  // 脱力信号を送りつつ現在位置を取得する
  int32_t s32_servo_now_pos = p_ics_serial->setFree(u8_id);
  if(s32_servo_now_pos == -1) {
    // -1の場合は通信失敗しているので以下の処理を行わない
    is_connected = false;
    return;
  }

  fl_raw_now_deg = (float)p_ics_serial->posDeg100(s32_servo_now_pos) * 0.01f;
  fl_raw_tgt_deg = fl_raw_now_deg; // 暴走しないようにTargetを現在値にしておく

  p_ics_serial->setSpd(u8_id, 127);
  p_ics_serial->setCur(u8_id, 63);
  p_ics_serial->setStrc(u8_id, 64);

  is_connected = true;
};

} // namespace ADT