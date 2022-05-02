#include "AD_joint_ics_servo.hpp"

namespace ADT {

void JointIcsServo::update() {

  // 角度値の更新
  int32_t s32_servo_tgt_pos = p_ics_serial->degPos100((int32_t)((fl_out_tgt_deg - fl_out_ofs_deg) * 100.0f));

  if(s32_servo_tgt_pos == -1) {
    return;
  }

  int32_t s32_servo_now_pos = 0;
  if(is_torque_on) {
    s32_servo_now_pos = p_ics_serial->setPos(u8_id, (uint32_t)s32_servo_tgt_pos);
  } else {
    s32_servo_now_pos = p_ics_serial->setFree(u8_id);
  }

  fl_out_now_deg = (float)p_ics_serial->posDeg100(s32_servo_now_pos) * 0.01f + fl_out_ofs_deg;

  // 電流値の更新
  int8_t s8_now_cur = (int8_t)p_ics_serial->getCur(u8_id);
  fl_out_now_cur    = s8_now_cur;
}

/**
 * @brief 初期設定関数
 * 
 * @param _p_ics_serial : begin済みのものを渡す
 * @param _id : このクラスで操作するサーボのID
 */
void JointIcsServo::init(IcsHardSerialClass *_p_ics_serial, uint8_t _id) {
  p_ics_serial = _p_ics_serial;
  u8_id        = _id;

  p_ics_serial->begin();

  int32_t s32_servo_now_pos = p_ics_serial->setFree(u8_id);
  fl_out_now_deg            = (float)p_ics_serial->posDeg100(s32_servo_now_pos) * 0.01f + fl_out_ofs_deg;
  fl_out_tgt_deg            = fl_out_now_deg; // 暴走しないようにTargetを現在値にしておく

  p_ics_serial->setSpd(u8_id, 127);
  p_ics_serial->setCur(u8_id, 63);
};

} // namespace ADT