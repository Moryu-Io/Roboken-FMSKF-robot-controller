#include "VD_motor_if_m2006.hpp"

namespace VDT {

/**
 * @brief モータ状態の予測値を返す
 *
 * @param sts
 * @param microsec_id
 */
void MOTOR_IF_M2006::get_status_estimate(Status &sts, int16_t microsec_id) {
  volatile uint8_t read_idx = status_head;

  // データ取得時間と現在時刻の差分を算出
  int32_t s32_delta_us = microsec_id - status_buf[read_idx].s16_microsec_id;

  s32_delta_us = (s32_delta_us < 0) ? 0x10000 - s32_delta_us : s32_delta_us;

  // 角度のみ速度情報から予測する
  sts.s16_rawAngle    = status_buf[read_idx].s16_rawAngle + status_buf[read_idx].s16_rawSpeedRpm * RAW_ANGLE_PER_A_ROTATION / 60 * s32_delta_us / 1000000;
  sts.s16_rawSpeedRpm = status_buf[read_idx].s16_rawSpeedRpm;
  sts.s16_rawCurr     = status_buf[read_idx].s16_rawCurr;
  sts.s16_microsec_id = microsec_id;
}

/**
 * @brief MSG受信時の状態格納用コールバック
 *
 * @param rxMsg
 * @param microsec_id
 */
void MOTOR_IF_M2006::rx_callback(CanMsgRx *rxMsg, int16_t microsec_id) {
  uint8_t write_idx = status_head + 1;
  if(write_idx >= MOTOR_STATUS_BUF_LEN) write_idx = 0;

  status_buf[write_idx].s16_microsec_id = microsec_id;

  int16_t raw_ang = 0;
  if(s8_motor_drive_dir == 1) {
    raw_ang = couplingU8toS16(rxMsg->u8_angle_h, rxMsg->u8_angle_l);
  } else {
    raw_ang = RAW_ANGLE_PER_A_ROTATION - couplingU8toS16(rxMsg->u8_angle_h, rxMsg->u8_angle_l);
  }
  status_buf[write_idx].s16_rawAngle    = raw_ang;
  status_buf[write_idx].s16_rawSpeedRpm = couplingU8toS16(rxMsg->u8_speed_h, rxMsg->u8_speed_l) * s8_motor_drive_dir;
  status_buf[write_idx].s16_rawCurr     = couplingU8toS16(rxMsg->u8_curr_h, rxMsg->u8_curr_l) * s8_motor_drive_dir;

  status_buf[write_idx].flt_dltOutAngle_rad = (float)(status_buf[write_idx].s16_rawAngle - status_buf[status_head].s16_rawAngle) * OUT_RAD_PER_RAW_ANGLE * GEAR_RATIO_INV;

  int16_t s16_delta_ang_raw = status_buf[write_idx].s16_rawAngle - status_buf[status_head].s16_rawAngle;

  s16_delta_ang_raw = (s16_delta_ang_raw > 4096) ? s16_delta_ang_raw - 8192 : ((s16_delta_ang_raw < -4096) ? s16_delta_ang_raw + 8192 : s16_delta_ang_raw);
  s64_rawAngleSum   = s64_rawAngleSum + s16_delta_ang_raw;

  status_head = write_idx;
}

}; // namespace VDT