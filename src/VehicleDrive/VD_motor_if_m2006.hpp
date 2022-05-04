#ifndef VD_MOTOR_IF_M2006_HPP_
#define VD_MOTOR_IF_M2006_HPP_

#include "global_config.hpp"

namespace VDT {

#define MOTOR_STATUS_BUF_LEN (3)

class MOTOR_IF_M2006 {
public:
  struct CanMsgRx {
    uint8_t u8_angle_h;
    uint8_t u8_angle_l;
    uint8_t u8_speed_h;
    uint8_t u8_speed_l;
    uint8_t u8_curr_h;
    uint8_t u8_curr_l;
    uint8_t u8_rsev[2];
  };

  struct Status {
    int16_t s16_microsec_id;
    int16_t s16_rawAngle;
    int16_t s16_rawSpeedRpm;
    int16_t s16_rawCurr;
  };

public:
  MOTOR_IF_M2006(){};

  /* 電流指示値 */
  void set_rawCurr_tgt(int16_t _tgt_cur) { s16_rawCurr_tgt = sat_curr(_tgt_cur); };
  void set_CurrA_tgt(float _curr_A) { s16_rawCurr_tgt = sat_curr((int16_t)(_curr_A * AMPERE_TO_RAW_CURR)); };
  void set_rawCurr_lim(int16_t _lim_cur) { s16_rawCurr_lim = _lim_cur; };
  void set_CurrA_lim(float _lim_cur) { s16_rawCurr_lim = (int16_t)(_lim_cur * AMPERE_TO_RAW_CURR); };

  /* 状態値取得 */
  int64_t get_rawAngleSum() { return s64_rawAngleSum; };

  void get_status_latest(Status &sts) {
    uint8_t read_idx = status_head;
    sts              = status_buf[read_idx];
  }

  void get_status_estimate(Status &sts, int16_t microsec_id);

  /* CAN通信用 */
  int16_t get_rawCurr_tgt() { return s16_rawCurr_tgt; };
  void    rx_callback(CanMsgRx *rxMsg, int16_t microsec_id);

protected:
  inline int16_t couplingU8toS16(uint8_t h, uint8_t l) { return (int16_t)((h << 8) | l); };
  inline int16_t sat_curr(int16_t _curr) { return (_curr > s16_rawCurr_lim) ? s16_rawCurr_lim : ((_curr < -s16_rawCurr_lim) ? -s16_rawCurr_lim : _curr); };

  /* 指示値 */
  int16_t s16_rawCurr_tgt;
  int16_t s16_rawCurr_lim = 3000;

  /* 状態値 */
  int64_t s64_rawAngleSum = 0;

  Status  status_buf[MOTOR_STATUS_BUF_LEN];
  uint8_t status_head = 0;

  /* 定数 */
  static constexpr int16_t RAW_ANGLE_PER_A_ROTATION = 8191;
  static constexpr float   RAW_CURR_TO_AMPERE       = 0.001f;
  static constexpr float   AMPERE_TO_RAW_CURR       = 1000.0f;
};

} // namespace VDT

#endif