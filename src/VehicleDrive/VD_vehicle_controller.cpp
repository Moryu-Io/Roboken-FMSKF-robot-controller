#include "VD_vehicle_controller.hpp"

namespace VDT {

void VEHICLE_CTRL::update() {
  /* IMUとの通信をKick */
  // parts_.p_imu->kickCom();

  /* 各モータの回転情報を取得 */
  MOTOR_IF_M2006::Status m_sts[M_Place::Num] = {};
  parts_.p_motor[M_Place::FL]->get_status_latest(m_sts[M_Place::FL]);
  parts_.p_motor[M_Place::BL]->get_status_latest(m_sts[M_Place::BL]);
  parts_.p_motor[M_Place::BR]->get_status_latest(m_sts[M_Place::BR]);
  parts_.p_motor[M_Place::FR]->get_status_latest(m_sts[M_Place::FR]);

  /* 各モータ回転速度[rad/s]を取得 */
  float Mvel[M_Place::Num] = {};

#if 1
  Mvel[M_Place::FL] = (float)m_sts[M_Place::FL].s16_rawSpeedRpm * MOTOR_IF_M2006::RPM_TO_RADPS;
  Mvel[M_Place::BL] = (float)m_sts[M_Place::BL].s16_rawSpeedRpm * MOTOR_IF_M2006::RPM_TO_RADPS;
  Mvel[M_Place::BR] = (float)m_sts[M_Place::BR].s16_rawSpeedRpm * MOTOR_IF_M2006::RPM_TO_RADPS;
  Mvel[M_Place::FR] = (float)m_sts[M_Place::FR].s16_rawSpeedRpm * MOTOR_IF_M2006::RPM_TO_RADPS;
#else
  Mvel[M_Place::FL] = m_sts[M_Place::FL].flt_SpeedRadPS;
  Mvel[M_Place::BL] = m_sts[M_Place::BL].flt_SpeedRadPS;
  Mvel[M_Place::BR] = m_sts[M_Place::BR].flt_SpeedRadPS;
  Mvel[M_Place::FR] = m_sts[M_Place::FR].flt_SpeedRadPS;
#endif

  /* 各モータ回転量[rad]を取得 */
  float Mrad[M_Place::Num] = {};

  Mrad[M_Place::FL] = (float)parts_.p_motor[M_Place::FL]->get_rawAngleSum() * MOTOR_IF_M2006::OUT_RAD_PER_RAW_ANGLE * MOTOR_IF_M2006::GEAR_RATIO_INV;
  Mrad[M_Place::BL] = (float)parts_.p_motor[M_Place::BL]->get_rawAngleSum() * MOTOR_IF_M2006::OUT_RAD_PER_RAW_ANGLE * MOTOR_IF_M2006::GEAR_RATIO_INV;
  Mrad[M_Place::BR] = (float)parts_.p_motor[M_Place::BR]->get_rawAngleSum() * MOTOR_IF_M2006::OUT_RAD_PER_RAW_ANGLE * MOTOR_IF_M2006::GEAR_RATIO_INV;
  Mrad[M_Place::FR] = (float)parts_.p_motor[M_Place::FR]->get_rawAngleSum() * MOTOR_IF_M2006::OUT_RAD_PER_RAW_ANGLE * MOTOR_IF_M2006::GEAR_RATIO_INV;

  /* 車体フレーム位置[mm]/速度[mm/s]に変換 */
  conv_Mdir_to_Vdir(Mvel, now_vhcl_vel_mmps);
  conv_Mdir_to_Vdir(Mrad, now_vhcl_pos_mm_);

  /* IMU情報取得 */
  // while(!parts_.p_imu->isComComp()) {}
  // parts_.p_imu->getComData(now_imu_data_);

  /* 車体制御計算 */
  float Mvel_tgt[M_Place::Num] = {};
  now_vhcl_vel_tgt_mmps.x      = parts_.p_vel_interp[En_Dir::Dir_X]->update();
  now_vhcl_vel_tgt_mmps.y      = parts_.p_vel_interp[En_Dir::Dir_Y]->update();
  now_vhcl_vel_tgt_mmps.th     = parts_.p_vel_interp[En_Dir::Dir_TH]->update();
  conv_Vdir_to_Mdir(now_vhcl_vel_tgt_mmps, Mvel_tgt);

  /* モータへの指示 */
  if(isPowerOn) {
    /* 速度制御 */
    parts_.p_ctrl[M_Place::FL]->set_target(Mvel_tgt[M_Place::FL] * MOTOR_IF_M2006::GEAR_RATIO);
    parts_.p_ctrl[M_Place::BL]->set_target(Mvel_tgt[M_Place::BL] * MOTOR_IF_M2006::GEAR_RATIO);
    parts_.p_ctrl[M_Place::BR]->set_target(Mvel_tgt[M_Place::BR] * MOTOR_IF_M2006::GEAR_RATIO);
    parts_.p_ctrl[M_Place::FR]->set_target(Mvel_tgt[M_Place::FR] * MOTOR_IF_M2006::GEAR_RATIO);

    /* トルク指示 */
    parts_.p_motor[M_Place::FL]->set_CurrA_tgt(parts_.p_ctrl[M_Place::FL]->update(Mvel[M_Place::FL]));
    parts_.p_motor[M_Place::BL]->set_CurrA_tgt(parts_.p_ctrl[M_Place::BL]->update(Mvel[M_Place::BL]));
    parts_.p_motor[M_Place::BR]->set_CurrA_tgt(parts_.p_ctrl[M_Place::BR]->update(Mvel[M_Place::BR]));
    parts_.p_motor[M_Place::FR]->set_CurrA_tgt(parts_.p_ctrl[M_Place::FR]->update(Mvel[M_Place::FR]));

    //DEBUG_PRINT_VDT_MOTOR("[VDT]tgt:%d,%d,%d,%d\n", (int)(Mvel_tgt[M_Place::FL] * MOTOR_IF_M2006::GEAR_RATIO), (int)(Mvel_tgt[M_Place::BL] * MOTOR_IF_M2006::GEAR_RATIO), (int)(Mvel_tgt[M_Place::BR] * MOTOR_IF_M2006::GEAR_RATIO), (int)(Mvel_tgt[M_Place::FR] * MOTOR_IF_M2006::GEAR_RATIO));

    //DEBUG_PRINT_VDT_MOTOR("[VDT]vel:%d,%d,%d,%d\n", (int)Mvel[M_Place::FL], (int)Mvel[M_Place::BL], (int)Mvel[M_Place::BR], (int)Mvel[M_Place::FR]);
  } else {
    /* 速度目標リセット */
    parts_.p_vel_interp[En_Dir::Dir_X]->reset();
    parts_.p_vel_interp[En_Dir::Dir_Y]->reset();
    parts_.p_vel_interp[En_Dir::Dir_TH]->reset();

    /* 速度制御リセット */
    parts_.p_ctrl[M_Place::FL]->reset();
    parts_.p_ctrl[M_Place::BL]->reset();
    parts_.p_ctrl[M_Place::BR]->reset();
    parts_.p_ctrl[M_Place::FR]->reset();

    /* トルク0指示 */
    parts_.p_motor[M_Place::FL]->set_CurrA_tgt(0.0f);
    parts_.p_motor[M_Place::BL]->set_CurrA_tgt(0.0f);
    parts_.p_motor[M_Place::BR]->set_CurrA_tgt(0.0f);
    parts_.p_motor[M_Place::FR]->set_CurrA_tgt(0.0f);
  }
}

void VEHICLE_CTRL::set_target_vel(Direction &_vel, Direction &_acl, Direction &_jrk) {
  parts_.p_vel_interp[En_Dir::Dir_X]->set_target_params(_vel.x, _acl.x, _jrk.x);
  parts_.p_vel_interp[En_Dir::Dir_Y]->set_target_params(_vel.y, _acl.y, _jrk.y);
  parts_.p_vel_interp[En_Dir::Dir_TH]->set_target_params(_vel.th, _acl.th, _jrk.th);
}

/**
 * @brief　車体フレームからモータフレームへの変換
 *
 * @param _Vdir : Direction 車体移動量/速度
 * @param _Mdir : float[4] モータ回転量/速度 radianベース
 */
void VEHICLE_CTRL::conv_Vdir_to_Mdir(Direction &_Vdir, float *_Mdir) {
  _Mdir[M_Place::FL] = (_Vdir.x - _Vdir.y - SQRTF2 * WHEEL_L_MM * _Vdir.th * 4.0f) / WHEEL_RADIUS_MM;
  _Mdir[M_Place::BL] = (_Vdir.x + _Vdir.y - SQRTF2 * WHEEL_L_MM * _Vdir.th * 4.0f) / WHEEL_RADIUS_MM;
  _Mdir[M_Place::BR] = (_Vdir.x - _Vdir.y + SQRTF2 * WHEEL_L_MM * _Vdir.th * 4.0f) / WHEEL_RADIUS_MM;
  _Mdir[M_Place::FR] = (_Vdir.x + _Vdir.y + SQRTF2 * WHEEL_L_MM * _Vdir.th * 4.0f) / WHEEL_RADIUS_MM;
}

/**
 * @brief モータフレームから車体フレームへの変換
 *
 * @param _Mdir : float[4] モータ回転量/速度 radianベース
 * @param _Vdir : Direction 車体移動量/速度
 */
void VEHICLE_CTRL::conv_Mdir_to_Vdir(float *_Mdir, Direction &_Vdir) {
  _Vdir.x  = (_Mdir[M_Place::FL] + _Mdir[M_Place::BL] + _Mdir[M_Place::BR] + _Mdir[M_Place::FR]) * 0.25f * WHEEL_RADIUS_MM;
  _Vdir.y  = (-_Mdir[M_Place::FL] + _Mdir[M_Place::BL] - _Mdir[M_Place::BR] + _Mdir[M_Place::FR]) * 0.25f * WHEEL_RADIUS_MM;
  _Vdir.th = (-_Mdir[M_Place::FL] - _Mdir[M_Place::BL] + _Mdir[M_Place::BR] + _Mdir[M_Place::FR]) * 0.25f / SQRTF2 / WHEEL_L_MM * WHEEL_RADIUS_MM;
}

}; // namespace VDT