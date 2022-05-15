#ifndef VD_VEHICLE_CONTROLLER_HPP_
#define VD_VEHICLE_CONTROLLER_HPP_

#include "global_config.hpp"

#include "VD_imu_if_base.hpp"
#include "VD_motor_if_m2006.hpp"

namespace VDT {

/****************
 *　相手方向X
 *  　↑
 *  Y←
 ****************/
struct Direction {
  float x;
  float y;
  float th;
};

class VEHICLE_CTRL {
public:
  enum M_Place {
    FL = 0,
    BL,
    BR,
    FR,
    Num,
  };

  struct Parts {
    IMU_IF         *p_imu;
    MOTOR_IF_M2006 *p_motor[M_Place::Num];
  };

public:
  VEHICLE_CTRL(Parts &_p) : parts_(_p){};

  void update();

  void set_target_vel(Direction *_dir, uint16_t _t_ms);

  void get_vehicle_pos_mm_latest(Direction &_pos) { _pos = now_vhcl_pos_mm_; };
  void get_vehicle_vel_mmps_latest(Direction &_vel) { _vel = now_vhcl_vel_mmps; };

private:
  void conv_Vdir_to_Mdir(Direction &_Vdir, float *_Mdir);
  void conv_Mdir_to_Vdir(float *_Mdir, Direction &_Vdir);

  Parts &parts_;

  /* デバイス情報 */
  IMU_IF::Data now_imu_data_;

  /* 制御情報 */
  Direction now_vhcl_pos_mm_;
  Direction now_vhcl_vel_mmps;
  Direction now_vhcl_vel_tgt_mmps;

  /* 機体パラメータ類 */
  const float WHEEL_RADIUS_MM  = 37.5f;       // √2
  const float WHEEL_TREAD_V_MM = 17;          // 縦(X方向トレッド)
  const float WHEEL_TREAD_H_MM = 20;          // 横(Y方向トレッド)
  const float WHEEL_L_MM       = 13.08148f;   // √(V^2 + H^2) * cos(45deg - atan(a/b)) / 2
  const float SQRTF2           = 1.41421356f; // √2
};

}; // namespace VDT

#endif