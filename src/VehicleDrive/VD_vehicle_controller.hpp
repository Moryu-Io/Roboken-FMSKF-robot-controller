#ifndef VD_VEHICLE_CONTROLLER_HPP_
#define VD_VEHICLE_CONTROLLER_HPP_

#include "global_config.hpp"

#include "../Utility/util_controller.hpp"
#include "../Utility/util_vel_interp.hpp"

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

enum En_Dir {
  Dir_X,
  Dir_Y,
  Dir_TH,
  Dir_Num,
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
    IMU_IF           *p_imu;
    UTIL::VelInterp *p_vel_interp[En_Dir::Dir_Num];
    MOTOR_IF_M2006   *p_motor[M_Place::Num];
    UTIL::PI_D       *p_ctrl[M_Place::Num];
  };

public:
  VEHICLE_CTRL(Parts &_p) : parts_(_p){};

  void update();

  void start() { isPowerOn = true; };
  void stop() { isPowerOn = false; };
  void set_target_vel(Direction &_vel, Direction &_acl, Direction &_jrk);
  void set_now_yaw_world(float yaw_rad) { now_vhcl_pos_m_.th = yaw_rad; };

  void get_vehicle_pos_m_latest(Direction &_pos) { _pos = now_vhcl_pos_m_; };
  void get_vehicle_vel_mmps_latest(Direction &_vel) { _vel = now_vhcl_vel_mmps; };
  void get_vehicle_vel_tgt_mmps_latest(Direction &_vel) { _vel = now_vhcl_vel_tgt_mmps; };

private:
  void conv_Vdir_to_Mdir(Direction &_Vdir, float *_Mdir);
  void conv_Mdir_to_Vdir(float *_Mdir, Direction &_Vdir);

  Parts &parts_;

  /* デバイス情報 */
  IMU_IF::Data now_imu_data_;

  /* 制御情報 */
  Direction now_vhcl_pos_m_;
  Direction now_vhcl_vel_mmps;
  Direction now_vhcl_vel_tgt_mmps;

  int64_t s64_rawAngleSumPrev[M_Place::Num];

  bool isPowerOn;

  /* 機体パラメータ類 */
  const float WHEEL_RADIUS_MM  = 37.5f; // 車輪半径
  const float WHEEL_TREAD_V_MM = 170.0f; // 前後の車輪中心間距離(X方向)
  const float WHEEL_TREAD_H_MM = 200.0f; // 左右の車輪中心間距離(Y方向)
  // 45度ローラの旋回係数: 車体中心から車輪までのX距離 + Y距離
  const float WHEEL_ROTATION_LEVER_MM = (WHEEL_TREAD_V_MM + WHEEL_TREAD_H_MM) * 0.5f;
};

}; // namespace VDT

#endif
