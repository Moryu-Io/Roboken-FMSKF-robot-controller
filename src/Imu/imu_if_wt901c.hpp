#ifndef IMU_IF_WT901C_HPP_
#define IMU_IF_WT901C_HPP_

#include "imu_if_base.hpp"

namespace IMT {

class IMU_IF_WT901C : public IMU_IF {
public:
  IMU_IF_WT901C() : IMU_IF(){};

  void init() override;
  void kickCom() override;
  void update() override;
  bool isComComp() override;
  void getDataLatest(Data &_d) override;

  void getDataImmediately(Data &_d) override;

  bool isError() override {return is_error;};

  struct ConfigReg {
    uint8_t u8_reg_config;
    uint8_t u8_reg_gyro_config;
    uint8_t u8_reg_accel_config;
    uint8_t u8_reg_accel_config2;
  };

  void setImuConfigReg(ConfigReg &_cfg);

protected:
  bool isTimeUp(uint32_t u32_init_cnt);

  bool is_error;
  uint32_t u32_count_timeup = 1000;

};

} // namespace VDT

#endif