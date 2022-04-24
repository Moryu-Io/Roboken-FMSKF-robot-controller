#ifndef VD_IMU_IF_MPU6500_HPP_
#define VD_IMU_IF_MPU6500_HPP_

#include "VD_imu_if_base.hpp"

namespace VDT {

class IMU_IF_MPU6500 : public IMU_IF {
public:
  IMU_IF_MPU6500() : IMU_IF(){};

  void kickCom() override;
  bool isComComp() override { return isCompSpiDmaQueue(); };
  bool getComData(Data &_d) override;

  void getDataImmediately(Data &_d) override;

  struct ConfigReg {
    uint8_t u8_reg_config;
    uint8_t u8_reg_gyro_config;
    uint8_t u8_reg_accel_config;
    uint8_t u8_reg_accel_config2;
  };

  void setImuConfigReg(ConfigReg &_cfg);

protected:
  virtual void kickSpiDma(uint8_t size) = 0;
  virtual bool isCompSpiDmaQueue()      = 0;

  uint8_t u8_rx_buf[16];
  uint8_t u8_tx_buf[16];

  float LSBtoG;
  float LSBtoRADPS;

  struct ValReg {
    uint8_t accel_x_h;
    uint8_t accel_x_l;
    uint8_t accel_y_h;
    uint8_t accel_y_l;
    uint8_t accel_z_h;
    uint8_t accel_z_l;
    uint8_t tempr_h;
    uint8_t tempr_l;
    uint8_t gyro_x_h;
    uint8_t gyro_x_l;
    uint8_t gyro_y_h;
    uint8_t gyro_y_l;
    uint8_t gyro_z_h;
    uint8_t gyro_z_l;
  };

};

} // namespace VDT

#endif