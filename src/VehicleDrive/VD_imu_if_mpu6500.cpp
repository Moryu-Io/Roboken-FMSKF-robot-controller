#include "VD_imu_if_mpu6500.hpp"

namespace VDT {

#define WRITE_REG(_reg) (_reg & 0b01111111)
#define READ_REG(_reg) (_reg | 0b10000000)

constexpr uint8_t REG_CONFIG        = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;

constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

void IMU_IF_MPU6500::kickCom(){
    u8_tx_buf[0] = READ_REG(REG_ACCEL_XOUT_H);
    for(int i=1; i<(14+1) ; i++) u8_tx_buf[i] = 0x00;

    kickSpiDma(15);
}

bool IMU_IF_MPU6500::getComData(Data &_d) {
  if(!isComComp()) return false;

  ValReg *p_imuval = (ValReg*)&u8_rx_buf[1]; // buf0はdummyである

  _d.accel_x = (float)((int16_t)((p_imuval->accel_x_h << 8) | p_imuval->accel_x_l)) * LSBtoG;
  _d.accel_y = (float)((int16_t)((p_imuval->accel_y_h << 8) | p_imuval->accel_y_l)) * LSBtoG;
  _d.accel_z = (float)((int16_t)((p_imuval->accel_z_h << 8) | p_imuval->accel_z_l)) * LSBtoG;
  _d.gyro_x  = (float)((int16_t)((p_imuval->gyro_x_h << 8)  | p_imuval->gyro_x_l)) * LSBtoRADPS;
  _d.gyro_y  = (float)((int16_t)((p_imuval->gyro_y_h << 8)  | p_imuval->gyro_y_l)) * LSBtoRADPS;
  _d.gyro_z  = (float)((int16_t)((p_imuval->gyro_z_h << 8)  | p_imuval->gyro_z_l)) * LSBtoRADPS;

  return true;
}

void IMU_IF_MPU6500::getDataImmediately(Data &_d){
    kickCom();
    while(!getComData(_d)){
        ;
    }
}

void IMU_IF_MPU6500::setImuConfigReg(ConfigReg &_cfg) {
  // MPU6500 の場合はConfigレジスタが並んでいるのでこのような形式でOK
  u8_tx_buf[0] = WRITE_REG(REG_CONFIG);
  u8_tx_buf[1] = _cfg.u8_reg_config;
  u8_tx_buf[2] = _cfg.u8_reg_gyro_config;
  u8_tx_buf[3] = _cfg.u8_reg_accel_config;
  u8_tx_buf[4] = _cfg.u8_reg_accel_config2;

  kickSpiDma(5);

  while(!isCompSpiDmaQueue()) {
    ;
  };

}

} // namespace VDT