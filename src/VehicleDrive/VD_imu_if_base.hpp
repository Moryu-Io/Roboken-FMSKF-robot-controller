#ifndef VD_IMU_IF_BASE_HPP_
#define VD_IMU_IF_BASE_HPP_

#include "global_config.hpp"

namespace VDT {

class IMU_IF {
public:
  IMU_IF(){};

  struct Data {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
  };

  virtual void init() = 0;

  virtual void kickCom()            = 0;
  virtual bool isComComp()          = 0;
  virtual bool getComData(Data &_d) = 0;

  virtual void getDataImmediately(Data &_d) = 0;

protected:
};

} // namespace VDT

#endif