#ifndef IMU_IF_BASE_HPP_
#define IMU_IF_BASE_HPP_

#include "global_config.hpp"

namespace IMT {

class IMU_IF {
public:
  IMU_IF(){};

  struct Data {
    float accel[3];
    float gyro[3];
    float mag[3];
    float angle[3];
    float qut[4];
  };

  virtual void init() = 0;

  virtual void kickCom()            = 0;
  virtual void update()             = 0;
  virtual bool isComComp()          = 0;
  virtual void getDataLatest(Data &_d) = 0;
  virtual void getDataImmediately(Data &_d) = 0;

  virtual bool isError() = 0;

protected:
};

} // namespace VDT

#endif