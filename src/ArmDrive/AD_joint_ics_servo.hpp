#ifndef AD_JOINT_ICS_SERVO_HPP_
#define AD_JOINT_ICS_SERVO_HPP_

#include "IcsHardSerialClass.h"

#include "AD_joint_base.hpp"
#include "global_config.hpp"

namespace ADT {

/**
 * @brief ICS通信方式サーボの関節を駆動するクラス
 * @note  TODO:HardwareSerial+DMAにすることで高速化したい
 * 
 */
class JointIcsServo : public JointBase {
public:
  JointIcsServo() : JointBase(){};

  void update() override;

  void init(IcsHardSerialClass *_p_ics_serial, uint8_t _id);

protected:
  IcsHardSerialClass *p_ics_serial;

  uint8_t u8_id;



};

}; // namespace ADT

#endif