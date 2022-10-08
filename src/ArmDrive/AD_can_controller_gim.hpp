#ifndef AD_CAN_CONTROLLER_GIM_HPP_
#define AD_CAN_CONTROLLER_GIM_HPP_

#include <FlexCAN_T4.h>

#include "AD_joint_gim_servo.hpp"
#include "global_config.hpp"

namespace ADT {

#define NUM_TX_MAILBOXES_GIM_CAN 2
#define NUM_RX_MAILBOXES_GIM_CAN 1

template <CAN_DEV_TABLE _bus>
class CAN_CTRL_GIM : public FlexCAN_T4<_bus, RX_SIZE_64, TX_SIZE_32> {
public:
public:
  CAN_CTRL_GIM(){};

  void init() {
    this->begin();
    this->setBaudRate(1000000); // 1Mbps
    this->setMaxMB(NUM_TX_MAILBOXES_GIM_CAN + NUM_RX_MAILBOXES_GIM_CAN);

    for(int i = 0; i < NUM_RX_MAILBOXES_GIM_CAN; i++) {
      this->setMB((FLEXCAN_MAILBOX)i, RX, STD);
    }
    for(int i = NUM_RX_MAILBOXES_GIM_CAN; i < (NUM_TX_MAILBOXES_GIM_CAN + NUM_RX_MAILBOXES_GIM_CAN); i++) {
      this->setMB((FLEXCAN_MAILBOX)i, TX, STD);
    }
    this->setMBFilter(REJECT_ALL);
    this->enableMBInterrupts();
    this->onReceive(MB0, rxmb_callback);
    this->setMBUserFilter(MB0, 0x07F, 0x7FF << 18); // 受信データのStdIDは0x7F
    // this->mailboxStatus();
  };

  void tx_routine() {
    CAN_message_t msg;
    msg.id = 0x001;
    if(p_servo_if->get_cantx_data(msg.buf)){
      /* 送信データがある場合 */
      this->write(MB1, msg);
    }
  };

  static void rxmb_callback(const CAN_message_t &msg);
  static JointGimServo *p_servo_if;

};

template <CAN_DEV_TABLE _bus>
void CAN_CTRL_GIM<_bus>::rxmb_callback(const CAN_message_t &msg){
  if(p_servo_if == NULL) return;
  DEBUG_PRINT_PRC_START(ADT_CAN3);

  p_servo_if->rx_callback((JointGimServo::GimMsgRx *)msg.buf, (int16_t)(micros() & 0x7FFF));

  DEBUG_PRINT_PRC_FINISH(ADT_CAN3);
};

} // namespace ADT

#endif