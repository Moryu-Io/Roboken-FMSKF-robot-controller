#ifndef AD_CAN_CONTROLLER_MG_HPP_
#define AD_CAN_CONTROLLER_MG_HPP_

#include <FlexCAN_T4.h>

#include "AD_joint_mg_servo.hpp"
#include "global_config.hpp"

namespace ADT {

#define NUM_TX_MAILBOXES_GIMMG_CAN 3
#define NUM_RX_MAILBOXES_GIMMG_CAN 3

template <CAN_DEV_TABLE _bus>
class CAN_CTRL_MG : public FlexCAN_T4<_bus, RX_SIZE_64, TX_SIZE_64> {
public:
public:
  CAN_CTRL_MG(){};

  void init() {
    this->begin();
    this->setBaudRate(1000000); // 1Mbps
    this->setMaxMB(NUM_TX_MAILBOXES_GIMMG_CAN + NUM_RX_MAILBOXES_GIMMG_CAN);

    for(int i = 0; i < NUM_RX_MAILBOXES_GIMMG_CAN; i++) {
      this->setMB((FLEXCAN_MAILBOX)i, RX, STD);
    }
    for(int i = NUM_RX_MAILBOXES_GIMMG_CAN; i < (NUM_TX_MAILBOXES_GIMMG_CAN + NUM_RX_MAILBOXES_GIMMG_CAN); i++) {
      this->setMB((FLEXCAN_MAILBOX)i, TX, STD);
    }
    this->setMBFilter(REJECT_ALL);
    this->enableMBInterrupts();
    this->onReceive(MB0, rxmb_callback);
    this->setMBUserFilter(MB0, 0x141, 0x7FF << 18); // 受信データのStdIDは0x7F
    // this->mailboxStatus();
  };

  void tx_routine() {
    CAN_message_t msg;
    msg.id = 0x141;
    if(p_servo_if->get_cantx1_data(msg.buf)){
      /* 送信データがある場合 */
      this->write((FLEXCAN_MAILBOX)(NUM_RX_MAILBOXES_GIMMG_CAN+1), msg);
    }
  
    if(p_servo_if->get_cantx2_data(msg.buf)){
      /* 送信データがある場合 */
      this->write((FLEXCAN_MAILBOX)(NUM_RX_MAILBOXES_GIMMG_CAN+2), msg);
    }
  };

  static void rxmb_callback(const CAN_message_t &msg);
  static JointMgServo *p_servo_if;

};

template <CAN_DEV_TABLE _bus>
void CAN_CTRL_MG<_bus>::rxmb_callback(const CAN_message_t &msg){
  if(p_servo_if == NULL) return;
  p_servo_if->rx_callback((JointMgServo::MgMsgRx *)msg.buf, (int16_t)(micros() & 0x7FFF));
};

} // namespace ADT

#endif