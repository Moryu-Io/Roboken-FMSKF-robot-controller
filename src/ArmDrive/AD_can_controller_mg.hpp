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
    this->enableLoopBack(false);
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
    this->onReceive(MB1, rxmb_callback);
    this->onReceive(MB2, rxmb_callback);
    this->setMBUserFilter(MB0, 0x141, 0x7FF << 18); // 受信データのStdIDは0x7F
    this->setMBUserFilter(MB1, 0x141, 0x7FF << 18); // 受信データのStdIDは0x7F
    this->setMBUserFilter(MB2, 0x141, 0x7FF << 18); // 受信データのStdIDは0x7F
    // this->mailboxStatus();
  };

  void tx_routine() {
    CAN_message_t msg;
    msg.id = 0x141;
    if(p_servo_if->get_cantx1_data(msg.buf)){
      /* 送信データがある場合 */
      this->write((FLEXCAN_MAILBOX)(NUM_RX_MAILBOXES_GIMMG_CAN+1), msg);
      u8_now_push_buf = 1;
    }else if(p_servo_if->get_cantx2_data(msg.buf)){
      /* 送信データがある場合 */
      this->write((FLEXCAN_MAILBOX)(NUM_RX_MAILBOXES_GIMMG_CAN+2), msg);
      u8_now_push_buf = 2;
    }
  };

  static void rxmb_callback(const CAN_message_t &msg);
  static JointMgServo *p_servo_if;

  static uint8_t u8_now_push_buf;

};

#if 0
template <CAN_DEV_TABLE _bus>
void CAN_CTRL_MG<_bus>::rxmb_callback(const CAN_message_t &msg){
  if(p_servo_if == NULL) return;

  DEBUG_PRINT_PRC_START(ADT_CAN3);

  p_servo_if->rx_callback((JointMgServo::MgMsgRx *)msg.buf, (int16_t)(micros() & 0x7FFF));

  if(u8_now_push_buf == 1){
    /* MGは2連続CAN送信を受け付けてくれないので、返答があってから2個目を投げる */
    CAN_message_t tx_msg;
    tx_msg.id = 0x141;
    if(p_servo_if->get_cantx2_data(tx_msg.buf)){
      /* 送信データがある場合 */
      this->write((FLEXCAN_MAILBOX)(NUM_RX_MAILBOXES_GIMMG_CAN+2), tx_msg);
      u8_now_push_buf = 2;
    }
  }

  DEBUG_PRINT_PRC_FINISH(ADT_CAN3);

};
#endif

} // namespace ADT

#endif