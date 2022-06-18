#ifndef VD_CAN_CONTROLLER_HPP_
#define VD_CAN_CONTROLLER_HPP_

#include <FlexCAN_T4.h>

#include "VD_motor_if_m2006.hpp"
#include "global_config.hpp"

namespace VDT {

#define NUM_TX_MAILBOXES 1
#define NUM_RX_MAILBOXES 4

template <CAN_DEV_TABLE _bus>
class CAN_CTRL : public FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16> {
public:
  CAN_CTRL(){};

  void init() {
    this->begin();
    this->setBaudRate(1000000); // 1Mbps
    this->setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);

    for(int i = 0; i < NUM_RX_MAILBOXES; i++) {
      this->setMB((FLEXCAN_MAILBOX)i, RX, STD);
    }
    for(int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++) {
      this->setMB((FLEXCAN_MAILBOX)i, TX, STD);
    }
    this->setMBFilter(REJECT_ALL);
    this->enableMBInterrupts();
    this->onReceive(MB0, mb0_callback);
    this->onReceive(MB1, mb1_callback);
    this->onReceive(MB2, mb2_callback);
    this->onReceive(MB3, mb3_callback);
    this->setMBUserFilter(MB0, 0x201, 0x7FF << 18);
    this->setMBUserFilter(MB1, 0x202, 0x7FF << 18);
    this->setMBUserFilter(MB2, 0x203, 0x7FF << 18);
    this->setMBUserFilter(MB3, 0x204, 0x7FF << 18);
    // this->mailboxStatus();
  };

  void tx_routine(){
    CAN_message_t msg;
    msg.id = 0x200;
    msg.buf[0] = (uint8_t)(p_motor_if[0]->get_rawCurr_tgt() >> 8);
    msg.buf[1] = (uint8_t)(p_motor_if[0]->get_rawCurr_tgt() & 0x00FF);
    msg.buf[2] = (uint8_t)(p_motor_if[1]->get_rawCurr_tgt() >> 8);
    msg.buf[3] = (uint8_t)(p_motor_if[1]->get_rawCurr_tgt() & 0x00FF);
    msg.buf[4] = (uint8_t)(p_motor_if[2]->get_rawCurr_tgt() >> 8);
    msg.buf[5] = (uint8_t)(p_motor_if[2]->get_rawCurr_tgt() & 0x00FF);
    msg.buf[6] = (uint8_t)(p_motor_if[3]->get_rawCurr_tgt() >> 8);
    msg.buf[7] = (uint8_t)(p_motor_if[3]->get_rawCurr_tgt() & 0x00FF);
    this->write(msg);
  };

  static void mb0_callback(const CAN_message_t &msg);
  static void mb1_callback(const CAN_message_t &msg);
  static void mb2_callback(const CAN_message_t &msg);
  static void mb3_callback(const CAN_message_t &msg);

  static MOTOR_IF_M2006 *p_motor_if[4];
};

template <CAN_DEV_TABLE _bus>
void CAN_CTRL<_bus>::mb0_callback(const CAN_message_t &msg) {
  if(p_motor_if[0] == NULL) return;
  p_motor_if[0]->rx_callback((MOTOR_IF_M2006::CanMsgRx *)msg.buf, (int16_t)(micros() & 0x7FFF));
};

template <CAN_DEV_TABLE _bus>
void CAN_CTRL<_bus>::mb1_callback(const CAN_message_t &msg) {
  if(p_motor_if[1] == NULL) return;
  p_motor_if[1]->rx_callback((MOTOR_IF_M2006::CanMsgRx *)msg.buf, (int16_t)(micros() & 0x7FFF));
};

template <CAN_DEV_TABLE _bus>
void CAN_CTRL<_bus>::mb2_callback(const CAN_message_t &msg) {
  if(p_motor_if[2] == NULL) return;
  p_motor_if[2]->rx_callback((MOTOR_IF_M2006::CanMsgRx *)msg.buf, (int16_t)(micros() & 0x7FFF));
};

template <CAN_DEV_TABLE _bus>
void CAN_CTRL<_bus>::mb3_callback(const CAN_message_t &msg) {
  if(p_motor_if[3] == NULL) return;
  p_motor_if[3]->rx_callback((MOTOR_IF_M2006::CanMsgRx *)msg.buf, (int16_t)(micros() & 0x7FFF));
};

} // namespace VDT

#endif