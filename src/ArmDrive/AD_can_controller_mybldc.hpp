#ifndef AD_CAN_CONTROLLER_MYBLDC_HPP_
#define AD_CAN_CONTROLLER_MYBLDC_HPP_

#include <FlexCAN_T4.h>

#include "AD_joint_mybldc_servo.hpp"
#include "global_config.hpp"

namespace ADT {

#define NUM_TX_MAILBOXES 4
#define NUM_RX_MAILBOXES 4

template <CAN_DEV_TABLE _bus>
class CAN_CTRL_MSV : public FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_256> {
public:
  CAN_CTRL_MSV(){};

  void init() {
    this->begin();
    this->setBaudRate(1000000); // 1Mbps
    this->setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);

    for(int i = 0; i < NUM_RX_MAILBOXES; i++) {
      this->setMB((FLEXCAN_MAILBOX)i, RX, EXT);
    }
    for(int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++) {
      this->setMB((FLEXCAN_MAILBOX)i, TX, EXT);
    }
    this->setMBFilter(REJECT_ALL);
    this->enableMBInterrupts();
    this->onReceive(MB0, rxmb0_callback);
    this->onReceive(MB1, rxmb1_callback);
    this->onReceive(MB2, rxmb2_callback);
    setMBmyFilterExt(MB0, (p_servo_if[0]->get_id() << 18) | 0x1000, (0x7FF << 18) | 0x1000); // Summaryだけに変更。いずれ直す
    setMBmyFilterExt(MB1, (p_servo_if[1]->get_id() << 18) | 0x1000, (0x7FF << 18) | 0x1000);
    setMBmyFilterExt(MB2, (p_servo_if[2]->get_id() << 18) | 0x1000, (0x7FF << 18) | 0x1000);
    // this->mailboxStatus();
  };

  void tx_routine() {
    CAN_message_t msg;
    for(int i = 0; i < 3; i++) {
      if(p_servo_if[i]->get_cantx_data(msg.buf, msg.id)) {
        /* 送信データがある場合 */
        msg.flags.extended = 1;
        this->write((FLEXCAN_MAILBOX)(NUM_RX_MAILBOXES+i), msg);
      }
    }
  };

  static void rxmb0_callback(const CAN_message_t &msg);
  static void rxmb1_callback(const CAN_message_t &msg);
  static void rxmb2_callback(const CAN_message_t &msg);

  static JointMyBldcServo *p_servo_if[3];

private:
  /**
   * @brief 拡張ID用Filter設定
   * @note  ライブラリをそのまま使用すると、拡張ID設定の場合、拡張される下18bitしかIDfilter出来ない
   *        レジスタを直接上書きすることで対処。副作用あるかもしれない
   */
  void setMBmyFilterExt(FLEXCAN_MAILBOX mb_num, uint32_t id1, uint32_t mask) {
    this->setMBUserFilter(mb_num, id1, mask);
    FLEXCANb_MBn_ID(_bus, mb_num) = id1;
  }
};

template <CAN_DEV_TABLE _bus>
void CAN_CTRL_MSV<_bus>::rxmb0_callback(const CAN_message_t &msg) {
  DEBUG_PRINT_PRC_START(ADT_CAN2);

  uint32_t u32_cmdid = msg.id & 0x3FFFF;
  uint16_t u16_devid = msg.id >> 18;

  if(p_servo_if[0]->get_id() == u16_devid) {
    /* 一致するCAN IDのデバイスがある場合 */
    p_servo_if[0]->rx_callback(u32_cmdid, (RES_MESSAGE *)msg.buf, (int16_t)(micros() & 0x7FFF));
  }

  DEBUG_PRINT_PRC_FINISH(ADT_CAN2);
};

template <CAN_DEV_TABLE _bus>
void CAN_CTRL_MSV<_bus>::rxmb1_callback(const CAN_message_t &msg) {
  DEBUG_PRINT_PRC_START(ADT_CAN2);

  uint32_t u32_cmdid = msg.id & 0x3FFFF;
  uint16_t u16_devid = msg.id >> 18;

  if(p_servo_if[1]->get_id() == u16_devid) {
    /* 一致するCAN IDのデバイスがある場合 */
    p_servo_if[1]->rx_callback(u32_cmdid, (RES_MESSAGE *)msg.buf, (int16_t)(micros() & 0x7FFF));
  }

  DEBUG_PRINT_PRC_FINISH(ADT_CAN2);
};

template <CAN_DEV_TABLE _bus>
void CAN_CTRL_MSV<_bus>::rxmb2_callback(const CAN_message_t &msg) {
  DEBUG_PRINT_PRC_START(ADT_CAN2);

  uint32_t u32_cmdid = msg.id & 0x3FFFF;
  uint16_t u16_devid = msg.id >> 18;

  if(p_servo_if[2]->get_id() == u16_devid) {
    /* 一致するCAN IDのデバイスがある場合 */
    p_servo_if[2]->rx_callback(u32_cmdid, (RES_MESSAGE *)msg.buf, (int16_t)(micros() & 0x7FFF));
  }

  DEBUG_PRINT_PRC_FINISH(ADT_CAN2);
};


} // namespace ADT

#endif