#ifndef AD_CAN_CONTROLLER_MYBLDC_HPP_
#define AD_CAN_CONTROLLER_MYBLDC_HPP_

#include <FlexCAN_T4.h>

#include "AD_joint_mybldc_servo.hpp"
#include "global_config.hpp"

namespace ADT {

#define NUM_TX_MAILBOXES 3
#define NUM_RX_MAILBOXES 3

template <CAN_DEV_TABLE _bus>
class CAN_CTRL_MSV : public FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_64> {
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
    this->onReceive(MB0, rxmb_callback);
    this->onReceive(MB1, rxmb_callback);
    this->onReceive(MB2, rxmb_callback);
    setMBmyFilterExt(MB0, p_servo_if[0]->get_id()<<18, 0x7FF << 18);
    setMBmyFilterExt(MB1, p_servo_if[1]->get_id()<<18, 0x7FF << 18);
    setMBmyFilterExt(MB2, p_servo_if[2]->get_id()<<18, 0x7FF << 18);
    this->mailboxStatus();
  };

  void tx_routine() {
    CAN_message_t msg;
    for(int i = 0; i < 3; i++) {
      if(p_servo_if[i]->get_cantx_data(msg.buf)) {
        /* 送信データがある場合 */
        msg.id = p_servo_if[i]->get_id();
        this->write(msg);
      }
    }
  };

  static void              rxmb_callback(const CAN_message_t &msg);
  static JointMyBldcServo *p_servo_if[3];

private:
  /**
   * @brief 拡張ID用Filter設定
   * @note  ライブラリをそのまま使用すると、拡張ID設定の場合、拡張される下18bitしかIDfilter出来ない
   *        レジスタを直接上書きすることで対処。副作用あるかもしれない
   */
  void setMBmyFilterExt(FLEXCAN_MAILBOX mb_num, uint32_t id1, uint32_t mask){
    this->setMBUserFilter(mb_num, id1, mask);
    FLEXCANb_MBn_ID(_bus, mb_num) = id1;
  }
};

template <CAN_DEV_TABLE _bus>
void CAN_CTRL_MSV<_bus>::rxmb_callback(const CAN_message_t &msg) {
  uint32_t u32_cmdid = msg.id & 0x3FFFF;
  uint16_t u16_devid = msg.id >> 18;

  for(int i = 0; i < 3; i++) {
    if(p_servo_if[i]->get_id() == u16_devid) {
      /* 一致するCAN IDのデバイスがある場合 */
      p_servo_if[i]->rx_callback(u32_cmdid, (RES_MESSAGE *)msg.buf, (int16_t)(micros() & 0x7FFF));
      break;
    }
  }
};

} // namespace ADT

#endif