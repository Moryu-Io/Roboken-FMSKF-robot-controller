// RTOS
#include <FreeRTOS_TEENSY4.h>
#include <message_buffer.h>

// Arduinoライブラリ
#include <TsyDMASPI.h>

// ローカル
#include "VD_can_controller.hpp"
#include "VD_imu_if_mpu6500.hpp"
#include "VD_motor_if_m2006.hpp"
#include "VD_task_main.hpp"
#include "VD_vehicle_controller.hpp"
#include "global_config.hpp"

namespace VDT {

// Peripheral設定
constexpr uint8_t U8_IMU1_CS_PIN = 37;
constexpr uint8_t U8_IMU2_CS_PIN = 36;

// デバイス設定
class IMU1 : public IMU_IF_MPU6500 {
public:
  void init() override {
    TsyDMASPI0.begin(U8_IMU1_CS_PIN, SPISettings(1000000, MSBFIRST, SPI_MODE3), true);
    LSBtoG     = 1.0f / 16834.0f;
    LSBtoRADPS = 3.141592f / (180.0f * 131.0f);
  }

protected:
  void kickSpiDma(uint8_t size) {
    TsyDMASPI0.queue(u8_tx_buf, u8_rx_buf, (size_t)size, U8_IMU1_CS_PIN);
  };
  bool isCompSpiDmaQueue() {
    return TsyDMASPI0.remained() == 0;
  };
};

// モータIF
MOTOR_IF_M2006 FL_motor(1);
MOTOR_IF_M2006 BL_motor(1);
MOTOR_IF_M2006 BR_motor(-1);
MOTOR_IF_M2006 FR_motor(-1);

// モータ用CAN IF
CAN_CTRL<CAN1> M_CAN;
template <>
MOTOR_IF_M2006 *CAN_CTRL<CAN1>::p_motor_if[4] = {&FL_motor, &BL_motor, &BR_motor, &FR_motor};

// IMU
static IMU1 imu1;

// Vehicleクラス用のパーツ
VEHICLE_CTRL::Parts vhcl_parts = {
    .p_imu   = &imu1,
    .p_motor = {&FL_motor, &BL_motor, &BR_motor, &FR_motor},
};

// インスタンス
static VEHICLE_CTRL vhclCtrl(vhcl_parts);

// RTOS メッセージ
MessageBufferHandle_t p_MsgBufReq;
MSG_REQ               msgReq;

void prepare_task() {
  p_MsgBufReq = xMessageBufferCreate(VDT_MSG_REQ_BUFFER_SIZE * sizeof(MSG_REQ));

  imu1.init();
  M_CAN.init();
}

void main(void *params) {
  uint32_t debug_counter = 0;
  uint32_t loop_tick     = (int)configTICK_RATE_HZ / 1000;

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);

    /* Msg処理 */
    if(xMessageBufferReceive(p_MsgBufReq, (void *)&msgReq, sizeof(MSG_REQ), 0) == sizeof(MSG_REQ)) {
      switch(msgReq.common.MsgId) {
      case REQ_MOVE_DIR:
        DEBUG_PRINT_VDT("[VDT]MOVE_DIR:%d\n", msgReq.move_dir.u32_cmd);

        break;
      default:
        break;
      }
    }

    /* 車体制御Routine */
    vhclCtrl.update();

    /* CAN Routine */
    // M_CAN.events(); // 多分不要
    M_CAN.tx_routine();

    /* 以下、デバッグ用 */
    if(debug_counter == 500) {
      Direction vhcl_pos;
      vhclCtrl.get_vehicle_pos_mm_latest(vhcl_pos);
      // DEBUG_PRINT_VDT_MOTOR("[VDT]%d,%d,%d\n", (int)vhcl_pos.x, (int)vhcl_pos.y, (int)vhcl_pos.th);
      MOTOR_IF_M2006::Status _m_sts[4] = {};
      FL_motor.get_status_latest(_m_sts[0]);
      BL_motor.get_status_latest(_m_sts[1]);
      BR_motor.get_status_latest(_m_sts[2]);
      FR_motor.get_status_latest(_m_sts[3]);
      // DEBUG_PRINT_VDT_MOTOR("[VDT]%d, %d, %d, %d\n", _m_sts[0].s16_rawAngle, _m_sts[1].s16_rawAngle, _m_sts[2].s16_rawAngle, _m_sts[3].s16_rawAngle);
      // DEBUG_PRINT_VDT_MOTOR("[VDT]%d, %d, %d, %d\n", _m_sts[0].s16_rawSpeedRpm, _m_sts[1].s16_rawSpeedRpm, _m_sts[2].s16_rawSpeedRpm, _m_sts[3].s16_rawSpeedRpm);
      // DEBUG_PRINT_VDT_MOTOR("[VDT]%d, %d, %d, %d\n", FL_motor.get_rawAngleSum(), BL_motor.get_rawAngleSum(), BR_motor.get_rawAngleSum(), FR_motor.get_rawAngleSum());
      debug_counter = 0;
    } else {
      debug_counter++;
    }
  }
}

void send_req_msg(MSG_REQ *_msg) {
  xMessageBufferSend(p_MsgBufReq, (void *)_msg, sizeof(MSG_REQ), 0);
}

}; // namespace VDT