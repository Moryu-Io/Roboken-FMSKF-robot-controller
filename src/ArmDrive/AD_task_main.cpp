// RTOS
#include <FreeRTOS_TEENSY4.h>

// Arduinoライブラリ
#include <HardwareSerial.h>

// ローカル
#include "../Utility/util_led.hpp"
#include "AD_can_controller_gim.hpp"
#include "AD_joint_ics_servo.hpp"
#include "AD_task_main.hpp"
#include "global_config.hpp"

namespace ADT {

// Peripheral設定
constexpr uint8_t U8_SERIAL_EN_PIN = 32;

// 通信管理
IcsHardSerialClass icsHardSerial(&Serial7, U8_SERIAL_EN_PIN, 115200, 10);
CAN_CTRL_GIM<CAN3> GIM_CAN;

// 関節管理
JointIcsServo j_Y0;
JointGimServo j_P1;

// リンク
template <>
JointGimServo *CAN_CTRL_GIM<CAN3>::p_servo_if = &j_P1;

// テスト用
float   ang_array[2]   = {30.0f, -30.0f};
uint8_t ang_array_head = 0;

/**
 * @brief タスク起動前の準備用関数
 *
 */
void prepare_task() {
  // Yaw0軸サーボ初期化
  pinMode(U8_SERIAL_EN_PIN, OUTPUT);
  icsHardSerial.begin();
  j_Y0.init(&icsHardSerial, 1);
  // j_Y0.set_torque_on(true);
  j_Y0.set_torque_on(false);
  j_P1.set_torque_on(true);
  j_P1.set_gain(4095, 100);

  // Pitch0軸サーボ初期化
  GIM_CAN.init();

  // 手首軸サーボ初期化(CAN)
}

/**
 * @brief タスク処理
 *
 * @param params
 */
void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / 1;

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);

    /* CAN系を先に通信する */
    j_P1.set_tgt_ang_deg(ang_array[ang_array_head]);
    j_P1.update();
    GIM_CAN.tx_routine();

    /* UART系 */
    j_Y0.set_tgt_ang_deg(ang_array[ang_array_head]);
    ang_array_head = ang_array_head ^ 1;
    j_Y0.update();

    //DEBUG_PRINT_ADT("[ADT]%d,%d\n", micros(), (int)j_Y0.get_now_deg());
    DEBUG_PRINT_ADT("[ADT]%d,%d\n", micros(), (int)j_P1.get_now_deg());
  }
}

}; // namespace ADT