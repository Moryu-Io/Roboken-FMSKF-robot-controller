// RTOS
#include <FreeRTOS_TEENSY4.h>

// Arduinoライブラリ
#include <HardwareSerial.h>

// ローカル
#include "../Utility/util_led.hpp"
#include "AD_joint_ics_servo.hpp"
#include "AD_task_main.hpp"
#include "global_config.hpp"

namespace ADT {

// Peripheral設定
constexpr uint8_t U8_SERIAL_EN_PIN = 32;

// 通信管理
IcsHardSerialClass icsHardSerial(&Serial7, U8_SERIAL_EN_PIN, 115200, 1000);

// 関節管理
JointIcsServo j_Y0;

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

  // Pitch0軸サーボ初期化

  // 手首軸サーボ初期化(CAN)
}

/**
 * @brief タスク処理
 *
 * @param params
 */
void main(void *params) {

  while(1) {
    vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);

    j_Y0.set_tgt_ang_deg(ang_array[ang_array_head]);
    ang_array_head = ang_array_head ^ 1;
    j_Y0.update();

    DEBUG_PRINT_ADT("[ADT]%d,%d\n", micros(), (int)j_Y0.get_now_deg());
  }
}

}; // namespace ADT