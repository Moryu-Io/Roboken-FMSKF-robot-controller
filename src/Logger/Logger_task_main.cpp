// RTOS
#include <FreeRTOS_TEENSY4.h>

// C++標準

// Arduinoライブラリ
#include <SD.h>

// ローカル
#include "../Utility/util_led.hpp"
#include "Logger_task_main.hpp"
#include "global_config.hpp"

namespace LGT {

// ローカルパラメータ定義
constexpr uint32_t U32_LOGGER_TASK_CTRL_FREQ_HZ = 50;
constexpr uint32_t U32_SD_WRITE_BUF_LEN         = 4096;

// Peripheral設定
constexpr int SD_CHIP_SELECT = BUILTIN_SDCARD;

// 変数
bool IS_SD_EXIST = false;

// バッファ
uint8_t       U8_SD_WRITE_BUF[2][U32_SD_WRITE_BUF_LEN] = {};
volatile bool IS_SD_WRITE_LOCKED                       = false; // 切り替えの瞬間のみLockする
uint8_t       U8_SD_WRITE_BUF_NOW_PAGE                 = 0;     // 現在書き込んでいるページ
uint32_t      U32_SD_WRITE_BUF_IDX                     = 0;     // 現在書き込んでいるIndex

void prepare_task() {
  if(!SD.begin(SD_CHIP_SELECT)) {
    IS_SD_EXIST = false;
    Serial.println("Card failed, or not present");
  } else {
    /* SDある場合 */
    IS_SD_EXIST = true;
  }
}

void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / U32_LOGGER_TASK_CTRL_FREQ_HZ;

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);

    if(!IS_SD_EXIST) {

    } else {
      /* SDある場合 */

      /* バッファの書き込み */

      /* 書き込みBuffer面の切り替え */
      IS_SD_WRITE_LOCKED       = true;
      U32_SD_WRITE_BUF_IDX     = 0;
      U8_SD_WRITE_BUF_NOW_PAGE = U8_SD_WRITE_BUF_NOW_PAGE ^ 1;
      IS_SD_WRITE_LOCKED       = false;
    }
  }
}

} // namespace LGT