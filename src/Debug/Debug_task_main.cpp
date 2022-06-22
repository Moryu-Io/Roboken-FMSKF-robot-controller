// RTOS
#include <FreeRTOS_TEENSY4.h>

// Arduinoライブラリ
#include <ADC.h>
#include <DMAChannel.h>

// ローカル
#include "../ArmDrive/AD_task_main.hpp"
#include "../FloorDetect/FD_task_main.hpp"
#include "../RobotManager/RM_task_main.hpp"
#include "../Utility/util_led.hpp"
#include "../VehicleDrive/VD_task_main.hpp"
#include "Debug_task_main.hpp"
#include "global_config.hpp"

namespace DEBUG {

#define PRINTBUFFER_LEN (4096)
struct PrintBuffer {
  uint8_t  u8_buf_[PRINTBUFFER_LEN];
  uint32_t u32_head_;
};

PrintBuffer DEBUG_PRINT_BUF[2];
uint8_t     U8_WRITE_PAGE = 0;

// 外部でSprintfするためのBuffer
char EXT_PRINT_BUF[1024];

void process_inputchar();

/**
 * @brief タスク起動前の準備用関数
 *
 */
void prepare_task() {
}

/**
 * @brief タスク処理
 *
 * @param params
 */
void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / 60;

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);

    /* 入力処理 */
    process_inputchar();

    /* 出力処理 */
    /* 書き込みページ切り替え */
    uint8_t _u8_read_page = U8_WRITE_PAGE;
    U8_WRITE_PAGE         = U8_WRITE_PAGE ^ 1;

    if(DEBUG_PRINT_BUF[_u8_read_page].u32_head_ != 0) {
      /* 書き込みデータがある場合 */
      Serial.write(DEBUG_PRINT_BUF[_u8_read_page].u8_buf_, DEBUG_PRINT_BUF[_u8_read_page].u32_head_);
      DEBUG_PRINT_BUF[_u8_read_page].u32_head_ = 0;
    }
  }
}

/**
 * @brief
 *
 * @param _buf
 * @param _size
 */
void print(char *_buf, uint32_t _size) {
  uint32_t _u32_head     = DEBUG_PRINT_BUF[U8_WRITE_PAGE].u32_head_;
  uint32_t _u32_rest_len = PRINTBUFFER_LEN - _u32_head;
  if(_u32_rest_len >= _size) {
    /* バッファ残りより少ない量の書き込みの場合 */
    memcpy(&DEBUG_PRINT_BUF[U8_WRITE_PAGE].u8_buf_[_u32_head], _buf, _size);
    DEBUG_PRINT_BUF[U8_WRITE_PAGE].u32_head_ += _size;
  } else {
    /* バッファ残りより多い場合、書き込める場所まで書き込む */
    memcpy(&DEBUG_PRINT_BUF[U8_WRITE_PAGE].u8_buf_[_u32_head], _buf, _u32_rest_len);
    DEBUG_PRINT_BUF[U8_WRITE_PAGE].u32_head_ += _u32_rest_len;
  }
}

/**
 * @brief ArmDriveTask向けメニュー
 *
 */
static void subproc_adt_menu() {
  Serial.printf("[DEBUG]ADT MENU\n");
  Serial.printf("[DEBUG]f:OFF, i:init, p:positioning, t:posseq\n");
  while(Serial.available() < 1) {};
  char _c = Serial.read();

  ADT::MSG_REQ adt_msg;

  switch(_c) {
  case 'i':
    /* 初期化モードへの遷移指示 */
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::INIT;
    adt_msg.change_mode.u8_forced   = 0;
    ADT::send_req_msg(&adt_msg);
    break;
  case 'f':
    /* OFFモードへの遷移指示(強制) */
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::OFF;
    adt_msg.change_mode.u8_forced   = 1;
    ADT::send_req_msg(&adt_msg);
    break;
  case 'p':
    /* Positioningモードへの遷移指示 */
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::POSITIONING;
    adt_msg.change_mode.u8_forced   = 0;
    ADT::send_req_msg(&adt_msg);
    break;
  case 't':
    /* PositioningSeqモードへの遷移指示 */
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::POSITIONING_SEQ;
    adt_msg.change_mode.u8_forced   = 0;
    ADT::send_req_msg(&adt_msg);
    break;
  default:
    break;
  }
};

/**
 * @brief VehicleDriveTask向けメニュー
 *
 */
static void subproc_vdt_menu() {
  Serial.printf("[DEBUG]VDT MENU\n");
  Serial.printf("[DEBUG]s:stop, hjkl:dir\n");
  while(Serial.available() < 1) {};
  char _c = Serial.read();

  VDT::MSG_REQ vdt_msg;

  switch(_c) {
  case 's':
    /* STOP指示 */
    vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
    vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::MOVE_STOP;
    vdt_msg.move_dir.u32_speed   = 0;
    vdt_msg.move_dir.u32_time_ms = 0;
    VDT::send_req_msg(&vdt_msg);
    break;
  case 'h':
    /* ← 指示 */
    vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
    vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::GO_LEFT;
    vdt_msg.move_dir.u32_speed   = 0;
    vdt_msg.move_dir.u32_time_ms = 100;
    VDT::send_req_msg(&vdt_msg);
    break;
  case 'j':
    /* ← 指示 */
    vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
    vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::GO_BACK;
    vdt_msg.move_dir.u32_speed   = 0;
    vdt_msg.move_dir.u32_time_ms = 100;
    VDT::send_req_msg(&vdt_msg);
    break;
  case 'k':
    /* ← 指示 */
    vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
    vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::GO_FORWARD;
    vdt_msg.move_dir.u32_speed   = 0;
    vdt_msg.move_dir.u32_time_ms = 100;
    VDT::send_req_msg(&vdt_msg);
    break;
  case 'l':
    /* ← 指示 */
    vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
    vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::GO_RIGHT;
    vdt_msg.move_dir.u32_speed   = 0;
    vdt_msg.move_dir.u32_time_ms = 100;
    VDT::send_req_msg(&vdt_msg);
    break;
  default:
    break;
  }
};

/**
 * @brief シリアル入力処理
 *
 */
void process_inputchar() {
  if(Serial.available()) {
    char _c = Serial.read();
    switch(_c) {
    case 'a':
      subproc_adt_menu();
      break;
    case 'v':
      subproc_vdt_menu();
      break;
    case 't':

      break;
    default:
      break;
    }
  }
}

}; // namespace DEBUG
